/*
 * Real-Time Scheduling Class (mapped to the SCHED_FIFO and SCHED_RR
 * policies)
 */

#include "sched.h"

#include <linux/slab.h>
#include <linux/irq_work.h>

#include "walt.h"

int sched_rr_timeslice = RR_TIMESLICE;

#if defined(CONFIG_SMP) && defined(HAVE_RT_PUSH_IPI)
static void push_irq_work_func(struct irq_work *work);
#endif

void init_rt_rq(struct rt_rq *rt_rq)
{
	struct rt_prio_array *array;
	int i;

	array = &rt_rq->active;
	for (i = 0; i < MAX_RT_PRIO; i++) {
		INIT_LIST_HEAD(array->queue + i);
		__clear_bit(i, array->bitmap);
	}
	/* delimiter for bitsearch: */
	__set_bit(MAX_RT_PRIO, array->bitmap);

#if defined CONFIG_SMP
	rt_rq->highest_prio.curr = MAX_RT_PRIO;
	rt_rq->highest_prio.next = MAX_RT_PRIO;
	rt_rq->rt_nr_migratory = 0;
	rt_rq->overloaded = 0;
	plist_head_init(&rt_rq->pushable_tasks);

#ifdef HAVE_RT_PUSH_IPI
	rt_rq->push_flags = 0;
	rt_rq->push_cpu = nr_cpu_ids;
	raw_spin_lock_init(&rt_rq->push_lock);
	init_irq_work(&rt_rq->push_work, push_irq_work_func);
#endif
#endif /* CONFIG_SMP */
#ifdef CONFIG_RT_GROUP_SCHED
	INIT_LIST_HEAD(&rt_rq->cfs_throttled_tasks);
#endif
}

#ifdef CONFIG_RT_GROUP_SCHED

static inline struct rq *rq_of_rt_rq(struct rt_rq *rt_rq)
{
	return rt_rq->rq;
}

void free_rt_sched_group(struct task_group *tg)
{
	unsigned long flags;
	int i;

	for_each_possible_cpu(i) {
		if (tg->rt_rq)
			kfree(tg->rt_rq[i]);
		if (tg->dl_se) {
			raw_spin_lock_irqsave(&cpu_rq(i)->lock, flags);
			if (!tg->dl_se[i]->dl_throttled)
				dequeue_dl_entity(tg->dl_se[i]);
			raw_spin_unlock_irqrestore(&cpu_rq(i)->lock, flags);

			hrtimer_cancel(&tg->dl_se[i]->dl_timer);
			kfree(tg->dl_se[i]);
		}
	}

	kfree(tg->rt_rq);
	kfree(tg->dl_se);
}

void init_tg_rt_entry(struct task_group *tg, struct rt_rq *rt_rq,
		struct sched_dl_entity *dl_se, int cpu,
		struct sched_dl_entity *parent)
{
	struct rq *rq = cpu_rq(cpu);

	rt_rq->highest_prio.curr = MAX_RT_PRIO;
	rt_rq->rq = rq;
	rt_rq->tg = tg;

	tg->rt_rq[cpu] = rt_rq;
	tg->dl_se[cpu] = dl_se;

	if (!dl_se)
		return;

	dl_se->dl_rq = &rq->dl;
	dl_se->my_q = rt_rq;
	RB_CLEAR_NODE(&dl_se->rb_node);
}

int alloc_rt_sched_group(struct task_group *tg, struct task_group *parent)
{
	struct rt_rq *rt_rq;
	struct sched_dl_entity *dl_se;
	int i;

	tg->rt_rq = kcalloc(nr_cpu_ids, sizeof(rt_rq), GFP_KERNEL);
	if (!tg->rt_rq)
		goto err;
	tg->dl_se = kcalloc(nr_cpu_ids, sizeof(dl_se), GFP_KERNEL);
	if (!tg->dl_se)
		goto err;

	init_dl_bandwidth(&tg->dl_bandwidth,
			def_dl_bandwidth.dl_period, 0);

	for_each_possible_cpu(i) {
		rt_rq = kzalloc_node(sizeof(struct rt_rq),
				     GFP_KERNEL, cpu_to_node(i));
		if (!rt_rq)
			goto err;

		dl_se = kzalloc_node(sizeof(struct sched_dl_entity),
				     GFP_KERNEL, cpu_to_node(i));
		if (!dl_se)
			goto err_free_rq;

		init_rt_rq(rt_rq);
		rt_rq->rq = cpu_rq(i);

		init_dl_task_timer(dl_se);

		dl_se->dl_runtime = tg->dl_bandwidth.dl_runtime;
		dl_se->dl_period = tg->dl_bandwidth.dl_period;
		dl_se->dl_deadline = dl_se->dl_period;
		dl_se->dl_bw = to_ratio(dl_se->dl_period, dl_se->dl_runtime);

		dl_se->dl_throttled = 0;

		init_tg_rt_entry(tg, rt_rq, dl_se, i, parent->dl_se[i]);
	}

	return 1;

err_free_rq:
	kfree(rt_rq);
err:
	return 0;
}

#else /* CONFIG_RT_GROUP_SCHED */

static inline struct rq *rq_of_rt_rq(struct rt_rq *rt_rq)
{
	return container_of(rt_rq, struct rq, rt);
}

void free_rt_sched_group(struct task_group *tg) { }

int alloc_rt_sched_group(struct task_group *tg, struct task_group *parent)
{
	return 1;
}
#endif /* CONFIG_RT_GROUP_SCHED */

#ifdef CONFIG_SMP

static void pull_rt_task(struct rq *this_rq);

static inline bool need_pull_rt_task(struct rq *rq, struct task_struct *prev)
{
	/* Try to pull RT tasks here if we lower this rq's prio */
	return rq->rt.highest_prio.curr > prev->prio;
}

static inline int rt_overloaded(struct rq *rq)
{
	return atomic_read(&rq->rd->rto_count);
}

static inline void rt_set_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	cpumask_set_cpu(rq->cpu, rq->rd->rto_mask);
	/*
	 * Make sure the mask is visible before we set
	 * the overload count. That is checked to determine
	 * if we should look at the mask. It would be a shame
	 * if we looked at the mask, but the mask was not
	 * updated yet.
	 *
	 * Matched by the barrier in pull_rt_task().
	 */
	smp_wmb();
	atomic_inc(&rq->rd->rto_count);
}

static inline void rt_clear_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	/* the order here really doesn't matter */
	atomic_dec(&rq->rd->rto_count);
	cpumask_clear_cpu(rq->cpu, rq->rd->rto_mask);
}

static void update_rt_migration(struct rt_rq *rt_rq)
{
	if (rt_rq->rt_nr_migratory && rt_rq->rt_nr_running > 1) {
		if (!rt_rq->overloaded) {
			rt_set_overload(rq_of_rt_rq(rt_rq));
			rt_rq->overloaded = 1;
		}
	} else if (rt_rq->overloaded) {
		rt_clear_overload(rq_of_rt_rq(rt_rq));
		rt_rq->overloaded = 0;
	}
}

static void inc_rt_migration(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
	struct task_struct *p;

	p = rt_task_of(rt_se);

	if (tsk_nr_cpus_allowed(p) > 1)
		rt_rq->rt_nr_migratory++;

	update_rt_migration(rt_rq);
}

static void dec_rt_migration(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
	struct task_struct *p;

	p = rt_task_of(rt_se);

	if (tsk_nr_cpus_allowed(p) > 1)
		rt_rq->rt_nr_migratory--;

	update_rt_migration(rt_rq);
}

static inline int has_pushable_tasks(struct rt_rq *rt_rq)
{
	return !plist_head_empty(&rt_rq->pushable_tasks);
}

static DEFINE_PER_CPU(struct callback_head, rt_push_head);
static DEFINE_PER_CPU(struct callback_head, rt_pull_head);

static void push_rt_tasks(struct rq *);
static void pull_rt_task(struct rq *);

static inline void queue_push_tasks(struct rq *rq)
{
	if (!has_pushable_tasks(&rq->rt))
		return;

	queue_balance_callback(rq, &per_cpu(rt_push_head, rq->cpu), push_rt_tasks);
}

static inline void queue_pull_task(struct rq *rq)
{
	queue_balance_callback(rq, &per_cpu(rt_pull_head, rq->cpu), pull_rt_task);
}

static void enqueue_pushable_task(struct rt_rq *rt_rq, struct task_struct *p)
{
	plist_del(&p->pushable_tasks, &rt_rq->pushable_tasks);
	plist_node_init(&p->pushable_tasks, p->prio);
	plist_add(&p->pushable_tasks, &rt_rq->pushable_tasks);

	/* Update the highest prio pushable task */
	if (p->prio < rt_rq->highest_prio.next)
		rt_rq->highest_prio.next = p->prio;
}

#ifdef CONFIG_RT_GROUP_SCHED
void dequeue_pushable_task(struct rt_rq *rt_rq, struct task_struct *p)
{
	plist_del(&p->pushable_tasks, &rt_rq->pushable_tasks);

	/* Update the new highest prio pushable task */
	if (has_pushable_tasks(rt_rq)) {
		p = plist_first_entry(&rt_rq->pushable_tasks,
				      struct task_struct, pushable_tasks);
		rt_rq->highest_prio.next = p->prio;
	} else
		rt_rq->highest_prio.next = MAX_RT_PRIO;
}
#endif
#else

static inline
void enqueue_pushable_task(struct rt_rq *rt_rq, struct task_struct *p)
{
}

static inline
void inc_rt_migration(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
}

static inline
void dec_rt_migration(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
}

static inline bool need_pull_rt_task(struct rq *rq, struct task_struct *prev)
{
	return false;
}

static inline void pull_rt_task(struct rq *this_rq)
{
}

static inline void queue_push_tasks(struct rq *rq)
{
}

static inline void queue_pull_task(struct rq *rq)
{
}
#endif /* CONFIG_SMP */

static inline int on_rt_rq(struct sched_rt_entity *rt_se)
{
	return rt_se->on_rq;
}

static inline int rt_se_prio(struct sched_rt_entity *rt_se)
{
	return rt_task_of(rt_se)->prio;
}

/*
 * Iterates through all the tasks on @rt_rq and, depending on @enqueue, moves
 * them between FIFO and OTHER.
 */
static void cfs_throttle_rt_tasks(struct rt_rq *rt_rq)
{
	struct rt_prio_array *array = &rt_rq->active;
	struct rq *rq = rq_of_rt_rq(rt_rq);
	int idx;
	struct list_head *pos, *n;

	if (bitmap_empty(array->bitmap, MAX_RT_PRIO))
		return;

	idx = sched_find_first_bit(array->bitmap);
	while (idx < MAX_RT_PRIO) {
		list_for_each_safe(pos, n, array->queue + idx) {
			struct sched_rt_entity *rt_se;
			struct task_struct *p;

			rt_se = list_entry(pos, struct sched_rt_entity, run_list);

			p = rt_task_of(rt_se);
			/*
			 * Don't enqueue in fair if the task is going
			 * to sleep. We'll handle the transition at
			 * wakeup time eventually.
			 */
			if (p->state != TASK_RUNNING)
				continue;

			list_add(&rt_se->cfs_throttled_task,
				 &rt_rq->cfs_throttled_tasks);
			__setprio_other(rq, p);
		}
		idx = find_next_bit(array->bitmap, MAX_RT_PRIO, idx + 1);
	}
}

void cfs_unthrottle_rt_tasks(struct rt_rq *rt_rq)
{
	struct rq *rq = rq_of_rt_rq(rt_rq);

	while (!list_empty(&rt_rq->cfs_throttled_tasks)) {
		struct sched_rt_entity *rt_se;
		struct task_struct *p;

		rt_se = list_first_entry(&rt_rq->cfs_throttled_tasks,
					 struct sched_rt_entity,
					 cfs_throttled_task);

		p = rt_task_of(rt_se);
		list_del_init(&rt_se->cfs_throttled_task);
		__setprio_fifo(rq, p);
	}
}

/*
 * Update the current task's runtime statistics. Skip current tasks that
 * are not in our scheduling class.
 */
static void update_curr_rt(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct rt_rq *rt_rq = rt_rq_of_se(&curr->rt);
	u64 delta_exec;

	if (curr->sched_class != &rt_sched_class)
		return;

	delta_exec = rq_clock_task(rq) - curr->se.exec_start;
	if (unlikely((s64)delta_exec <= 0))
		return;

	/* Kick cpufreq (see the comment in kernel/sched/sched.h). */
	cpufreq_update_this_cpu(rq, SCHED_CPUFREQ_RT);

	schedstat_set(curr->se.statistics.exec_max,
		      max(curr->se.statistics.exec_max, delta_exec));

	curr->se.sum_exec_runtime += delta_exec;
	account_group_exec_runtime(curr, delta_exec);

	curr->se.exec_start = rq_clock_task(rq);
	cpuacct_charge(curr, delta_exec);

	sched_rt_avg_update(rq, delta_exec);

	if (!dl_bandwidth_enabled())
		return;

	if (is_dl_group(rt_rq)) {
		struct sched_dl_entity *dl_se = dl_group_of(rt_rq);

		if (dl_se->dl_throttled) {
			resched_curr(rq);
			return;
		}

		BUG_ON(rt_rq->rt_nr_running > rq->nr_running);
		dl_se->runtime -= delta_exec;

		/* A group exhausts the budget. */
		if (dl_runtime_exceeded(dl_se)) {
			dequeue_dl_entity(dl_se);

			if (likely(start_dl_timer(dl_se))) {
				dl_se->dl_throttled = 1;
				cfs_throttle_rt_tasks(rt_rq);
			} else
				enqueue_dl_entity(dl_se, dl_se,
						  ENQUEUE_REPLENISH);

			resched_curr(rq);
		}
	}
}

#if defined CONFIG_SMP

static void
inc_rt_prio_smp(struct rt_rq *rt_rq, int prio, int prev_prio)
{
	struct rq *rq = rq_of_rt_rq(rt_rq);

	if (is_dl_group(rt_rq))
		return;

	if (rq->online && prio < prev_prio)
		cpupri_set(&rq->rd->cpupri, rq->cpu, prio);
}

static void
dec_rt_prio_smp(struct rt_rq *rt_rq, int prio, int prev_prio)
{
	struct rq *rq = rq_of_rt_rq(rt_rq);

	if (is_dl_group(rt_rq))
		return;

	if (rq->online && rt_rq->highest_prio.curr != prev_prio)
		cpupri_set(&rq->rd->cpupri, rq->cpu, rt_rq->highest_prio.curr);
}

#else /* CONFIG_SMP */

static inline
void inc_rt_prio_smp(struct rt_rq *rt_rq, int prio, int prev_prio) {}
static inline
void dec_rt_prio_smp(struct rt_rq *rt_rq, int prio, int prev_prio) {}

#endif /* CONFIG_SMP */

#if defined(CONFIG_SMP)
static void
inc_rt_prio(struct rt_rq *rt_rq, int prio)
{
	int prev_prio = rt_rq->highest_prio.curr;

	if (is_dl_group(rt_rq))
		return;

	if (prio < prev_prio)
		rt_rq->highest_prio.curr = prio;

	inc_rt_prio_smp(rt_rq, prio, prev_prio);
}

static void
dec_rt_prio(struct rt_rq *rt_rq, int prio)
{
	int prev_prio = rt_rq->highest_prio.curr;

	if (is_dl_group(rt_rq))
		return;

	if (rt_rq->rt_nr_running) {

		WARN_ON(prio < prev_prio);

		/*
		 * This may have been our highest task, and therefore
		 * we may have some recomputation to do
		 */
		if (prio == prev_prio) {
			struct rt_prio_array *array = &rt_rq->active;

			rt_rq->highest_prio.curr =
				sched_find_first_bit(array->bitmap);
		}

	} else
		rt_rq->highest_prio.curr = MAX_RT_PRIO;

	dec_rt_prio_smp(rt_rq, prio, prev_prio);
}

#else

static inline void inc_rt_prio(struct rt_rq *rt_rq, int prio) {}
static inline void dec_rt_prio(struct rt_rq *rt_rq, int prio) {}

#endif /* CONFIG_SMP && !CONFIG_RT_GROUP_SCHED */

static inline
unsigned int rt_se_nr_running(struct sched_rt_entity *rt_se)
{
	return 1;
}

static inline
void inc_rt_tasks(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
	int prio = rt_se_prio(rt_se);

	WARN_ON(!rt_prio(prio));
	rt_rq->rt_nr_running += rt_se_nr_running(rt_se);

	inc_rt_prio(rt_rq, prio);

	if (is_dl_group(rt_rq)) {
		struct sched_dl_entity *dl_se = dl_group_of(rt_rq);

		if (!dl_se->dl_throttled)
			add_nr_running(rq_of_rt_rq(rt_rq), 1);
	} else {
		add_nr_running(rq_of_rt_rq(rt_rq), 1);
	}

	inc_rt_migration(rt_se, rt_rq);
}

static inline
void dec_rt_tasks(struct sched_rt_entity *rt_se, struct rt_rq *rt_rq)
{
	WARN_ON(!rt_prio(rt_se_prio(rt_se)));
	rt_rq->rt_nr_running -= rt_se_nr_running(rt_se);

	dec_rt_prio(rt_rq, rt_se_prio(rt_se));
	if (is_dl_group(rt_rq)) {
		struct sched_dl_entity *dl_se = dl_group_of(rt_rq);

		if (!dl_se->dl_throttled)
			sub_nr_running(rq_of_rt_rq(rt_rq), 1);
	} else {
		sub_nr_running(rq_of_rt_rq(rt_rq), 1);
	}
	dec_rt_migration(rt_se, rt_rq);
}

/*
 * Change rt_se->run_list location unless SAVE && !MOVE
 *
 * assumes ENQUEUE/DEQUEUE flags match
 */
static inline bool move_entity(unsigned int flags)
{
	if ((flags & (DEQUEUE_SAVE | DEQUEUE_MOVE)) == DEQUEUE_SAVE)
		return false;

	return true;
}

static void __delist_rt_entity(struct sched_rt_entity *rt_se, struct rt_prio_array *array)
{
	list_del_init(&rt_se->run_list);

	if (list_empty(array->queue + rt_se_prio(rt_se)))
		__clear_bit(rt_se_prio(rt_se), array->bitmap);

	rt_se->on_list = 0;
}

static void enqueue_rt_entity(struct sched_rt_entity *rt_se, unsigned int flags)
{
	struct rt_rq *rt_rq = rt_rq_of_se(rt_se);
	struct rt_prio_array *array = &rt_rq->active;
	struct list_head *queue = array->queue + rt_se_prio(rt_se);

	if (move_entity(flags)) {
		WARN_ON_ONCE(rt_se->on_list);
		if (flags & ENQUEUE_HEAD)
			list_add(&rt_se->run_list, queue);
		else
			list_add_tail(&rt_se->run_list, queue);

		__set_bit(rt_se_prio(rt_se), array->bitmap);
		rt_se->on_list = 1;
	}
	rt_se->on_rq = 1;

	inc_rt_tasks(rt_se, rt_rq);
}

static void dequeue_rt_entity(struct sched_rt_entity *rt_se, unsigned int flags)
{
	struct rt_rq *rt_rq = rt_rq_of_se(rt_se);
	struct rt_prio_array *array = &rt_rq->active;

	if (move_entity(flags)) {
		WARN_ON_ONCE(!rt_se->on_list);
		__delist_rt_entity(rt_se, array);
	}
	rt_se->on_rq = 0;

	dec_rt_tasks(rt_se, rt_rq);
}

/*
 * Adding/removing a task to/from a priority array:
 */
static void
enqueue_task_rt(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_rt_entity *rt_se = &p->rt;
	struct rt_rq *rt_rq = rt_rq_of_se(rt_se);

	if (is_dl_group(rt_rq)) {
		BUG_ON(	(rt_rq->rt_nr_running == 0) &&
			(!RB_EMPTY_NODE(&dl_group_of(rt_rq)->rb_node))
		);
	}
	
	if (flags & ENQUEUE_WAKEUP)
		rt_se->timeout = 0;

	/* Task arriving in an idle group of tasks. */
	if (is_dl_group(rt_rq) && (rt_rq->rt_nr_running == 0)) {
		struct sched_dl_entity *dl_se = dl_group_of(rt_rq);

		if (!dl_se->dl_throttled) {
			enqueue_dl_entity(dl_se, dl_se, flags);
			resched_curr(rq);
		} else {
			BUG_ON(rt_throttled(p));
			/*
			 * rt_se's group was throttled while this task was
			 * sleeping/blocked/migrated.
			 *
			 * Do the transition towards OTHER now.
			 */
			if ((flags & ENQUEUE_REPLENISH) == 0) {
				BUG_ON(on_rt_rq(rt_se));
				lockdep_assert_held(&rq->lock);

				list_add(&rt_se->cfs_throttled_task,
					&rt_rq->cfs_throttled_tasks);
				rt_se->cfs_throttle_rt_rq = rt_rq;
				p->sched_class = &fair_sched_class;
				p->prio = DEFAULT_PRIO;
				p->sched_class->enqueue_task(rq, p, flags);
				p->sched_class->switched_to(rq, p);

				return;
			}
		}
	}

	BUG_ON(p->sched_class != &rt_sched_class);
	enqueue_rt_entity(rt_se, flags);
	walt_inc_cumulative_runnable_avg(rq, p);

	if (!task_current(rq, p) && tsk_nr_cpus_allowed(p) > 1)
		enqueue_pushable_task(rt_rq, p);
}

static void dequeue_task_rt(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_rt_entity *rt_se = &p->rt;
	struct rt_rq *rt_rq = rt_rq_of_se(rt_se);

	if (!rt_throttled(p))
		update_curr_rt(rq);
	if (p->sched_class != &rt_sched_class) {
		p->sched_class->dequeue_task(rq, p, flags);
		return;
	}
	dequeue_rt_entity(rt_se, flags);
	walt_dec_cumulative_runnable_avg(rq, p);

	dequeue_pushable_task(rt_rq_of_se(rt_se), p);

	/* Last task of the task group. */
	if (is_dl_group(rt_rq) && !rt_rq->rt_nr_running) {
		struct sched_dl_entity *dl_se = dl_group_of(rt_rq);

#ifndef CONFIG_RT_GROUP_SCHED
		queue_pull_task(rq);
#endif
		if (!rt_rq->rt_nr_running) {
			dequeue_dl_entity(dl_se);
			resched_curr(rq);
		}
	}
}

/*
 * Put task to the head or the end of the run list without the overhead of
 * dequeue followed by enqueue.
 */
static void
requeue_rt_entity(struct rt_rq *rt_rq, struct sched_rt_entity *rt_se, int head)
{
	if (on_rt_rq(rt_se)) {
		struct rt_prio_array *array = &rt_rq->active;
		struct list_head *queue = array->queue + rt_se_prio(rt_se);

		if (head)
			list_move(&rt_se->run_list, queue);
		else
			list_move_tail(&rt_se->run_list, queue);
	}
	
	if (is_dl_group(rt_rq)) {
		BUG_ON(	(rt_rq->rt_nr_running == 0) &&
			(!RB_EMPTY_NODE(&dl_group_of(rt_rq)->rb_node))
		);
	}
}

static void requeue_task_rt(struct rq *rq, struct task_struct *p, int head)
{
	struct sched_rt_entity *rt_se = &p->rt;
	struct rt_rq *rt_rq;

	rt_rq = rt_rq_of_se(rt_se);
	requeue_rt_entity(rt_rq, rt_se, head);
	
	if (is_dl_group(rt_rq)) {
		BUG_ON(	(rt_rq->rt_nr_running == 0) &&
			(!RB_EMPTY_NODE(&dl_group_of(rt_rq)->rb_node))
		);
	}
}

static void yield_task_rt(struct rq *rq)
{
	requeue_task_rt(rq, rq->curr, 0);
}

#ifdef CONFIG_SMP
static int find_lowest_rq(struct task_struct *task);

static int
select_task_rq_rt(struct task_struct *p, int cpu, int sd_flag, int flags)
{
	struct task_struct *curr;
	struct rq *rq;

	/* For anything but wake ups, just return the task_cpu */
	if (sd_flag != SD_BALANCE_WAKE && sd_flag != SD_BALANCE_FORK)
		goto out;

	rq = cpu_rq(cpu);

	rcu_read_lock();
	curr = READ_ONCE(rq->curr); /* unlocked access */

	/*
	 * If the current task on @p's runqueue is an RT task, then
	 * try to see if we can wake this RT task up on another
	 * runqueue. Otherwise simply start this RT task
	 * on its current runqueue.
	 *
	 * We want to avoid overloading runqueues. If the woken
	 * task is a higher priority, then it will stay on this CPU
	 * and the lower prio task should be moved to another CPU.
	 * Even though this will probably make the lower prio task
	 * lose its cache, we do not want to bounce a higher task
	 * around just because it gave up its CPU, perhaps for a
	 * lock?
	 *
	 * For equal prio tasks, we just let the scheduler sort it out.
	 *
	 * Otherwise, just let it ride on the affined RQ and the
	 * post-schedule router will push the preempted task away
	 *
	 * This test is optimistic, if we get it wrong the load-balancer
	 * will have to sort it out.
	 */
	if (curr && unlikely(rt_task(curr)) &&
	    (tsk_nr_cpus_allowed(curr) < 2 ||
	     curr->prio <= p->prio)) {
		int target = find_lowest_rq(p);

		/*
		 * Don't bother moving it if the destination CPU is
		 * not running a lower priority task.
		 */
		if (target != -1 &&
		    p->prio < cpu_rq(target)->rt.highest_prio.curr)
			cpu = target;
	}
	rcu_read_unlock();

out:
	return cpu;
}

static void check_preempt_equal_prio(struct rq *rq, struct task_struct *p)
{
	/*
	 * Current can't be migrated, useless to reschedule,
	 * let's hope p can move out.
	 */
	if (tsk_nr_cpus_allowed(rq->curr) == 1 ||
	    !cpupri_find(&rq->rd->cpupri, rq->curr, NULL))
		return;

	/*
	 * p is migratable, so let's not schedule it and
	 * see if it is pushed or pulled somewhere else.
	 */
	if (tsk_nr_cpus_allowed(p) != 1
	    && cpupri_find(&rq->rd->cpupri, p, NULL))
		return;

	/*
	 * There appears to be other cpus that can accept
	 * current and none to run 'p', so lets reschedule
	 * to try and push current away:
	 */
	requeue_task_rt(rq, p, 1);
	resched_curr(rq);
}

#endif /* CONFIG_SMP */

/*
 * Preempt the current task with a newly woken task if needed:
 */
static void check_preempt_curr_rt(struct rq *rq, struct task_struct *p, int flags)
{
	if (is_dl_group(rt_rq_of_se(&p->rt)) &&
	    is_dl_group(rt_rq_of_se(&rq->curr->rt))) {
		struct sched_dl_entity *dl_se, *curr_dl_se;

		dl_se = dl_group_of(rt_rq_of_se(&p->rt));
		curr_dl_se = dl_group_of(rt_rq_of_se(&rq->curr->rt));

		if (dl_entity_preempt(dl_se, curr_dl_se)) {
			resched_curr(rq);
			return;
		} else if (!dl_entity_preempt(curr_dl_se, dl_se)) {
			if (p->prio < rq->curr->prio) {
				resched_curr(rq);
				return;
			}
		}
		return;
	} else if (is_dl_group(rt_rq_of_se(&p->rt))) {
		resched_curr(rq);
		return;
	} else if (is_dl_group(rt_rq_of_se(&rq->curr->rt))) {
		return;
	}

	if (p->prio < rq->curr->prio) {
		resched_curr(rq);
		return;
	}

#ifdef CONFIG_SMP
	/*
	 * If:
	 *
	 * - the newly woken task is of equal priority to the current task
	 * - the newly woken task is non-migratable while current is migratable
	 * - current will be preempted on the next reschedule
	 *
	 * we should check to see if current can readily move to a different
	 * cpu.  If so, we will reschedule to allow the push logic to try
	 * to move current somewhere else, making room for our non-migratable
	 * task.
	 */
	if (p->prio == rq->curr->prio && !test_tsk_need_resched(rq->curr))
		check_preempt_equal_prio(rq, p);
#endif
}

#ifdef CONFIG_SMP
static void sched_rt_update_capacity_req(struct rq *rq)
{
	u64 total, used, age_stamp, avg;
	s64 delta;

	if (!sched_freq())
		return;

	sched_avg_update(rq);
	/*
	 * Since we're reading these variables without serialization make sure
	 * we read them once before doing sanity checks on them.
	 */
	age_stamp = READ_ONCE(rq->age_stamp);
	avg = READ_ONCE(rq->rt_avg);
	delta = rq_clock(rq) - age_stamp;

	if (unlikely(delta < 0))
		delta = 0;

	total = sched_avg_period() + delta;

	used = div_u64(avg, total);
	if (unlikely(used > SCHED_CAPACITY_SCALE))
		used = SCHED_CAPACITY_SCALE;

	set_rt_cpu_capacity(rq->cpu, 1, (unsigned long)(used));
}
#else
static inline void sched_rt_update_capacity_req(struct rq *rq)
{ }

#endif

struct sched_rt_entity *pick_next_rt_entity(struct rq *rq,
						   struct rt_rq *rt_rq)
{
	struct rt_prio_array *array = &rt_rq->active;
	struct sched_rt_entity *next = NULL;
	struct list_head *queue;
	int idx;

	idx = sched_find_first_bit(array->bitmap);
	BUG_ON(idx >= MAX_RT_PRIO);

	queue = array->queue + idx;
	next = list_entry(queue->next, struct sched_rt_entity, run_list);

	return next;
}

static struct task_struct *_pick_next_task_rt(struct rq *rq)
{
	struct sched_rt_entity *rt_se;
	struct task_struct *p;
	struct rt_rq *rt_rq  = &rq->rt;

	rt_se = pick_next_rt_entity(rq, rt_rq);
	BUG_ON(!rt_se);

	p = rt_task_of(rt_se);
	p->se.exec_start = rq_clock_task(rq);

	return p;
}

static struct task_struct *
pick_next_task_rt(struct rq *rq, struct task_struct *prev, struct pin_cookie cookie)
{
	struct task_struct *p;
	struct rt_rq *rt_rq = &rq->rt;

	if (need_pull_rt_task(rq, prev)) {
		/*
		 * This is OK, because current is on_cpu, which avoids it being
		 * picked for load-balance and preemption/IRQs are still
		 * disabled avoiding further scheduler activity on it and we're
		 * being very careful to re-start the picking loop.
		 */
		lockdep_unpin_lock(&rq->lock, cookie);
		pull_rt_task(rq);
		lockdep_repin_lock(&rq->lock, cookie);
		/*
		 * pull_rt_task() can drop (and re-acquire) rq->lock; this
		 * means a dl or stop task can slip in, in which case we need
		 * to re-start task selection.
		 */
		if (unlikely((rq->stop && task_on_rq_queued(rq->stop)) ||
			     rq->dl.dl_nr_running))
			return RETRY_TASK;
	}

	/*
	 * We may dequeue prev's rt_rq in put_prev_task().
	 * So, we update time before rt_nr_running check.
	 */
	if (prev->sched_class == &rt_sched_class)
		update_curr_rt(rq);

	if (!rt_rq->rt_nr_running) {
		/*
		 * The next task to be picked on this rq will have a lower
		 * priority than rt tasks so we can spend some time to update
		 * the capacity used by rt tasks based on the last activity.
		 * This value will be the used as an estimation of the next
		 * activity.
		 */
		sched_rt_update_capacity_req(rq);
		return NULL;
	}

	put_prev_task(rq, prev);

	p = _pick_next_task_rt(rq);

	/* The running task is never eligible for pushing */
	dequeue_pushable_task(rt_rq, p);

	queue_push_tasks(rq);

	return p;
}

static void put_prev_task_rt(struct rq *rq, struct task_struct *p)
{
	struct rt_rq *rt_rq = rt_rq_of_se(&p->rt);

	update_curr_rt(rq);

	/*
	 * The previous task needs to be made eligible for pushing
	 * if it is still active
	 */
	if (on_rt_rq(&p->rt) && tsk_nr_cpus_allowed(p) > 1)
		enqueue_pushable_task(rt_rq, p);
}

#ifdef CONFIG_SMP

/* Only try algorithms three times */
#define RT_MAX_TRIES 3

static int pick_rt_task(struct rt_rq *rt_rq, struct task_struct *p, int cpu)
{
	if (!task_running(rq_of_rt_rq(rt_rq), p) &&
	    cpumask_test_cpu(cpu, tsk_cpus_allowed(p)))
		return 1;
	return 0;
}

/*
 * Return the highest pushable rq's task, which is suitable to be executed
 * on the cpu, NULL otherwise
 */
static
struct task_struct *pick_highest_pushable_task(struct rt_rq *rt_rq, int cpu)
{
	struct plist_head *head = &rt_rq->pushable_tasks;
	struct task_struct *p;

	if (!has_pushable_tasks(rt_rq))
		return NULL;

	plist_for_each_entry(p, head, pushable_tasks) {
		if (pick_rt_task(rt_rq, p, cpu))
			return p;
	}

	return NULL;
}

static DEFINE_PER_CPU(cpumask_var_t, local_cpu_mask);

static int find_lowest_rq(struct task_struct *task)
{
	struct sched_domain *sd;
	struct cpumask *lowest_mask = this_cpu_cpumask_var_ptr(local_cpu_mask);
	int this_cpu = smp_processor_id();
	int cpu      = task_cpu(task);

	/* Make sure the mask is initialized first */
	if (unlikely(!lowest_mask))
		return -1;

	if (tsk_nr_cpus_allowed(task) == 1)
		return -1; /* No other targets possible */

	if (!cpupri_find(&task_rq(task)->rd->cpupri, task, lowest_mask))
		return -1; /* No targets found */

	/*
	 * At this point we have built a mask of cpus representing the
	 * lowest priority tasks in the system.  Now we want to elect
	 * the best one based on our affinity and topology.
	 *
	 * We prioritize the last cpu that the task executed on since
	 * it is most likely cache-hot in that location.
	 */
	if (cpumask_test_cpu(cpu, lowest_mask))
		return cpu;

	/*
	 * Otherwise, we consult the sched_domains span maps to figure
	 * out which cpu is logically closest to our hot cache data.
	 */
	if (!cpumask_test_cpu(this_cpu, lowest_mask))
		this_cpu = -1; /* Skip this_cpu opt if not among lowest */

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (sd->flags & SD_WAKE_AFFINE) {
			int best_cpu;

			/*
			 * "this_cpu" is cheaper to preempt than a
			 * remote processor.
			 */
			if (this_cpu != -1 &&
			    cpumask_test_cpu(this_cpu, sched_domain_span(sd))) {
				rcu_read_unlock();
				return this_cpu;
			}

			best_cpu = cpumask_first_and(lowest_mask,
						     sched_domain_span(sd));
			if (best_cpu < nr_cpu_ids) {
				rcu_read_unlock();
				return best_cpu;
			}
		}
	}
	rcu_read_unlock();

	/*
	 * And finally, if there were no matches within the domains
	 * just give the caller *something* to work with from the compatible
	 * locations.
	 */
	if (this_cpu != -1)
		return this_cpu;

	cpu = cpumask_any(lowest_mask);
	if (cpu < nr_cpu_ids)
		return cpu;
	return -1;
}

/* Will lock the rq it finds */
static struct rq *find_lock_lowest_rq(struct task_struct *task, struct rq *rq)
{
	struct rq *lowest_rq = NULL;
	int tries;
	int cpu;

	for (tries = 0; tries < RT_MAX_TRIES; tries++) {
		cpu = find_lowest_rq(task);

		if ((cpu == -1) || (cpu == rq->cpu))
			break;

		lowest_rq = cpu_rq(cpu);

		if (lowest_rq->rt.highest_prio.curr <= task->prio) {
			/*
			 * Target rq has tasks of equal or higher priority,
			 * retrying does not release any lock and is unlikely
			 * to yield a different result.
			 */
			lowest_rq = NULL;
			break;
		}

		/* if the prio of this runqueue changed, try again */
		if (double_lock_balance(rq, lowest_rq)) {
			/*
			 * We had to unlock the run queue. In
			 * the mean time, task could have
			 * migrated already or had its affinity changed.
			 * Also make sure that it wasn't scheduled on its rq.
			 */
			if (unlikely(task_rq(task) != rq ||
				     !cpumask_test_cpu(lowest_rq->cpu,
						       tsk_cpus_allowed(task)) ||
				     task_running(rq, task) ||
				     !rt_task(task) ||
				     !task_on_rq_queued(task))) {

				double_unlock_balance(rq, lowest_rq);
				lowest_rq = NULL;
				break;
			}
		}

		/* If this rq is still suitable use it. */
		if (lowest_rq->rt.highest_prio.curr > task->prio)
			break;

		/* try again */
		double_unlock_balance(rq, lowest_rq);
		lowest_rq = NULL;
	}

	return lowest_rq;
}

static struct task_struct *pick_next_pushable_task(struct rt_rq *rt_rq)
{
	struct rq *rq = rq_of_rt_rq(rt_rq);
	struct task_struct *p;

	if (!has_pushable_tasks(rt_rq))
		return NULL;

	p = plist_first_entry(&rt_rq->pushable_tasks,
			      struct task_struct, pushable_tasks);

	BUG_ON(rq->cpu != task_cpu(p));
	BUG_ON(task_current(rq, p));
	BUG_ON(tsk_nr_cpus_allowed(p) <= 1);

	BUG_ON(!task_on_rq_queued(p));
	BUG_ON(!rt_task(p));

	return p;
}

/*
 * If the current CPU has more than one RT task, see if the non
 * running task can migrate over to a CPU that is running a task
 * of lesser priority.
 */
static int push_rt_task(struct rq *rq)
{
	struct task_struct *next_task;
	struct rq *lowest_rq;
	int ret = 0;

	if (!rq->rt.overloaded)
		return 0;

	next_task = pick_next_pushable_task(&rq->rt);
	if (!next_task)
		return 0;

retry:
	if (unlikely(next_task == rq->curr)) {
		WARN_ON(1);
		return 0;
	}

	/*
	 * It's possible that the next_task slipped in of
	 * higher priority than current. If that's the case
	 * just reschedule current.
	 */
	if (unlikely(next_task->prio < rq->curr->prio)) {
		resched_curr(rq);
		return 0;
	}

	/* We might release rq lock */
	get_task_struct(next_task);

	/* find_lock_lowest_rq locks the rq if found */
	lowest_rq = find_lock_lowest_rq(next_task, rq);
	if (!lowest_rq) {
		struct task_struct *task;
		/*
		 * find_lock_lowest_rq releases rq->lock
		 * so it is possible that next_task has migrated.
		 *
		 * We need to make sure that the task is still on the same
		 * run-queue and is also still the next task eligible for
		 * pushing.
		 */
		task = pick_next_pushable_task(&rq->rt);
		if (task_cpu(next_task) == rq->cpu && task == next_task) {
			/*
			 * The task hasn't migrated, and is still the next
			 * eligible task, but we failed to find a run-queue
			 * to push it to.  Do not retry in this case, since
			 * other cpus will pull from us when ready.
			 */
			goto out;
		}

		if (!task)
			/* No more tasks, just exit */
			goto out;

		/*
		 * Something has shifted, try again.
		 */
		put_task_struct(next_task);
		next_task = task;
		goto retry;
	}

	deactivate_task(rq, next_task, 0);
	set_task_cpu(next_task, lowest_rq->cpu);
	activate_task(lowest_rq, next_task, 0);
	ret = 1;

	resched_curr(lowest_rq);

	double_unlock_balance(rq, lowest_rq);

out:
	put_task_struct(next_task);

	return ret;
}

static void push_rt_tasks(struct rq *rq)
{
	/* push_rt_task will return true if it moved an RT */
	while (push_rt_task(rq))
		;
}

#ifdef HAVE_RT_PUSH_IPI
/*
 * The search for the next cpu always starts at rq->cpu and ends
 * when we reach rq->cpu again. It will never return rq->cpu.
 * This returns the next cpu to check, or nr_cpu_ids if the loop
 * is complete.
 *
 * rq->rt.push_cpu holds the last cpu returned by this function,
 * or if this is the first instance, it must hold rq->cpu.
 */
static int rto_next_cpu(struct rq *rq)
{
	int prev_cpu = rq->rt.push_cpu;
	int cpu;

	cpu = cpumask_next(prev_cpu, rq->rd->rto_mask);

	/*
	 * If the previous cpu is less than the rq's CPU, then it already
	 * passed the end of the mask, and has started from the beginning.
	 * We end if the next CPU is greater or equal to rq's CPU.
	 */
	if (prev_cpu < rq->cpu) {
		if (cpu >= rq->cpu)
			return nr_cpu_ids;

	} else if (cpu >= nr_cpu_ids) {
		/*
		 * We passed the end of the mask, start at the beginning.
		 * If the result is greater or equal to the rq's CPU, then
		 * the loop is finished.
		 */
		cpu = cpumask_first(rq->rd->rto_mask);
		if (cpu >= rq->cpu)
			return nr_cpu_ids;
	}
	rq->rt.push_cpu = cpu;

	/* Return cpu to let the caller know if the loop is finished or not */
	return cpu;
}

static int find_next_push_cpu(struct rq *rq)
{
	struct rq *next_rq;
	int cpu;

	while (1) {
		cpu = rto_next_cpu(rq);
		if (cpu >= nr_cpu_ids)
			break;
		next_rq = cpu_rq(cpu);

		/* Make sure the next rq can push to this rq */
		if (next_rq->rt.highest_prio.next < rq->rt.highest_prio.curr)
			break;
	}

	return cpu;
}

#define RT_PUSH_IPI_EXECUTING		1
#define RT_PUSH_IPI_RESTART		2

static void tell_cpu_to_push(struct rq *rq)
{
	int cpu;

	if (rq->rt.push_flags & RT_PUSH_IPI_EXECUTING) {
		raw_spin_lock(&rq->rt.push_lock);
		/* Make sure it's still executing */
		if (rq->rt.push_flags & RT_PUSH_IPI_EXECUTING) {
			/*
			 * Tell the IPI to restart the loop as things have
			 * changed since it started.
			 */
			rq->rt.push_flags |= RT_PUSH_IPI_RESTART;
			raw_spin_unlock(&rq->rt.push_lock);
			return;
		}
		raw_spin_unlock(&rq->rt.push_lock);
	}

	/* When here, there's no IPI going around */

	rq->rt.push_cpu = rq->cpu;
	cpu = find_next_push_cpu(rq);
	if (cpu >= nr_cpu_ids)
		return;

	rq->rt.push_flags = RT_PUSH_IPI_EXECUTING;

	irq_work_queue_on(&rq->rt.push_work, cpu);
}

/* Called from hardirq context */
static void try_to_push_tasks(void *arg)
{
	struct rt_rq *rt_rq = arg;
	struct rq *rq, *src_rq;
	int this_cpu;
	int cpu;

	this_cpu = rt_rq->push_cpu;

	/* Paranoid check */
	BUG_ON(this_cpu != smp_processor_id());

	rq = cpu_rq(this_cpu);
	src_rq = rq_of_rt_rq(rt_rq);

again:
	if (has_pushable_tasks(&rq->rt)) {
		raw_spin_lock(&rq->lock);
		push_rt_task(rq);
		raw_spin_unlock(&rq->lock);
	}

	/* Pass the IPI to the next rt overloaded queue */
	raw_spin_lock(&rt_rq->push_lock);
	/*
	 * If the source queue changed since the IPI went out,
	 * we need to restart the search from that CPU again.
	 */
	if (rt_rq->push_flags & RT_PUSH_IPI_RESTART) {
		rt_rq->push_flags &= ~RT_PUSH_IPI_RESTART;
		rt_rq->push_cpu = src_rq->cpu;
	}

	cpu = find_next_push_cpu(src_rq);

	if (cpu >= nr_cpu_ids)
		rt_rq->push_flags &= ~RT_PUSH_IPI_EXECUTING;
	raw_spin_unlock(&rt_rq->push_lock);

	if (cpu >= nr_cpu_ids)
		return;

	/*
	 * It is possible that a restart caused this CPU to be
	 * chosen again. Don't bother with an IPI, just see if we
	 * have more to push.
	 */
	if (unlikely(cpu == rq->cpu))
		goto again;

	/* Try the next RT overloaded CPU */
	irq_work_queue_on(&rt_rq->push_work, cpu);
}

static void push_irq_work_func(struct irq_work *work)
{
	struct rt_rq *rt_rq = container_of(work, struct rt_rq, push_work);

	try_to_push_tasks(rt_rq);
}
#endif /* HAVE_RT_PUSH_IPI */

static void pull_rt_task(struct rq *this_rq)
{
	int this_cpu = this_rq->cpu, cpu;
	bool resched = false;
	struct task_struct *p;
	struct rt_rq *src_rt_rq;
	struct rq *src_rq;

	if (likely(!rt_overloaded(this_rq)))
		return;

	/*
	 * Match the barrier from rt_set_overloaded; this guarantees that if we
	 * see overloaded we must also see the rto_mask bit.
	 */
	smp_rmb();

#ifdef HAVE_RT_PUSH_IPI
	if (sched_feat(RT_PUSH_IPI)) {
		tell_cpu_to_push(this_rq);
		return;
	}
#endif

	for_each_cpu(cpu, this_rq->rd->rto_mask) {
		if (this_cpu == cpu)
			continue;

		src_rq = cpu_rq(cpu);
		src_rt_rq = &src_rq->rt;

		/*
		 * Don't bother taking the src_rq->lock if the next highest
		 * task is known to be lower-priority than our current task.
		 * This may look racy, but if this value is about to go
		 * logically higher, the src_rq will push this task away.
		 * And if its going logically lower, we do not care
		 */
		if (src_rt_rq->highest_prio.next >=
		    this_rq->rt.highest_prio.curr)
			continue;

		/*
		 * We can potentially drop this_rq's lock in
		 * double_lock_balance, and another CPU could
		 * alter this_rq
		 */
		double_lock_balance(this_rq, src_rq);

		/*
		 * We can pull only a task, which is pushable
		 * on its rq, and no others.
		 */
		p = pick_highest_pushable_task(src_rt_rq, this_cpu);

		/*
		 * Do we have an RT task that preempts
		 * the to-be-scheduled task?
		 */
		if (p && (p->prio < this_rq->rt.highest_prio.curr)) {
			WARN_ON(p == src_rq->curr);
			WARN_ON(!task_on_rq_queued(p));

			/*
			 * There's a chance that p is higher in priority
			 * than what's currently running on its cpu.
			 * This is just that p is wakeing up and hasn't
			 * had a chance to schedule. We only pull
			 * p if it is lower in priority than the
			 * current task on the run queue
			 */
			if (p->prio < src_rq->curr->prio)
				goto skip;

			resched = true;

			deactivate_task(src_rq, p, 0);
			set_task_cpu(p, this_cpu);
			activate_task(this_rq, p, 0);
			/*
			 * We continue with the search, just in
			 * case there's an even higher prio task
			 * in another runqueue. (low likelihood
			 * but possible)
			 */
		}
skip:
		double_unlock_balance(this_rq, src_rq);
	}

	if (resched)
		resched_curr(this_rq);
}

#ifdef CONFIG_RT_GROUP_SCHED
int group_push_rt_task(struct rt_rq *rt_rq)
{
	struct rq *rq = rq_of_rt_rq(rt_rq);

	if (is_dl_group(rt_rq))
		return 0;

	return push_rt_task(rq);
}

void group_push_rt_tasks(struct rt_rq *rt_rq)
{
	while (group_push_rt_task(rt_rq))
		;
}
#else
void group_push_rt_tasks(struct rt_rq *rt_rq)
{
	push_rt_tasks(rq_of_rt_rq(rt_rq));
}
#endif

/*
 * If we are not running and we are not going to reschedule soon, we should
 * try to push tasks away now
 */
static void task_woken_rt(struct rq *rq, struct task_struct *p)
{
	struct rt_rq *rt_rq = rt_rq_of_se(&p->rt);

	if (!task_running(rq, p) &&
	    !test_tsk_need_resched(rq->curr) &&
	    tsk_nr_cpus_allowed(p) > 1 &&
	    (dl_task(rq->curr) || rt_task(rq->curr)) &&
	    (tsk_nr_cpus_allowed(rq->curr) < 2 ||
	     rq->curr->prio <= p->prio))
		group_push_rt_tasks(rt_rq);
}

/* Assumes rq->lock is held */
static void rq_online_rt(struct rq *rq)
{
	if (rq->rt.overloaded)
		rt_set_overload(rq);

	cpupri_set(&rq->rd->cpupri, rq->cpu, rq->rt.highest_prio.curr);
}

/* Assumes rq->lock is held */
static void rq_offline_rt(struct rq *rq)
{
	if (rq->rt.overloaded)
		rt_clear_overload(rq);

	cpupri_set(&rq->rd->cpupri, rq->cpu, CPUPRI_INVALID);
}

/*
 * When switch from the rt queue, we bring ourselves to a position
 * that we might want to pull RT tasks from other runqueues.
 */
static void switched_from_rt(struct rq *rq, struct task_struct *p)
{
	struct rt_rq *rt_rq = rt_rq_of_se(&p->rt);

	BUG_ON(task_cpu(p) != cpu_of(rq));

	/*
	 * If there are other RT tasks then we will reschedule
	 * and the scheduling of the other RT tasks will handle
	 * the balancing. But if we are the last RT task
	 * we may need to handle the pulling of RT tasks
	 * now.
	 */
	if (!task_on_rq_queued(p) || rt_rq->rt_nr_running)
		return;

#ifndef CONFIG_RT_GROUP_SCHED
	queue_pull_task(rq);
#endif
}

void __init init_sched_rt_class(void)
{
	unsigned int i;

	for_each_possible_cpu(i) {
		zalloc_cpumask_var_node(&per_cpu(local_cpu_mask, i),
					GFP_KERNEL, cpu_to_node(i));
	}
}
#endif /* CONFIG_SMP */

/*
 * When switching a task to RT, we may overload the runqueue
 * with RT tasks. In this case we try to push them off to
 * other runqueues.
 */
static void switched_to_rt(struct rq *rq, struct task_struct *p)
{
	/*
	 * If we are already running, then there's nothing
	 * that needs to be done. But if we are not running
	 * we may need to preempt the current running task.
	 * If that current running task is also an RT task
	 * then see if we can move to another run queue.
	 */
	if (task_on_rq_queued(p) && rq->curr != p) {
#ifdef CONFIG_SMP
#ifndef CONFIG_RT_GROUP_SCHED
		if (tsk_nr_cpus_allowed(p) > 1 && rq->rt.overloaded)
			queue_push_tasks(rq);
#else
		if (rt_rq_of_se(&p->rt)->overloaded) {
		} else {
			if (p->prio < rq->curr->prio)
				resched_curr(rq);
		}
#endif
#else
		if (p->prio < rq->curr->prio)
			resched_curr(rq);
#endif /* CONFIG_SMP */
	}
}

/*
 * Priority of the task has changed. This may cause
 * us to initiate a push or pull.
 */
static void
prio_changed_rt(struct rq *rq, struct task_struct *p, int oldprio)
{
#ifdef CONFIG_SMP
	struct rt_rq *rt_rq = rt_rq_of_se(&p->rt);
#endif

	if (!task_on_rq_queued(p))
		return;

	if (rq->curr == p) {
#ifdef CONFIG_SMP
		/*
		 * If our priority decreases while running, we
		 * may need to pull tasks to this runqueue.
		 */
		if (oldprio < p->prio)
#ifndef CONFIG_RT_GROUP_SCHED
			queue_pull_task(rq);
#endif
		/*
		 * If there's a higher priority task waiting to run
		 * then reschedule.
		 */
		if (p->prio > rt_rq->highest_prio.curr)
			resched_curr(rq);
#else
		/* For UP simply resched on drop of prio */
		if (oldprio < p->prio)
			resched_curr(rq);
#endif /* CONFIG_SMP */
	} else {
		/*
		 * This task is not running, but if it is
		 * greater than the current running task
		 * then reschedule.
		 */
		if (p->prio < rq->curr->prio)
			resched_curr(rq);
	}
}

static void watchdog(struct rq *rq, struct task_struct *p)
{
	unsigned long soft, hard;

	/* max may change after cur was read, this will be fixed next tick */
	soft = task_rlimit(p, RLIMIT_RTTIME);
	hard = task_rlimit_max(p, RLIMIT_RTTIME);

	if (soft != RLIM_INFINITY) {
		unsigned long next;

		if (p->rt.watchdog_stamp != jiffies) {
			p->rt.timeout++;
			p->rt.watchdog_stamp = jiffies;
		}

		next = DIV_ROUND_UP(min(soft, hard), USEC_PER_SEC/HZ);
		if (p->rt.timeout > next)
			p->cputime_expires.sched_exp = p->se.sum_exec_runtime;
	}
}

static void task_tick_rt(struct rq *rq, struct task_struct *p, int queued)
{
	struct sched_rt_entity *rt_se = &p->rt;

	update_curr_rt(rq);
#ifdef CONFIG_RT_GROUP_SCHED
	if (is_dl_group(&rq->rt)) {
		struct sched_dl_entity *dl_se = dl_group_of(&rq->rt);

		if (hrtick_enabled(rq) && queued && dl_se->runtime > 0)
			start_hrtick_dl(rq, dl_se);
	}
#endif

	if (rq->rt.rt_nr_running)
		sched_rt_update_capacity_req(rq);

	watchdog(rq, p);

	/*
	 * RR tasks need a special form of timeslice management.
	 * FIFO tasks have no timeslices.
	 */
	if (p->policy != SCHED_RR)
		return;

	if (--p->rt.time_slice)
		return;

	p->rt.time_slice = sched_rr_timeslice;

	/*
	 * Requeue to the end of queue if we (and all of our ancestors) are not
	 * the only element on the queue
	 */
	if (rt_se->run_list.prev != rt_se->run_list.next) {
		requeue_task_rt(rq, p, 0);
		set_tsk_need_resched(p);
		return;
	}
}

static void set_curr_task_rt(struct rq *rq)
{
	struct task_struct *p = rq->curr;
	struct rt_rq *rt_rq = rt_rq_of_se(&p->rt);

	p->se.exec_start = rq_clock_task(rq);

	/* The running task is never eligible for pushing */
	dequeue_pushable_task(rt_rq, p);
}

static unsigned int get_rr_interval_rt(struct rq *rq, struct task_struct *task)
{
	/*
	 * Time slice is 0 for SCHED_FIFO tasks
	 */
	if (task->policy == SCHED_RR)
		return sched_rr_timeslice;
	else
		return 0;
}

const struct sched_class rt_sched_class = {
	.next			= &fair_sched_class,
	.enqueue_task		= enqueue_task_rt,
	.dequeue_task		= dequeue_task_rt,
	.yield_task		= yield_task_rt,

	.check_preempt_curr	= check_preempt_curr_rt,

	.pick_next_task		= pick_next_task_rt,
	.put_prev_task		= put_prev_task_rt,

#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_rt,

	.set_cpus_allowed       = set_cpus_allowed_common,
	.rq_online              = rq_online_rt,
	.rq_offline             = rq_offline_rt,
	.task_woken		= task_woken_rt,
	.switched_from		= switched_from_rt,
#endif

	.set_curr_task          = set_curr_task_rt,
	.task_tick		= task_tick_rt,

	.get_rr_interval	= get_rr_interval_rt,

	.prio_changed		= prio_changed_rt,
	.switched_to		= switched_to_rt,

	.update_curr		= update_curr_rt,
};

#ifdef CONFIG_SCHED_DEBUG
extern void print_rt_rq(struct seq_file *m, int cpu, struct rt_rq *rt_rq);

void print_rt_stats(struct seq_file *m, int cpu)
{
}
#endif /* CONFIG_SCHED_DEBUG */
