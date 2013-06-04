#ifndef _LINUX_FUTEX_H
#define _LINUX_FUTEX_H

#include <uapi/linux/futex.h>

struct inode;
struct mm_struct;
struct task_struct;
union ktime;

long do_futex(u32 __user *uaddr, int op, u32 val, union ktime *timeout,
	      u32 __user *uaddr2, u32 val2, u32 val3);

extern int
handle_futex_death(u32 __user *uaddr, struct task_struct *curr, int pi);

#ifdef CONFIG_FUTEX
extern void exit_robust_list(struct task_struct *curr);
extern void exit_pi_state_list(struct task_struct *curr);
extern int futex_cmpxchg_enabled;
#else
static inline void exit_robust_list(struct task_struct *curr)
{
}
static inline void exit_pi_state_list(struct task_struct *curr)
{
}
#endif
#endif
