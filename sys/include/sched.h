#ifndef _SCHED_H
#define _SCHED_H

#include <stdint.h>

extern uint32_t curtick;
extern int critcount;

#define TCB_FLAGS_TIMEDOUT	0x01

typedef void (*task_fn_t)(void *);

static
inline
void
crit_enter(void)
{
	if (critcount++ == 0)
		__asm__ __volatile__ ("cpsid i");
}

static
inline
void
crit_exit(void)
{
	if (--critcount == 0)
		__asm__ __volatile__ ("cpsie i");
}

void sched_yield(void);
void sched_sleep(const char *wmesg, int ticks);
#if 0
void sched_wait(const volatile void *ident, int flags, const char *wmesg, int timo_ticks);
#endif

int task_create(task_fn_t task_fn, uint32_t priv, int prio, size_t stacksz, const char *task_desc);
int sched_init(void);
void sched_start(void) __attribute__((noreturn));

#endif
