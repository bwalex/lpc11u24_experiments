#include <sem.h>

void
sem_init(struct sem *sem, int initial_count)
{
	sem->sem_count = initial_count;
	TAILQ_INIT(&sem->sem_waitq);
}

int
sem_tryget(struct sem *sem, int count)
{
	int acquired = 0;

	crit_enter();
	if (sem->sem_count >= count) {
		sem->sem_count -= count;
		acquired = 1;
	}
	crit_exit();
	return acquired;
}

int
sem_get(struct sem *sem, int flags, int count, int timo_ticks)
{
	int on_sleepq = 0;

	crit_enter();
again:
	if (sem->sem_count >= count) {
		sem->sem_count -= count;
		crit_exit();
		return 0;
	} else if (flags & SEM_FLAGS_ISR) {
		return -1;
	}

	if (!on_sleepq && timo_ticks > 0) {
		sleepq_insert(curtcb, timo_ticks);
		on_sleepq = 1;
	}

	TAILQ_INSERT_TAIL(&sem->sem_waitq, curtcb, tcb_wlink);
	curtcb = NULL;
	_sched_yield();
	crit_exit();

	crit_enter();
	if (curtcb->tcb_flags & TCB_FLAGS_TIMEDOUT) {
		TAILQ_REMOVE(&sem->sem_waitq, curtcb, tcb_wlink);
		crit_exit();
		return -1;
	}

	goto again;
}

void
sem_put(struct sem *sem, int flags, int count)
{
	struct tcb *tcb;
	int need_preempt = 0;

	crit_enter();
	sem->sem_count += count;
	TAILQ_FOREACH(tcb, &sem->sem_waitq, tcb_wlink) {
		runq_insert(tcb);
		if (!(flags & SEM_FLAGS_ISR) && (tcb->tcb_prio > curtcb->tcb_prio))
			need_preempt = 1;
	}
	/* Empty waitq */
	TAILQ_INIT(&sem->sem_waitq);

	if (need_preempt)
		sched_yield();

	crit_exit();
}
