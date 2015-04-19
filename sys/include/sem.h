#ifndef _SEM_H
#define _SEM_H

#include <stdint.h>
#include <sched_priv.h>

#define SEM_FLAGS_ISR		0x01

struct sem {
	int			sem_count;
	TAILQ_HEAD(, tcb)	sem_waitq;
};

void sem_init(struct sem *sem, int initial_count);
int  sem_tryget(struct sem *sem, int count);
int  sem_get(struct sem *sem, int flags, int count, int timo_ticks);
void sem_put(struct sem *sem, int flags, int count);

#endif
