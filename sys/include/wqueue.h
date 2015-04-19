#ifndef _WQUEUE_H
#define _WQUEUE_H

#include <stdint.h>
#include <sem.h>

struct queue {
	struct sem		q_produce_sem;
	struct sem		q_consume_sem;

	size_t			q_itemsz;
	size_t			q_nitems;

	uint8_t			*q_rd_ptr;
	uint8_t			*q_wr_ptr;

	void			*q_data;
	uint8_t			*q_data_end;
};

int queue_init(struct queue *queue, size_t itemsz, size_t nitems);
int queue_tryget(struct queue *queue, int flags, void *obj);
int queue_get(struct queue *queue, int flags, void *obj, int timo_ticks);
int queue_tryput(struct queue *queue, int flags, void *obj);
int queue_put(struct queue *queue, int flags, void *obj, int timo_ticks);

#endif
