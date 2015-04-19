#include <wqueue.h>
#include <sem.h>

int
queue_init(struct queue *queue, size_t itemsz, size_t nitems)
{
	size_t qsz = itemsz * nitems;

	if ((queue->q_data = malloc(qsz)) == NULL)
		return -1;

	queue->q_rd_ptr = queue->q_wr_ptr = queue->q_data;
	queue->q_itemsz = itemsz;
	queue->q_nitems = nitems;
	queue->q_data_end = (uint8_t *)queue->q_data + qsz;

	sem_init(&queue->q_produce_sem, nitems);
	sem_init(&queue->q_consume_sem, 0);

	return 0;
}

int
queue_tryget(struct queue *queue, int flags, void *obj)
{
	int ok = 0;

	crit_enter();
	if (sem_tryget(&queue->q_consume_sem, 1)) {
		ok = 1;
		memcpy(obj, queue->q_rd_ptr, queue->q_itemsz);
		/* Increment read pointer and wrap around if needed */
		if ((queue->q_rd_ptr += queue->q_itemsz) >= queue->q_data_end)
			queue->q_rd_ptr = queue->q_data;

		sem_put(&queue->q_produce_sem, flags, 1);
	}
	crit_exit();

	return ok;
}

int
queue_get(struct queue *queue, int flags, void *obj, int timo_ticks)
{
	int err = -1;

	crit_enter();
	if ((err = sem_get(&queue->q_consume_sem, flags, 1, timo_ticks)) == 0) {
		memcpy(obj, queue->q_rd_ptr, queue->q_itemsz);
		/* Increment read pointer and wrap around if needed */
		if ((queue->q_rd_ptr += queue->q_itemsz) >= queue->q_data_end)
			queue->q_rd_ptr = queue->q_data;

		sem_put(&queue->q_produce_sem, flags, 1);
	}
	crit_exit();

	return err;
}

int
queue_tryput(struct queue *queue, int flags, void *obj)
{
	int ok = 0;

	crit_enter();
	if (sem_tryget(&queue->q_produce_sem, 1)) {
		ok = 1;
		memcpy(queue->q_wr_ptr, obj, queue->q_itemsz);
		/* Increment write pointer and wrap around if needed */
		if ((queue->q_wr_ptr += queue->q_itemsz) >= queue->q_data_end)
			queue->q_wr_ptr = queue->q_data;

		sem_put(&queue->q_consume_sem, flags, 1);
	}
	crit_exit();

	return ok;
}


int
queue_put(struct queue *queue, int flags, void *obj, int timo_ticks)
{
	int err = -1;

	crit_enter();
	if ((err = sem_get(&queue->q_produce_sem, flags, 1, timo_ticks)) == 0) {
		memcpy(queue->q_wr_ptr, obj, queue->q_itemsz);
		/* Increment read pointer and wrap around if needed */
		if ((queue->q_wr_ptr += queue->q_itemsz) >= queue->q_data_end)
			queue->q_wr_ptr = queue->q_data;

		sem_put(&queue->q_consume_sem, flags, 1);
	}
	crit_exit();

	return err;
}
