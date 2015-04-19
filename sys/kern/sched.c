#include <stdint.h>
#include <sched_priv.h>
#include <queue.h>


static TAILQ_HEAD(, tcb) runq =
	TAILQ_HEAD_INITIALIZER(runq);

static TAILQ_HEAD(, tcb) sleepq =
	TAILQ_HEAD_INITIALIZER(sleepq);

uint32_t curtick;
struct tcb *curtcb;
int critcount;

void
runq_insert(struct tcb *tcb)
{
	struct tcb *elem;

	crit_enter();
	TAILQ_FOREACH(elem, &runq, tcb_qlink) {
		if (tcb->tcb_prio > elem->tcb_prio) {
			TAILQ_INSERT_BEFORE(elem, tcb, tcb_qlink);
			break;
		}
	}

	if (elem == NULL) {
		/* If elem is NULL, we must have reached the end of the queue
		 * before finding a suitable place.
		 * If so, just insert it at the back.
		 */
		TAILQ_INSERT_TAIL(&runq, tcb, tcb_qlink);
	}
	crit_exit();
}

void
sleepq_insert(struct tcb *tcb, int timo_ticks)
{
	struct tcb *elem;

	crit_enter();

	tcb->tcb_timo = timo_ticks;
	tcb->tcb_timo_tick = curtick + timo_ticks;
	tcb->tcb_flags &= ~TCB_FLAGS_TIMEDOUT;

	TAILQ_FOREACH(elem, &sleepq, tcb_qlink) {
		if (tcb->tcb_timo < elem->tcb_timo) {
			TAILQ_INSERT_BEFORE(elem, tcb, tcb_qlink);
			break;
		}
	}

	if (elem == NULL) {
		/* If elem is NULL, we must have reached the end of the queue
		 * before finding a suitable place.
		 * If so, just insert it at the back.
		 */
		TAILQ_INSERT_TAIL(&sleepq, tcb, tcb_qlink);
	}
	crit_exit();
}

void
process_sleepq(void)
{
	struct tcb *elem;

	crit_enter();

	while (((elem = TAILQ_FIRST(&sleepq)) != NULL) &&
	       (elem->tcb_timo_tick == curtick)) {
		TAILQ_REMOVE(&sleepq, elem, tcb_qlink);
		elem->tcb_flags |= TCB_FLAGS_TIMEDOUT;
		runq_insert(elem);
	}

	crit_exit();
}

void
_sched_yield(void)
{
	*SCB_ICSR_PTR = SCB_ICSR_PENDSVSET;
}

void
sched_yield(void)
{
	/*
	 * A yield allows tasks at the same or higher priority to run, but
	 * if only lower priority tasks are available, the task will be
	 * rescheduled immediately.
	 *
	 * To yield to lower priority tasks as well, sched_sleep() must be
	 * used.
	 */
	crit_enter();
	runq_insert(curtcb);
	curtcb = NULL;
	_sched_yield();
	crit_exit();
}

void
sched_sleep(const char *wmesg, int ticks)
{
	crit_enter();
	sleepq_insert(curtcb, ticks);
	curtcb = NULL;
	_sched_yield();
	crit_exit();
}

struct tcb *
sched_select(void)
{
	crit_enter();
	curtcb = TAILQ_FIRST(&runq);
	TAILQ_REMOVE(&runq, curtcb, tcb_qlink);
	crit_exit();
	return curtcb;
}


void
idle_task(void *arg)
{
	for (;;) {
		__asm__ __volatile__ ("wfi");
	}
}

int
task_create(task_fn_t task_fn, uint32_t priv, int prio, size_t stacksz, const char *task_desc)
{
	int err;
	struct tcb *tcb;
	uint8_t *stack_base, *sp;

	if ((tcb = malloc(sizeof(*tcb))) == NULL)
		return -1;

	memset(tcb, 0, sizeof(*tcb));
	tcb->tcb_prio = prio;
	tcb->tcb_fn = task_fn;

	if ((stack_base = malloc(stacksz + 16)) == NULL) {
		free(tcb);
		return -1;
	}

	stacksz += sizeof(struct md_ef);

	/* Align SP to 16-byte boundary */
	sp = (uint8_t *)((uint32_t)(stack_base + stacksz + 15) & ~0xfUL);
	tcb->tcb_sp_base = stack_base;

	/* Set up initial exception frame */
	tcb->tcb_sp = init_stack(sp, task_fn, priv);

	runq_insert(tcb);

	return 0;
}

int
sched_init(void)
{
	curtick = 0;
	curtcb = NULL;
	critcount = 0;

#if 0
	/* XXX: Unnecessary with TAILQ_HEAD_INITIALIZER */
	TAILQ_INIT(&runq);
	TAILQ_INIT(&sleepq);
#endif

	return task_create(idle_task, NULL, -100, 0, "idle");
}

void
sched_start(void)
{
	struct tcb *tcb;

	tcb = sched_select();
	init_task(tcb);
	/* NOTREACHED */
}

void
systick_handler(void)
{
	crit_enter();
	++curtick;
	process_sleepq();
	runq_insert(curtcb);
	_sched_yield();
	crit_exit();
}


