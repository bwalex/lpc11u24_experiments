#ifndef _SCHED_PRIV_H
#define _SCHED_PRIV_H

#include <stdint.h>
#include <queue.h>
#include <sched.h>

/* XXX: Consider using ASSYM flow - but seems overkill */
/*ASSYM(TCB_MDDATA_OFF, offsetof(struct tcb, tcb_mddata));*/

#define TCB_MDDATA_OFF		8
#define TCB_SP_OFF		4

#define MDTCB_MDREGS_OFF	0
#define	TCB_MDREGS_OFF		(TCB_MDDATA_OFF + MDTCB_MDREGS_OFF)

#define SCB_ICSR_PTR		((volatile uint32_t *)0xE000ED04)
#define SCB_ICSR_NMIPENDSET	(1 << 31)
#define SCB_ICSR_PENDSVSET	(1 << 28)
#define SCB_ICSR_PENDSVCLR	(1 << 27)
#define SCB_ICSR_PENDSTSET	(1 << 26)
#define SCB_ICSR_PENDSTCLR	(1 << 25)

#define STACK_ALIGN_BYTES	8
#define STACK_ALIGN_MASK	~0x7UL

/*
 * Exception frame:
 *             <previous>  <- SP points here before interrupt
 * SP + 0x1c   xPSR
 * SP + 0x18   pc
 * SP + 0x14   lr
 * SP + 0x10   r12
 * SP + 0x0c   r3
 * SP + 0x08   r2
 * SP + 0x04   r1
 * SP + 0x00   r0          <- SP points here after interrupt
 */

struct md_ef {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
};


struct md_tcb {
	struct {
		uint32_t r4;
		uint32_t r5;
		uint32_t r6;
		uint32_t r7;
		uint32_t r8;
		uint32_t r9;
		uint32_t r10;
		uint32_t r11;
	} md_regs;
};

struct tcb {
	void			*tcb_sp_base;
	void			*tcb_sp;
	struct md_tcb		tcb_mddata;
	task_fn_t		tcb_fn;
	int			tcb_prio;
	int			tcb_timo;
	uint32_t		tcb_timo_tick;
	int			tcb_flags;

	TAILQ_ENTRY(tcb)	tcb_qlink;
	TAILQ_ENTRY(tcb)	tcb_wlink;
};

extern struct tcb *curtcb;

#if defined (__cplusplus)
extern "C" {
#endif
void pendsv_handler(void);
void systick_handler(void);

void *init_stack(void *sp, void *pc, uint32_t r0);
void init_task(struct tcb *tcb) __attribute__((noreturn));

void runq_insert(struct tcb *tcb);
void sleepq_insert(struct tcb *tcb, int timo_ticks);
void process_sleepq(void);
void _sched_yield(void);
struct tcb *sched_select(void);
void idle_task(void *arg);

#if defined (__cplusplus)
}
#endif

#endif
