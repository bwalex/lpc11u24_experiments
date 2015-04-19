#define DEFAULT_XPSR		0x01000000 /* only T=1 */
#define DEFAULT_LR		0x00000000 /* XXX: find a better value */
#define CONTROL_PSP		0x2

#define MD_EF_SIZE		32
#define TCB_MDDATA_OFF		8
#define TCB_SP_OFF		4

#define MDTCB_MDREGS_OFF	0
#define	TCB_MDREGS_OFF		(TCB_MDDATA_OFF + MDTCB_MDREGS_OFF)
#define TCB_MDREGS_LO_SZ	(4*4)
#define TCB_MDREGS_HI_SZ	(4*4)
#define TCB_MDREGS_SZ		(TCB_MDREGS_LO_SZ + TCB_MDREGS_HI_SZ)
#define	TCB_MDREGS_HI_OFF	(TCB_MDDATA_OFF + MDTCB_MDREGS_OFF + TCB_MDREGS_LO_SZ)

/* void *init_stack(void *sp, void *pc, uint32_t r0); */
/*                        r0        r1           r2   */
init_stack:
	/* Adjust stack pointer by exception frame size */
	sub r0, #MD_EF_SIZE

	/* Write xPSR into frame */
	ldr r3, =DEFAULT_XPSR
	str r3, [r0,#0x1c]

	/* Write pc into frame */
	str r1, [r0,#0x18]

	/* Write lr into frame */
	ldr r3, =DEFAULT_LR
	str r3, [r0,#0x14]

	/* Write r0 into frame */
	str r2, [r0]

	bx lr


/* void init_task(struct tcb *tcb) __attribute__((noreturn)); */
/*                            r0                              */
init_task:
	/* Disable interrupts so all of this can happen atomically */
	cpsid i

	/* Set up PSP as stack pointer, and switch to it */
	ldr r1, [r0,#TCB_SP_OFF]
	msr PSP, r1
	mov r1, #CONTROL_PSP
	msr CONTROL, r1

	/* Pop from exception frame, and pass control to task */
	pop {r0-r5}
	mov r12, r4
	mov lr,  r5
	cpsie i
	pop {pc}


/* void pendsv_handler(void); */
pendsv_handler:
	/* Find TCB (into t2) */
	ldr r2, =curtcb
	ldr r2, [r2]

	/* Save stack pointer */
	mrs r0, PSP
	str r0, [r2,#TCB_SP_OFF]

	/* Save low registers */
	add r2, r2,#TCB_MDREGS_OFF
	stmia r2!, {r4,r5,r6,r7}

	/* Save high registers */
	mov r4, r8
	mov r5, r9
	mov r6, r10
	mov r7, r11
	stmia r2!, {r4,r5,r6,r7}

	/* Ask scheduler for next task's TCB, returned in r0 */
	push {lr}
	bl sched_select
	pop {r2}

	/* Restore stack pointer */
	ldr r1, [r0,#TCB_SP_OFF]
	msr PSP, r1

	/* Restore high registers */
	add r0, r0,#TCB_MDREGS_HI_OFF
	ldmia r0!, {r4,r5,r6,r7}
	mov r8, r4
	mov r9, r5
	mov r10, r6
	mov r11, r7

	/* Restore low registers */
	sub r0, r0,#TCB_MDREGS_SZ
	ldmia r0!, {r4,r5,r6,r7}

	/* "Return" to what was our lr */
	bx r2
