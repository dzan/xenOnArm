#ifndef __ASM_EVENT_H__
#define __ASM_EVENT_H__

#include <asm/gic.h>
#include <asm/domain.h>

void vcpu_kick(struct vcpu *v);
void vcpu_mark_events_pending(struct vcpu *v);

static inline int _local_events_need_delivery(int check_masked)
{
    struct pending_irq *p = irq_to_pending(current, VGIC_IRQ_EVTCHN_CALLBACK);
    struct cpu_user_regs *regs = guest_cpu_user_regs();

    /* guest IRQs are masked */
    if ( check_masked && (regs->cpsr & PSR_IRQ_MASK) )
        return 0;

    /* XXX: if the first interrupt has already been delivered, we should
     * check whether any higher priority interrupts are in the
     * lr_pending queue or in the LR registers and return 1 only in that
     * case.
     * In practice the guest interrupt handler should run with
     * interrupts disabled so this shouldn't be a problem in the general
     * case.
     */
    if ( gic_events_need_delivery() )
        return 1;

    if ( vcpu_info(current, evtchn_upcall_pending) &&
        !vcpu_info(current, evtchn_upcall_mask) &&
        list_empty(&p->inflight) )
        return 1;

    return 0;
}

static inline int local_events_need_delivery(void)
{
    return _local_events_need_delivery(1);
}

int local_event_delivery_is_enabled(void);

static inline void local_event_delivery_disable(void)
{
    current->vcpu_info->evtchn_upcall_mask = 1;
}

static inline void local_event_delivery_enable(void)
{
    current->vcpu_info->evtchn_upcall_mask = 0;
}

/* No arch specific virq definition now. Default to global. */
static inline int arch_virq_is_global(int virq)
{
    return 1;
}

#endif
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
