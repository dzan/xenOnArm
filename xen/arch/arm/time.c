/*
 * xen/arch/arm/time.c
 *
 * Time and timer support, using the ARM Generic Timer interfaces
 *
 * Tim Deegan <tim@xen.org>
 * Copyright (c) 2011 Citrix Systems.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <xen/config.h>
#include <xen/console.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/lib.h>
#include <xen/mm.h>
#include <xen/softirq.h>
#include <xen/sched.h>
#include <xen/time.h>
#include <xen/sched.h>
#include <xen/event.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/gic.h>
#include <asm/cpufeature.h>

/*
 * Unfortunately the hypervisor timer interrupt appears to be buggy in
 * some versions of the model. Disable this to use the physical timer
 * instead.
 */
#define USE_HYP_TIMER 1
#define IRQ_SPI(x) (x)
#define EXYNOS5_IRQ_MCT_L0		IRQ_SPI(120)
#define EXYNOS5_IRQ_MCT_L1		IRQ_SPI(121)

#define MCT_BASE 0x101c0000
#define EXYNOS4_MCTREG(x)   (MCT_BASE + (x))

#define EXYNOS4_MCT_G_CNT_L		EXYNOS4_MCTREG(0x100)
#define EXYNOS4_MCT_G_CNT_U		EXYNOS4_MCTREG(0x104)
#define EXYNOS4_MCT_G_CNT_WSTAT		EXYNOS4_MCTREG(0x110)

#define EXYNOS4_MCT_G_COMP0_L		EXYNOS4_MCTREG(0x200)
#define EXYNOS4_MCT_G_COMP0_U		EXYNOS4_MCTREG(0x204)
#define EXYNOS4_MCT_G_COMP0_ADD_INCR	EXYNOS4_MCTREG(0x208)

#define EXYNOS4_MCT_G_TCON		EXYNOS4_MCTREG(0x240)

#define EXYNOS4_MCT_G_INT_CSTAT		EXYNOS4_MCTREG(0x244)
#define EXYNOS4_MCT_G_INT_ENB		EXYNOS4_MCTREG(0x248)
#define EXYNOS4_MCT_G_WSTAT		EXYNOS4_MCTREG(0x24C)

#define _EXYNOS4_MCT_L_BASE		EXYNOS4_MCTREG(0x300)
    // cpu nb
#define EXYNOS4_MCT_L_BASE(x)		(_EXYNOS4_MCT_L_BASE + (0x100 * x))
#define EXYNOS4_MCT_L_MASK		(0xffffff00)

#define MCT_L_TCNTB_OFFSET		(0x00)
#define MCT_L_ICNTB_OFFSET		(0x08)
#define MCT_L_TCON_OFFSET		(0x20)
#define MCT_L_INT_CSTAT_OFFSET		(0x30)
#define MCT_L_INT_ENB_OFFSET		(0x34)
#define MCT_L_WSTAT_OFFSET		(0x40)

#define MCT_G_TCON_START		(1 << 8)
#define MCT_G_TCON_COMP0_AUTO_INC	(1 << 1)
#define MCT_G_TCON_COMP0_ENABLE		(1 << 0)

#define MCT_L_TCON_INTERVAL_MODE	(1 << 2)
#define MCT_L_TCON_INT_START		(1 << 1)
#define MCT_L_TCON_TIMER_START		(1 << 0)
uint64_t __read_mostly boot_count;

/* For fine-grained timekeeping, we use the ARM "Generic Timer", a
 * register-mapped time source in the SoC. */
unsigned long __read_mostly cpu_khz;  /* CPU clock frequency in kHz. */

/*static inline*/ s_time_t ticks_to_ns(uint64_t ticks)
{
    return muldiv64(ticks, SECONDS(1), 1000 * cpu_khz);
}

/*static inline*/ uint64_t ns_to_ticks(s_time_t ns)
{
    return muldiv64(ns, 1000 * cpu_khz, SECONDS(1));
}

/* TODO: On a real system the firmware would have set the frequency in
   the CNTFRQ register.  Also we'd need to use devicetree to find
   the RTC.  When we've seen some real systems, we can delete this.
static uint32_t calibrate_timer(void)
{
    uint32_t sec;
    uint64_t start, end;
    paddr_t rtc_base = 0x1C170000ull;
    volatile uint32_t *rtc;

    ASSERT(!local_irq_is_enabled());
    set_fixmap(FIXMAP_MISC, rtc_base >> PAGE_SHIFT, DEV_SHARED);
    rtc = (uint32_t *) FIXMAP_ADDR(FIXMAP_MISC);

    printk("Calibrating timer against RTC...");
    // Turn on the RTC
    rtc[3] = 1;
    // Wait for an edge
    sec = rtc[0] + 1;
    do {} while ( rtc[0] != sec );
    // Now time a few seconds
    start = READ_SYSREG64(CNTPCT_EL0);
    do {} while ( rtc[0] < sec + 32 );
    end = READ_SYSREG64(CNTPCT_EL0);
    printk("done.\n");

    clear_fixmap(FIXMAP_MISC);
    return (end - start) / 32;
}
*/

uint32_t read_register(uint32_t addr)
{
    volatile const uint32_t *reg;
    uint32_t value;
    set_fixmap(FIXMAP_MISC, addr >> PAGE_SHIFT, DEV_SHARED);
    reg = (uint32_t *)(FIXMAP_ADDR(FIXMAP_MISC) + (addr & ~PAGE_MASK));
    value = *reg;
    clear_fixmap(FIXMAP_MISC);
    return value;
}

void write_register(uint32_t addr, uint32_t value)
{
    volatile uint32_t *reg;
    set_fixmap(FIXMAP_MISC, addr >> PAGE_SHIFT, DEV_SHARED);
    reg = (uint32_t *)(FIXMAP_ADDR(FIXMAP_MISC) + (addr & ~PAGE_MASK));
    *reg = value;
    clear_fixmap(FIXMAP_MISC);
}

/* Set up the timer on the boot CPU */
int __init init_xen_time(void)
{
    uint32_t reg;

    /* Check that this CPU supports the Generic Timer interface */
    if ( !cpu_has_gentimer )
        panic("CPU does not support the Generic Timer v1 interface.\n");

    cpu_khz = READ_SYSREG32(CNTFRQ_EL0) / 1000;

    // enable timer on exynos5 arndale board
    // should probably be done by u-boot
    reg = read_register(EXYNOS4_MCT_G_TCON);
    write_register(EXYNOS4_MCT_G_TCON, reg | MCT_G_TCON_START);

    boot_count = READ_SYSREG64(CNTPCT_EL0);
    printk("Using generic timer at %lu KHz\n", cpu_khz);

    return 0;
}

/* Return number of nanoseconds since boot */
s_time_t get_s_time(void)
{
    uint64_t ticks = READ_SYSREG64(CNTPCT_EL0) - boot_count;
    return ticks_to_ns(ticks);
}

/* Set the timer to wake us up at a particular time.
 * Timeout is a Xen system time (nanoseconds since boot); 0 disables the timer.
 * Returns 1 on success; 0 if the timeout is too soon or is in the past. */
int reprogram_timer(s_time_t timeout)
{
    uint64_t deadline;

    if ( timeout == 0 )
    {
#if USE_HYP_TIMER
        WRITE_SYSREG32(0, CNTHP_CTL_EL2);
#else
        WRITE_SYSREG32(0, CNTP_CTL_EL0);
#endif
        return 1;
    }

    deadline = ns_to_ticks(timeout) + boot_count;
#if USE_HYP_TIMER
    WRITE_SYSREG64(deadline, CNTHP_CVAL_EL2);
    WRITE_SYSREG32(CNTx_CTL_ENABLE, CNTHP_CTL_EL2);
#else
    WRITE_SYSREG64(deadline, CNTP_CVAL_EL0);
    WRITE_SYSREG32(CNTx_CTL_ENABLE, CNTP_CTL_EL0);
#endif
    isb();

    /* No need to check for timers in the past; the Generic Timer fires
     * on a signed 63-bit comparison. */
    return 1;
}

/* Handle the firing timer */
static void timer_interrupt(int irq, void *dev_id, struct cpu_user_regs *regs)
{
    if ( irq == 26 && READ_SYSREG32(CNTHP_CTL_EL2) & CNTx_CTL_PENDING )
    {
        /* Signal the generic timer code to do its work */
        raise_softirq(TIMER_SOFTIRQ);
        /* Disable the timer to avoid more interrupts */
        WRITE_SYSREG32(0, CNTHP_CTL_EL2);
    }

    if (irq == 30 && READ_SYSREG32(CNTP_CTL_EL0) & CNTx_CTL_PENDING )
    {
        /* Signal the generic timer code to do its work */
        raise_softirq(TIMER_SOFTIRQ);
        /* Disable the timer to avoid more interrupts */
        WRITE_SYSREG32(0, CNTP_CTL_EL0);
    }
}

static void vtimer_interrupt(int irq, void *dev_id, struct cpu_user_regs *regs)
{
    current->arch.virt_timer.ctl = READ_SYSREG32(CNTV_CTL_EL0);
    WRITE_SYSREG32(current->arch.virt_timer.ctl | CNTx_CTL_MASK, CNTV_CTL_EL0);
    vgic_vcpu_inject_irq(current, irq, 1);
}

/* Set up the timer interrupt on this CPU */
void __cpuinit init_timer_interrupt(void)
{
    /* Sensible defaults */
    WRITE_SYSREG64(0, CNTVOFF_EL2);     /* No VM-specific offset */
    WRITE_SYSREG32(0, CNTKCTL_EL1);     /* No user-mode access */
#if USE_HYP_TIMER
    /* Do not let the VMs program the physical timer, only read the physical counter */
    WRITE_SYSREG32(CNTHCTL_PA, CNTHCTL_EL2);
#else
    /* Cannot let VMs access physical counter if we are using it */
    WRITE_SYSREG32(0, CNTHCTL_EL2);
#endif
    WRITE_SYSREG32(0, CNTP_CTL_EL0);    /* Physical timer disabled */
    WRITE_SYSREG32(0, CNTHP_CTL_EL2);   /* Hypervisor's timer disabled */
    isb();

    /* XXX Need to find this IRQ number from devicetree? */
    request_irq(26, timer_interrupt, 0, "hyptimer", NULL);
    request_irq(27, vtimer_interrupt, 0, "virtimer", NULL);
    request_irq(30, timer_interrupt, 0, "phytimer", NULL);
}

/* Wait a set number of microseconds */
void udelay(unsigned long usecs)
{
    s_time_t deadline = get_s_time() + 1000 * (s_time_t) usecs;
    while ( get_s_time() - deadline < 0 )
        ;
    dsb();
    isb();
}

/* VCPU PV timers. */
void send_timer_event(struct vcpu *v)
{
    send_guest_vcpu_virq(v, VIRQ_TIMER);
}

/* VCPU PV clock. */
void update_vcpu_system_time(struct vcpu *v)
{
    /* XXX update shared_info->wc_* */
}

void domain_set_time_offset(struct domain *d, int32_t time_offset_seconds)
{
    d->time_offset_seconds = time_offset_seconds;
    /* XXX update guest visible wallclock time */
}

struct tm wallclock_time(void)
{
    return (struct tm) { 0 };
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
