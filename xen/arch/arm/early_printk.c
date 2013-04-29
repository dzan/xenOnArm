/*
 * printk() for use before the final page tables are setup.
 *
 * Copyright (C) 2012 Citrix Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/stdarg.h>
#include <xen/string.h>
#include <asm/early_printk.h>
#include <xen/mm.h>

#if defined EARLY_RAMOOPS_ADDRESS
void __init early_putch(char c)
{
    static uint32_t rammops_offset = 0;
    volatile char *r;
    static uint32_t pfn = EARLY_RAMOOPS_ADDRESS >> PAGE_SHIFT;

    r = (char *)(FIXMAP_ADDR(FIXMAP_CONSOLE) + rammops_offset);
    *r = c;
    rammops_offset += sizeof(char);
    if (rammops_offset >= PAGE_SIZE) {
        clear_fixmap(FIXMAP_CONSOLE);
        if (pfn >= (EARLY_RAMOOPS_ADDRESS + 5 * 0x20000 - 1) >> PAGE_SHIFT)
            pfn = (EARLY_RAMOOPS_ADDRESS >> PAGE_SHIFT) - 1;
        set_fixmap(FIXMAP_CONSOLE, ++pfn, DEV_SHARED);
        rammops_offset = 0;
    }
}
#endif

#ifdef EARLY_UART_ADDRESS

void __init early_putch(char c)
{
    volatile uint32_t *r;

    r = (uint32_t *)FIXMAP_ADDR(FIXMAP_CONSOLE);

#ifdef MACH_VEXPRESS
    /* XXX: assuming a PL011 UART. */
    while(*(r + 0x6) & 0x8)
        ;
    *r = c;
#elif defined MACH_EXYNOS5
    while (!(*(r + (0x10/4)) & (1<<1)))
        ;
    *(r+(0x20/4)) = c;
#endif
}
#endif

#if defined(EARLY_UART_ADDRESS) || defined(EARLY_RAMOOPS_ADDRESS)
static void __init early_puts(const char *s)
{
    while (*s != '\0') {
        if (*s == '\n')
            early_putch('\r');
        early_putch(*s);
        s++;
    }
}

void __init early_vprintk(const char *fmt, va_list args)
{
#ifndef EARLY_RAMOOPS_ADDRESS
    char buf[80];
#else
    char buf[1024];
#endif

    vsnprintf(buf, sizeof(buf), fmt, args);
    early_puts(buf);
}

void __init early_printk(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    early_vprintk(fmt, args);
    va_end(args);
}

void __attribute__((noreturn)) __init
early_panic(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    early_vprintk(fmt, args);
    va_end(args);

    while(1);
}

#endif /* #ifdef EARLY_UART_ADDRESS */
