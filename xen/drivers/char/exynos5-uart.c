/*
 * xen/drivers/char/exynos5-uart.c
 *
 * Driver for ARM PrimeCell PL011 UART.
 *
 * Anthony PERARD <anthony.perard@citrix.com>
 * Copyright (c) 2012 Citrix Systems.
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
#include <xen/serial.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <asm/early_printk.h>

// fifo does not work
#define EXYNOS_UART_USE_FIFO

static struct exynos5_uart {
    unsigned int baud, clock_hz, data_bits, parity, stop_bits, irq;
    volatile uint32_t *regs;
    struct irqaction irqaction;
} exynos5_com[4] = {{0}};

/* register addresses */
#define ULCON     (0x00/4)
#define UCON      (0x04/4)
#define UFCON     (0x08/4)
#define UMCON     (0x0c/4)
#define UTRSTAT   (0x10/4)
#define UERSTAT   (0x14/4)
#define UFSTAT    (0x18/4)
#define UMSTAT    (0x1c/4)
#define UTXH      (0x20/4)
#define URXH      (0x24/4)
#define UBRDIV    (0x28/4)
#define UFRACVAL  (0x2c/4)
#define UINTP     (0x30/4)
#define UINTS     (0x34/4)
#define UINTM     (0x38/4)

/* ULCON */
#define RXIRQ (0x1<<0)
#define RXDMA (0x2<<0)
#define TXIRQ (0x1<<2)
#define TXDMA (0x2<<2)

/* UFCON */
#define FIFO_TX_RESET (1<<2)
#define FIFO_RX_RESET (1<<1)
#define FIFO_EN   (1<<0)

/* UMCON */
#define INT_EN (1<<3)

/* UTRSTAT */
#define TXE     (1<<2)
#define TXFE    (1<<1)
#define RXDR    (1<<0)

/* Interrupt bits (UINTP, UINTS, UINTM) */
#define MODEM   (1<<3)
#define TXD     (1<<2)
#define ERROR   (1<<1)
#define RXD     (1<<0)
#define ALLI    (MODEM|TXD|ERROR|RXD)

/* These parity settings can be ORed directly into the ULCON. */
#define PARITY_NONE  (0)
#define PARITY_ODD   (0x4)
#define PARITY_EVEN  (0x5)
#define FORCED_CHECKED_AS_ONE (0x6)
#define FORCED_CHECKED_AS_ZERO (0x7)

static void exynos5_uart_interrupt(int irq, void *data, struct cpu_user_regs *regs)
{
    struct serial_port *port = data;
    struct exynos5_uart *uart = port->uart;
    unsigned int status = uart->regs[UINTP];

    if ( status )
    {
        do
        {
            // clear all pending interrept
            // but should take care of ERROR and MODEM

            if (status & ERROR) {
                int error_bit = uart->regs[UERSTAT] & 0xf;
                if (error_bit & (1<<0))
                    printk("uart: overrun error\n");
                if (error_bit & (1<<1))
                    printk("uart: parity error\n");
                if (error_bit & (1<<2))
                    printk("uart: frame error\n");
                if (error_bit & (1<<3))
                    printk("uart: break detected\n");
                uart->regs[UINTP] = ERROR;
            }


            if ( status & (RXD|ERROR) ) {
                /* uart->regs[UINTM] |= RXD|ERROR; */
                serial_rx_interrupt(port, regs);
                /* uart->regs[UINTM] &= ~(RXD|ERROR); */
                uart->regs[UINTP] = RXD|ERROR;
            }

            if ( status & (TXD|MODEM) ) {
                /* uart->regs[UINTM] |= TXD|MODEM; */
                serial_tx_interrupt(port, regs);
                /* uart->regs[UINTM] &= ~(TXD|MODEM); */
                uart->regs[UINTP] = TXD|MODEM;
            }

            status = uart->regs[UINTP];
        } while (status != 0);
    }
}

static void __init exynos5_uart_init_preirq(struct serial_port *port)
{
    struct exynos5_uart *uart = port->uart;
    unsigned int divisor;

    /* reset, TX/RX disables */
    uart->regs[UCON] = 0x0;

    /* No Interrupt, auto flow control */
    uart->regs[UMCON] = 0x0;

    /* Line control and baud-rate generator. */
    if ( uart->baud != BAUD_AUTO )
    {
        /* Baud rate specified: program it into the divisor latch. */
        // div_val = ubrdiv + ufracval/16
        // or
        // div_val = (clock_uart/(baud*16))-1
        divisor = ((uart->clock_hz) / (uart->baud)) - 1;
        // FIXME will use a hacked divisor, assuming the src clock and bauds
        uart->regs[UFRACVAL] = 53;
        uart->regs[UBRDIV] = 4;
        /* uart->regs[UFRACVAL] = divisor & 0xf; */
        /* uart->regs[UBRDIV] = divisor >> 4; */
    }
    else
    {
        // TODO, should be updated
        /* Baud rate already set: read it out from the divisor latch. */
        //divisor = (uart->regs[IBRD] << 6) | uart->regs[FBRD];
        //uart->baud = (uart->clock_hz << 2) / divisor;
    }
    uart->regs[ULCON] = ( (uart->data_bits - 5) << 0
                          | ((uart->stop_bits - 1) << 2)
                          | uart->parity << 3 );

    /* Mask and clear the interrupts */
    uart->regs[UINTM] = ALLI;
    uart->regs[UINTP] = ALLI;

    /* enable FIFO */
    uart->regs[UFCON] = FIFO_TX_RESET | FIFO_RX_RESET;
    while (uart->regs[UFCON] & (FIFO_TX_RESET | FIFO_RX_RESET))
           ;
    // reset FIFO_TX_RESET | FIFO_RX_RESET | 
#ifdef EXYNOS_UART_USE_FIFO
    uart->regs[UFCON] = (0x6<<8)|FIFO_EN;
#else
    uart->regs[UFCON] = 0;
#endif


    /* Enable the UART for RX and TX */
    // level tx/rx inturrupt,only rx
    // enable rx timeout interrupt
    uart->regs[UCON] = (0<<9)|(0<<8)|RXIRQ|TXIRQ|(1<<7);
}

static void __init exynos5_uart_init_postirq(struct serial_port *port)
{
    struct exynos5_uart *uart = port->uart;
    int rc;

    if ( uart->irq > 0 )
    {
        uart->irqaction.handler = exynos5_uart_interrupt;
        uart->irqaction.name    = "exynos5_uart";
        uart->irqaction.dev_id  = port;
        if ( (rc = setup_irq(uart->irq, &uart->irqaction)) != 0 )
            printk("ERROR: Failed to allocate exynos5_uart IRQ %d\n", uart->irq);

        /* Unmask interrupts */
        uart->regs[UINTM] = 0; //MODEM|TXD|ERROR; // only have rx interrupt

        /* Clear pending error interrupts */
        uart->regs[UINTP] = ALLI;

        /* Enable interrupts */
        uart->regs[UMCON] |= INT_EN;
    }
}

static void exynos5_uart_suspend(struct serial_port *port)
{
    BUG(); // XXX
}

static void exynos5_uart_resume(struct serial_port *port)
{
    BUG(); // XXX
}

static unsigned int exynos5_uart_tx_ready(struct serial_port *port)
{
    struct exynos5_uart *uart = port->uart;

#ifdef EXYNOS_UART_USE_FIFO
    // Tx FIFO full
    if (uart->regs[UFSTAT] & (1<<24))
        return 0;
    else
    {
        int x = 16 - ((uart->regs[UFSTAT] >> 16) & 0xff);
        // Tx FIFO count
        if (x > 0)
            return x;
        else if (x == 0)
            return 0;
        else {
            panic("unwanted value: %d\n", x);
            return 0;
        }
    }
#else
    return uart->regs[UTRSTAT] & TXFE ? 1 : 0;
#endif
}

static void exynos5_uart_putc(struct serial_port *port, char c)
{
    struct exynos5_uart *uart = port->uart;
    uart->regs[UTXH] = (uint32_t) (unsigned char) c;
}

static int exynos5_uart_getc(struct serial_port *port, char *pc)
{
    struct exynos5_uart *uart = port->uart;

#ifdef EXYNOS_UART_USE_FIFO
    // check if rx fifo is full or if there is something in it
    if ( (uart->regs[UFSTAT] & (1<<8)) || (uart->regs[UFSTAT] & 0xff))
    {
        *pc = uart->regs[URXH] & 0xff;
        return 1;
    }
    else
        return 0;
#else
    if ( !(uart->regs[UTRSTAT] & RXDR) )
        return 0;
    *pc = uart->regs[URXH] & 0xff;
    return 1;
#endif
}

static int __init exynos5_uart_irq(struct serial_port *port)
{
    struct exynos5_uart *uart = port->uart;
    if ( uart->irq > 0 )
        return uart->irq;
    else
        return -1;
}

static struct uart_driver __read_mostly exynos5_uart_driver = {
    .init_preirq  = exynos5_uart_init_preirq,
    .init_postirq = exynos5_uart_init_postirq,
    .endboot      = NULL,
    .suspend      = exynos5_uart_suspend,
    .resume       = exynos5_uart_resume,
    .tx_ready     = exynos5_uart_tx_ready,
    .putc         = exynos5_uart_putc,
    .getc         = exynos5_uart_getc,
    .irq          = exynos5_uart_irq
};

void __init exynos5_uart_init(int index, unsigned long register_base_address)
{
    // in device tree: compatible = "samsung,exynos4210-uart"
    // node: serial@
    struct exynos5_uart *uart;

    if ( (index < 0) || (index > 4) )
        return;

    uart = &exynos5_com[index];

    /* uart->clock_hz  = 0x16e3600; */
    uart->baud      = BAUD_AUTO;//115200;
    uart->data_bits = 8;
    uart->parity    = PARITY_NONE;
    uart->stop_bits = 1;
    uart->irq       = 32+51 + index; /* TODO Need to find this from devicetree */
    uart->regs      = (uint32_t *) register_base_address;

    /* Register with generic serial driver. */
    serial_register_uart(uart - exynos5_com, &exynos5_uart_driver, uart);
}

/*
 * Local variables:
 * mode: C
 * c-set-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
