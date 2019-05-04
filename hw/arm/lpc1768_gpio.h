/*
 * NXP LPC1768 GPIO support
 *
 * Copyright (c) 2019 Michael Bikovitsky <biko@bikodbg.com>
 *
 * This code is licensed under the GPL v2.
 */

#ifndef HW_LPC1768_GPIO_H
#define HW_LPC1768_GPIO_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"


#define TYPE_LPC1768_GPIO "lpc1768,gpio"
#define LPC1768_GPIO(obj) \
    OBJECT_CHECK(Lpc1768GpioState, (obj), TYPE_LPC1768_GPIO)

#define NUM_LINES (sizeof(uint32_t) * 8 * 5)


typedef struct Lpc1768GpioPort
{
    uint32_t fiodir;
    uint32_t fiomask;
    uint32_t fiopin;
} Lpc1768GpioPort;

typedef struct Lpc1768GpioState
{
    // Private
    SysBusDevice parent_obj;

    // Public
    Lpc1768GpioPort ports[5];
    MemoryRegion port_config;
    qemu_irq out_lines[NUM_LINES];
} Lpc1768GpioState;


#endif
