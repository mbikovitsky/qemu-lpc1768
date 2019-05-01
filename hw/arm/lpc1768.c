/*
 * NXP LPC1768 Board support
 *
 * Copyright (c) 2011,2012 Masashi YOKOTA <yktmss@gmail.com>
 * Copyright (c) 2019 Michael Bikovitsky <biko@bikodbg.com>
 *
 * This code is licensed under the GPL v2.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/arm/arm.h"
#include "hw/arm/armv7m.h"

#define NUM_IRQ_LINES 64

static
void do_sys_reset(void *opaque, int n, int level)
{
    if (level) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void lpc1768_common_init(const char *kernel_filename, const char *cpu_model)
{
    DeviceState *nvic;
    MemoryRegion *ex_sram;
    MemoryRegion *pin_connect_block;
    int sram_size;
    int flash_size;

    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *system_memory = get_system_memory();

    flash_size = 512 * 1024;
    sram_size  = 32 * 1024;

    ///////////////////////////////////////////////////
    /* Flash programming is done via the SCU, so pretend it is ROM.  */
    memory_region_init_ram(flash, NULL, "lpc1768.flash", flash_size,
                           &error_fatal);
    memory_region_set_readonly(flash, true);
    memory_region_add_subregion(system_memory, 0, flash);

    memory_region_init_ram(sram, NULL, "lpc1768.sram", sram_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x10000000, sram);

    nvic = qdev_create(NULL, TYPE_ARMV7M);
    qdev_prop_set_uint32(nvic, "num-irq", NUM_IRQ_LINES);
    qdev_prop_set_string(nvic, "cpu-type", cpu_model);
    object_property_set_link(OBJECT(nvic), OBJECT(system_memory),
                                     "memory", &error_abort);
    /* This will exit with an error if the user passed us a bad cpu_type */
    qdev_init_nofail(nvic);

    qdev_connect_gpio_out_named(nvic, "SYSRESETREQ", 0,
                                qemu_allocate_irq(&do_sys_reset, NULL, 0));
    ///////////////////////////////////////////////////

    sram_size = 16 * 1024 * 2; /* 2 blocks of 16 KiB */
    ex_sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(ex_sram, NULL, "armv7m.AHB-SRAM", sram_size, &error_fatal);
    memory_region_add_subregion(system_memory, 0x2007c000, ex_sram);

    // HACK: This should probably be memory-mapped, so that we can do
    // something intelligent with the pin values.
    pin_connect_block = g_new(MemoryRegion, 1);
    memory_region_init_ram(pin_connect_block, NULL, "lpc1768.pin-connect-block", 0x4000, &error_fatal);
    memory_region_add_subregion(system_memory, 0x4002C000, pin_connect_block);

    DeviceState *dev = qdev_create(NULL, "lpc1768,uart");
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    qdev_init_nofail(dev);
    sysbus_mmio_map(s, 0, 0x4000C000);
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(nvic, 5)); // 21 - (16) = 5

    sysbus_create_simple("lpc1768,sysc", 0x400FC000, NULL);

    armv7m_load_kernel(ARM_CPU(first_cpu), kernel_filename, flash_size);
}

/* FIXME: Figure out how to generate these from lpc1768_boards.  */
static void lpc1768_generic_init(MachineState *machine)
{
    lpc1768_common_init(machine->kernel_filename, machine->cpu_type);
}

static void lpc1768_machine_init(MachineClass *mc)
{
    mc->desc = "LPC1768 Generic board";
    mc->init = lpc1768_generic_init;
    // mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
}

DEFINE_MACHINE("lpc1768_generic", lpc1768_machine_init)
