/*
 * NXP LPC1768 GPIO support
 *
 * Copyright (c) 2019 Michael Bikovitsky <biko@bikodbg.com>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"

#include "hw/arm/lpc1768_gpio.h"


#define PORT_SIZE (PORT_FIELDS_COUNT * sizeof(uint32_t))


typedef enum _PORT_FIELD
{
    FIODIR = 0,

    RESERVED1,
    RESERVED2,
    RESERVED3,

    FIOMASK,
    FIOPIN,
    FIOSET,
    FIOCLR,

    // Must be last:
    PORT_FIELDS_COUNT
} PORT_FIELD;


static uint32_t byte_mask(unsigned size)
{
    return (uint32_t)((((uint64_t)1) << (size * 8)) - 1);
}

static uint32_t get_field(uint32_t value, size_t offset, unsigned size)
{
    return (value >> (offset * 8)) & byte_mask(size);
}

static uint32_t set_field(uint32_t old_value, uint32_t new_value, size_t offset, unsigned size)
{
    uint32_t mask = byte_mask(size);
    uint32_t bit_offset = offset * 8;
    uint32_t template = old_value & (~(mask << bit_offset));

    return ((new_value & mask) << (bit_offset)) | template;
}

static Lpc1768GpioPort * decode_address(Lpc1768GpioState * state,
                                        hwaddr address,
                                        unsigned size,
                                        PORT_FIELD * port_field,
                                        size_t * byte_offset,
                                        size_t * port_number)
{
    Lpc1768GpioPort * port = NULL;
    size_t port_index = 0;
    size_t port_offset = 0;
    PORT_FIELD field_type = RESERVED1;
    size_t field_offset = 0;

    if (address >= PORT_SIZE * ARRAY_SIZE(state->ports))
    {
        goto cleanup;
    }

    port_index = address / PORT_SIZE;
    port_offset = address % PORT_SIZE;

    field_type = port_offset / sizeof(uint32_t);
    field_offset = port_offset % sizeof(uint32_t);

    switch (field_type)
    {
    case RESERVED1:
    case RESERVED2:
    case RESERVED3:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "LPC1768_GPIO: Reserved fields are reserved\n");
        goto cleanup;

    case FIODIR:
    case FIOMASK:
    case FIOPIN:
    case FIOSET:
    case FIOCLR:
        break;

    default:
        __builtin_unreachable();
    }

    if (field_offset + size > sizeof(uint32_t))
    {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "LPC1768_GPIO: Can't access past field boundary\n");
        goto cleanup;
    }

    // Return results to caller:
    port = &state->ports[port_index];
    *port_field = field_type;
    *byte_offset = field_offset;
    if (NULL != port_number)
    {
        *port_number = port_index;
    }

cleanup:
    return port;
}

static uint64_t lpc1768_gpio_port_read(void * opaque, hwaddr address, unsigned size)
{
    uint64_t result = 0;
    Lpc1768GpioState * state = opaque;
    Lpc1768GpioPort * port = NULL;
    PORT_FIELD port_field = RESERVED1;
    size_t byte_offset = 0;

    port = decode_address(state, address, size, &port_field, &byte_offset, NULL);
    if (NULL == port)
    {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "LPC1768_GPIO: Bad read offset 0x%" HWADDR_PRIx "\n",
                      address);
        goto cleanup;
    }

    switch (port_field)
    {
    case FIODIR:
        result = get_field(port->fiodir, byte_offset, size);
        break;

    case FIOMASK:
        result = get_field(port->fiomask, byte_offset, size);
        break;

    case FIOPIN:
    {
        uint32_t masked_field = (port->fiopin) & (~(port->fiomask));
        result = get_field(masked_field, byte_offset, size);
        break;
    }

    case FIOSET:
        result = get_field(port->fiopin, byte_offset, size);
        break;

    case FIOCLR:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "LPC1768_GPIO: Can't read from FIOCLR\n");
        goto cleanup;

    default:
        __builtin_unreachable();
    }

cleanup:
    return result;
}

static void update_pins(Lpc1768GpioState * state, size_t port_number, uint32_t previous_pins)
{
    Lpc1768GpioPort * port = NULL;
    uint32_t diff = 0;
    uint32_t pin_index = 0;
    qemu_irq interrupt = NULL;

    port = &state->ports[port_number];

    diff = previous_pins ^ port->fiopin;
    while (0 != diff)
    {
        pin_index = __builtin_ctz(diff);
        interrupt = state->out_lines[port_number * sizeof(uint32_t) * 8 + pin_index];
        if (NULL != interrupt)
        {
            if (port->fiopin & (1 << pin_index))
            {
                qemu_irq_raise(interrupt);
            }
            else
            {
                qemu_irq_lower(interrupt);
            }
        }

        diff &= ~(1 << pin_index);
    }
}

static void lpc1768_gpio_port_write(void * opaque, hwaddr address, uint64_t value, unsigned size)
{
    Lpc1768GpioState * state = opaque;
    Lpc1768GpioPort * port = NULL;
    PORT_FIELD port_field = RESERVED1;
    size_t byte_offset = 0;
    size_t port_number = 0;
    uint32_t new_value = (uint32_t)value;

    port = decode_address(state, address, size, &port_field, &byte_offset, &port_number);
    if (NULL == port)
    {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "LPC1768_GPIO: Bad write offset 0x%" HWADDR_PRIx "\n",
                      address);
        goto cleanup;
    }

    switch (port_field)
    {
    case FIODIR:
        port->fiodir = set_field(port->fiodir, new_value, byte_offset, size);
        break;

    case FIOMASK:
        port->fiomask = set_field(port->fiomask, new_value, byte_offset, size);
        break;

    case FIOSET:
    case FIOCLR:
    {
        // Behave as if there's no pin mask, then fall through to
        // the FIOPIN case which will take care of it.
        uint32_t pins = get_field(port->fiopin, byte_offset, size);
        if (FIOSET == port_field)
        {
            // Writing 1s produces 1s. Writing 0s has no effect.
            new_value = pins | new_value;
        }
        else if (FIOCLR == port_field)
        {
            // Writing 1s produces 0s. Writing 0s has no effect.
            // https://stackoverflow.com/a/7526411
            new_value = pins & (~new_value);
        }
    }
    /* FALLTHRU */
    case FIOPIN:
    {
        uint32_t mask = 0;
        uint32_t masked_field = 0;
        uint32_t masked_new_field = 0;
        uint32_t previous_pins = 0;

        // 1s are pins that are not masked and marked as output, therefore
        // can be modified. 0s are pins that are either masked or
        // marked as input, and therefore should not be modified.
        mask = (~port->fiomask) & port->fiodir;

        masked_field = (port->fiopin) & (~mask);
        masked_new_field = set_field(port->fiopin, new_value, byte_offset, size) & (mask);

        // masked_field has the original field value, with all the pins
        // that *should be* modified zeroed-out.
        // masked_new_field has the original field, with the relevant bits
        // replaced by new_value, and with all the pins that *should not*
        // be modified zeroed-out.
        // The net result is that ORing the two values gives us what we need:
        // - Pins that should not be modified have their original values
        //   in masked_field and a 0 in masked_new_field, so they retain
        //   their values in the resulting value.
        // - Pins that should be modified have 0 in masked_field and
        //   the new value in maked_new_field, so the result has the new value.

        previous_pins = port->fiopin;
        port->fiopin = masked_field | masked_new_field;

        update_pins(state, port_number, previous_pins);

        break;
    }

    default:
        __builtin_unreachable();
    }

cleanup:
    return;
}

static const MemoryRegionOps lpc1768_gpio_port_ops = {
    .read = lpc1768_gpio_port_read,
    .write = lpc1768_gpio_port_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void lpc1768_gpio_reset(DeviceState * device)
{
    Lpc1768GpioState * state = LPC1768_GPIO(device);

    memset(state->ports, 0, sizeof(state->ports));
}

static void lpc1768_gpio_instance_init(Object * object)
{
    SysBusDevice * sysbus = SYS_BUS_DEVICE(object);
    Lpc1768GpioState * state = LPC1768_GPIO(object);

    memory_region_init_io(&state->port_config,
                          OBJECT(state),
                          &lpc1768_gpio_port_ops,
                          state,
                          "lpc1768,gpio,port_config",
                          0x4000);
    sysbus_init_mmio(sysbus, &state->port_config);

    qdev_init_gpio_out(DEVICE(state), state->out_lines, ARRAY_SIZE(state->out_lines));
}

static void lpc1768_gpio_class_init(ObjectClass * klass, void * data)
{
    DeviceClass * device_class = DEVICE_CLASS(klass);

    device_class->reset = lpc1768_gpio_reset;
}

static const TypeInfo lpc1768_gpio_info = {
    .name = TYPE_LPC1768_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = lpc1768_gpio_instance_init,
    .instance_size = sizeof(Lpc1768GpioState),
    .class_init = lpc1768_gpio_class_init,
};

static void lpc1768_gpio_register_types(void)
{
    type_register_static(&lpc1768_gpio_info);
}

type_init(lpc1768_gpio_register_types)
