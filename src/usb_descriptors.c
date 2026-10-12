// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// USB descriptors: CDC (debug console) + MSC composite.

#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"

#define USB_VID 0xCafe
#define USB_PID 0x5a10
#define USB_BCD 0x0200

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    // IAD for CDC
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_MSC,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_MSC_OUT   0x03
#define EPNUM_MSC_IN    0x83

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

static const char *const string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   // English (0x0409)
    "Bluebridge",                   // 1: Manufacturer
    "Bluebridge",                   // 2: Product
    NULL,                           // 3: Serial, from flash unique ID
    "Bluebridge Console",           // 4: CDC interface
    "Bluebridge Storage",           // 5: MSC interface
};

static uint16_t desc_str[32 + 1];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;
    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    const char *str;

    switch (index) {
    case STRID_LANGID:
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
        break;
    default:
        if (index == STRID_SERIAL) {
            pico_get_unique_board_id_string(serial, sizeof serial);
            str = serial;
        } else if (index < sizeof string_desc_arr / sizeof string_desc_arr[0]) {
            str = string_desc_arr[index];
        } else {
            return NULL;
        }
        chr_count = strlen(str);
        if (chr_count > 32)
            chr_count = 32;
        for (size_t i = 0; i < chr_count; i++)
            desc_str[1 + i] = (uint8_t)str[i];
        break;
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
