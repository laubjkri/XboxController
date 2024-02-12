#ifndef HOG_HOST_H
#define HOG_HOST_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "boards/pico_w.h"
#include "xbox_controller.h"


typedef void (*GattEventHandler)(const uint8_t *data, uint16_t size);

int btstack_main(GattEventHandler);

typedef enum
{
    W4_WORKING,
    W4_FIND_HID_DEVICE,
    W4_CONNECTED,
    W4_ENCRYPTED,
    W4_FIND_HID_SERVICE,
    W4_FIND_HID_CHARACTERISTICS,
    W4_ENABLE_BOOT_KEYBOARD,
    W4_ENABLE_BOOT_MOUSE,
    READY,
    W4_TIMEOUT_THEN_SCAN,
    W4_TIMEOUT_THEN_RECONNECT,
    BT_STATE_END
} BtState;

// Static to have it here in the header
static const char* BtStates[] =
{
    "W4_WORKING",
    "W4_FIND_HID_DEVICE",
    "W4_CONNECTED",
    "W4_ENCRYPTED",
    "W4_FIND_HID_SERVICE",
    "W4_FIND_HID_CHARACTERISTICS",
    "W4_ENABLE_BOOT_KEYBOARD",
    "W4_ENABLE_BOOT_MOUSE",
    "READY",
    "W4_TIMEOUT_THEN_SCAN",
    "W4_TIMEOUT_THEN_RECONNECT"
};

extern BtState bt_state;



#endif