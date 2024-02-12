/*
 * Copyright (C) 2020 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

/* HID Host LE
 *
 * @text This example implements a minimal HID-over-GATT Host. It scans for LE HID devices, connects to it,
 * discovers the Characteristics relevant for the HID Service and enables Notifications on them.
 * It then dumps all Boot Keyboard and Mouse Input Reports
 */

#include <inttypes.h>
#include <btstack_tlv.h>

#include "btstack_config.h"
#include "btstack.h"
#include "hog_host.h"
#include "keytable.h"


// TAG to store remote device address and type in TLV
#define TLV_TAG_HOGD ((((uint32_t) 'H') << 24 ) | (((uint32_t) 'O') << 16) | (((uint32_t) 'G') << 8) | 'D')

#define UNI_BT_HID_APPEARANCE_KEYBOARD      0x3c1
#define UNI_BT_HID_APPEARANCE_MOUSE         0x3c2
#define UNI_BT_HID_APPEARANCE_JOYSTICK      0x3c3
#define UNI_BT_HID_APPEARANCE_GAMEPAD       0x3c4

typedef struct {
    bd_addr_t addr;
    bd_addr_type_t addr_type;
} le_device_addr_t;

static GattEventHandler gatt_event_callback = NULL;
static GattEventHandler gatt_keyboard_callback = NULL;

const char* get_bt_state_name(BtState bt_state)
{
    if (bt_state >= 0 && bt_state < BT_STATE_END)
    {
        return BtStates[bt_state];
    }
    else
    {
        return "Unknown";
    }
}

BtState bt_state;

static void set_app_state(BtState state)
{
    bt_state = state;
    VERBOSE_LOG("================= State: %s =================\n", get_bt_state_name(state));
}


static le_device_addr_t remote_device;
static hci_con_handle_t connection_handle;

static uint8_t hid_descriptor_storage[512];
static btstack_packet_callback_registration_t sm_event_callback_registration;

static uint16_t hids_cid;
static hid_protocol_mode_t protocol_mode = HID_PROTOCOL_MODE_REPORT;

// used for GATT queries
static gatt_client_service_t              hid_service;
static gatt_client_characteristic_t       protocol_mode_characteristic;
static gatt_client_characteristic_t       boot_keyboard_input_characteristic;
static gatt_client_characteristic_t       boot_mouse_input_characteristic;
static gatt_client_characteristic_t       hid_information_characteristic;
static gatt_client_characteristic_t       hid_controlpoint_characteristic;
static gatt_client_characteristic_t       report_map_characteristic;
static gatt_client_characteristic_t       report_characteristic;
static gatt_client_characteristic_t       empty_characteristic;
static uint8_t                            characteristic_size =      sizeof(gatt_client_characteristic_t);

static gatt_client_notification_t         keyboard_notifications;
static gatt_client_notification_t         mouse_notifications;
static gatt_client_notification_t         default_notifications;

// used to implement connection timeout and reconnect timer
static btstack_timer_source_t connection_timer;

// register for events from HCI/GAP and SM
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// used to store remote device in TLV
static const btstack_tlv_t * btstack_tlv_singleton_impl;
static void *                btstack_tlv_singleton_context;

// Simplified US Keyboard with Shift modifier

/**
 * @section HOG Boot Keyboard Handler
 * @text Boot Keyboard Input Report contains a report of format
 * [ modifier, reserved, 6 x usage for key 1..6 from keyboard usage]
 * Track new usages, map key usage to actual character and simulate terminal
 */

/* last_keys stores keyboard report to detect new key down events */
#define NUM_KEYS 6
static uint8_t last_keys[NUM_KEYS];

static void handle_notification_event(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) return;
    const uint8_t* data = gatt_event_notification_get_value(packet);

    if (gatt_event_callback != NULL)
    {
        gatt_event_callback(data, size);
    }

    // int i;
    // for (i = 0; i < size; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");
}

static void handle_boot_keyboard_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) return;
    const uint8_t * data = gatt_event_notification_get_value(packet);

    if (gatt_keyboard_callback != NULL) {
        gatt_keyboard_callback(data, size);
    }

    // int i;
    // for (i = 0; i < size; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");
}

/**
 * @section HOG Boot Mouse Handler
 * @text Boot Mouse Input Report contains a report of format
 * [ buttons, dx, dy, dz = scroll wheel]
 * Decode packet and print on stdout
 *
 * @param packet_type
 * @param channel
 * @param packet
 * @param size
 */
static void handle_boot_mouse_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) return;
    const uint8_t * data = gatt_event_notification_get_value(packet);
    uint8_t buttons =          data[0];
    int8_t dx       = (int8_t) data[1];
    int8_t dy       = (int8_t) data[2];
    int8_t dwheel   = (int8_t) data[3];
    printf("Mouse: %i, %i - wheel %i - buttons 0x%02x\n", dx, dy, dwheel, buttons);
}

static void hid_handle_input_report(uint8_t service_index, const uint8_t* report, uint16_t report_len)
{
    // check if HID Input Report

    if (report_len < 1) return;

    btstack_hid_parser_t parser;

    switch (protocol_mode)
    {
        case HID_PROTOCOL_MODE_BOOT:            
            btstack_hid_parser_init(&parser,
                btstack_hid_get_boot_descriptor_data(),
                btstack_hid_get_boot_descriptor_len(),
                HID_REPORT_TYPE_INPUT, report, report_len);
            break;

        default:
            btstack_hid_parser_init(&parser,
                hids_client_descriptor_storage_get_descriptor_data(hids_cid, service_index),
                hids_client_descriptor_storage_get_descriptor_len(hids_cid, service_index),
                HID_REPORT_TYPE_INPUT, report, report_len);
            break;
    }

    int shift = 0;
    uint8_t new_keys[NUM_KEYS];
    memset(new_keys, 0, sizeof(new_keys));
    int     new_keys_count = 0;
    while (btstack_hid_parser_has_more(&parser))
    {
        uint16_t usage_page;
        uint16_t usage;
        int32_t  value;
        btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);
        if (usage_page != 0x07) continue;
        switch (usage)
        {
            case 0xe1:
            case 0xe6:
                if (value)
                {
                    shift = 1;
                }
                continue;
            case 0x00:
                continue;
            default:
                break;
        }
        if (usage >= sizeof(keytable_us_none)) continue;

        // store new keys
        new_keys[new_keys_count++] = (uint8_t)usage;

        // check if usage was used last time (and ignore in that case)
        int i;
        for (i = 0; i < NUM_KEYS; i++)
        {
            if (usage == last_keys[i])
            {
                usage = 0;
            }
        }
        if (usage == 0) continue;

        uint8_t key;
        if (shift)
        {
            key = keytable_us_shift[usage];
        }
        else
        {
            key = keytable_us_none[usage];
        }
        if (key == CHAR_ILLEGAL) continue;
        if (key == CHAR_BACKSPACE)
        {
            printf("\b \b");    // go back one char, print space, go back one char again
            continue;
        }
        printf("%c", key);
    }
    memcpy(last_keys, new_keys, NUM_KEYS);
}


/**
 * @section Test if advertisement contains HID UUID
 * @param packet
 * @param size
 * @returns true if it does
 */
static bool adv_event_contains_hid_service(const uint8_t * packet){
    const uint8_t * ad_data = gap_event_advertising_report_get_data(packet);
    uint8_t ad_len = gap_event_advertising_report_get_data_length(packet);
    return ad_data_contains_uuid16(ad_len, ad_data, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE);
}

// tests
static void get_advertisement_data(const uint8_t* adv_data, uint8_t adv_size, uint16_t* appearance, char* name) {
    ad_context_t context;

    for (ad_iterator_init(&context, adv_size, (uint8_t*)adv_data); ad_iterator_has_more(&context);
         ad_iterator_next(&context)) {
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t size = ad_iterator_get_data_len(&context);
        const uint8_t* data = ad_iterator_get_data(&context);

        int i;
        // Assigned Numbers GAP

        switch (data_type) {
            case BLUETOOTH_DATA_TYPE_FLAGS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_16_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_32_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_128_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                for (i = 0; i < size; i++) {
                    name[i] = data[i];
                }
                name[size] = 0;
                break;
            case BLUETOOTH_DATA_TYPE_TX_POWER_LEVEL:
                break;
            case BLUETOOTH_DATA_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE:
                break;
            case BLUETOOTH_DATA_TYPE_SERVICE_DATA:
                break;
            case BLUETOOTH_DATA_TYPE_PUBLIC_TARGET_ADDRESS:
            case BLUETOOTH_DATA_TYPE_RANDOM_TARGET_ADDRESS:
                break;
            case BLUETOOTH_DATA_TYPE_APPEARANCE:
                // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
                *appearance = little_endian_read_16(data, 0);
                break;
            case BLUETOOTH_DATA_TYPE_ADVERTISING_INTERVAL:
                break;
            case BLUETOOTH_DATA_TYPE_3D_INFORMATION_DATA:
                break;
            case BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA:  // Manufacturer Specific Data
                break;
            case BLUETOOTH_DATA_TYPE_CLASS_OF_DEVICE:
                printf("class of device: %#x\n", little_endian_read_16(data, 0));
                break;
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_HASH_C:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_RANDOMIZER_R:
            case BLUETOOTH_DATA_TYPE_DEVICE_ID:
                printf("device id: %#x\n", little_endian_read_16(data, 0));
                break;
            case BLUETOOTH_DATA_TYPE_LE_BLUETOOTH_DEVICE_ADDRESS:
            case BLUETOOTH_DATA_TYPE_MESH_BEACON:
            case BLUETOOTH_DATA_TYPE_MESH_MESSAGE:
                // Safely ignore these messages
                break;
            case BLUETOOTH_DATA_TYPE_SECURITY_MANAGER_OUT_OF_BAND_FLAGS:
                // fall-through
            default:
                printf("Advertising Data Type 0x%2x not handled yet\n", data_type);
                break;
        }
    }
}

static void adv_event_get_data(const uint8_t* packet, uint16_t* appearance, char* name) {
    const uint8_t* ad_data;
    uint16_t ad_len;

    ad_data = gap_event_advertising_report_get_data(packet);
    ad_len = gap_event_advertising_report_get_data_length(packet);

    // if (!ad_data_contains_uuid16(ad_len, ad_data, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE))
    get_advertisement_data(ad_data, ad_len, appearance, name);
    printf("appearance: %#x\n", *appearance);
    printf("name: %s\n", name);

    switch (*appearance) {
        case UNI_BT_HID_APPEARANCE_MOUSE:
            printf("Device '%s' identified as Mouse\n", name);
            break;
        case UNI_BT_HID_APPEARANCE_JOYSTICK:
            printf("Device '%s' identified as Joystick\n", name);
            break;
        case UNI_BT_HID_APPEARANCE_GAMEPAD:
            printf("Device '%s' identified as Gamepad\n", name);
            break;
        case UNI_BT_HID_APPEARANCE_KEYBOARD:
            printf("Device '%s' identified as Keyboard\n", name);
            break;
        default:
            printf("Unsupported appearance: %#x\n", appearance);
            break;
    }

}
// end tests

#pragma region Aux functions
/**
 * Start scanning
 */
static void hog_start_scan(void){
    printf("Scanning for LE HID devices...\n");
    set_app_state(W4_FIND_HID_DEVICE);
    // Passive scanning, 100% (scan interval = scan window)
    gap_set_scan_parameters(0,48,48);
    gap_start_scan();
}

/**
 * Handle timeout for outgoing connection
 * @param ts
 */
static void hog_connection_timeout(btstack_timer_source_t * ts){
    UNUSED(ts);
    printf("Timeout - abort connection\n");
    gap_connect_cancel();
    hog_start_scan();
}


/**
 * Connect to remote device but set timer for timeout
 */
static void hog_connect(void) {
    // set timer
    btstack_run_loop_set_timer(&connection_timer, 10000);
    btstack_run_loop_set_timer_handler(&connection_timer, &hog_connection_timeout);
    btstack_run_loop_add_timer(&connection_timer);
    set_app_state(W4_CONNECTED);
    gap_connect(remote_device.addr, remote_device.addr_type);
}

/**
 * Handle timer event to trigger reconnect
 * @param ts
 */
static void hog_reconnect_timeout(btstack_timer_source_t * ts){
    UNUSED(ts);
    switch (bt_state){
        case W4_TIMEOUT_THEN_RECONNECT:
            hog_connect();
            break;
        case W4_TIMEOUT_THEN_SCAN:
            hog_start_scan();
            break;
        default:
            break;
    }
}

/**
 * Start connecting after boot up: connect to last used device if possible, start scan otherwise
 */
static void hog_start_connect(void){
    // check if we have a bonded device
    btstack_tlv_get_instance(&btstack_tlv_singleton_impl, &btstack_tlv_singleton_context);
    if (btstack_tlv_singleton_impl){
        int len = btstack_tlv_singleton_impl->get_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (uint8_t *) &remote_device, sizeof(remote_device));
        if (len == sizeof(remote_device)){
            printf("Bonded, connect to device with %s address %s ...\n", remote_device.addr_type == 0 ? "public" : "random" , bd_addr_to_str(remote_device.addr));
            hog_connect();
            return;
        }
    }
    // otherwise, scan for HID devices
    hog_start_scan();
}

/**
 * In case of error, disconnect and start scanning again
 */
static void handle_outgoing_connection_error(void){
    printf("Error occurred, disconnect and start over\n");
    gap_disconnect(connection_handle);
    hog_start_scan();
}
#pragma endregion

/**
 * Handle GATT Client Events dependent on current state
 *
 * @param packet_type
 * @param channel
 * @param packet
 * @param size
 */
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint8_t packet_packet_type = hci_event_packet_get_type(packet);
    FUNCTION_CALL_LOG("handle_gatt_client_event() called. packet_type: %d, packet-packet type: %d\n", packet_type, packet_packet_type);

#pragma region HID report stuff
    // The xbox controller does not seem to transmit a HID report
    if (packet_packet_type == HCI_EVENT_GATTSERVICE_META)
    {
        uint8_t subevent = hci_event_gattservice_meta_get_subevent_code(packet);

        VERBOSE_LOG("GATT event is HCI_EVENT_GATTSERVICE_META. Subevent: %d\n", subevent);
        uint8_t status;
        switch (subevent)
        {
            case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
                status = gattservice_subevent_hid_service_connected_get_status(packet);
                switch (status)
                {
                    case ERROR_CODE_SUCCESS:
                        printf("HID service client connected, found %d services\n",
                            gattservice_subevent_hid_service_connected_get_num_instances(packet));

                        // store device as bonded
                        if (btstack_tlv_singleton_impl)
                        {
                            btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t*)&remote_device, sizeof(remote_device));
                        }
                        // done
                        printf("Ready - please start typing or mousing..\n");
                        set_app_state(READY);
                        break;
                    default:
                        printf("HID service client connection failed, status 0x%02x.\n", status);
                        handle_outgoing_connection_error();
                        break;
                }
                break;

            case GATTSERVICE_SUBEVENT_HID_REPORT:
                hid_handle_input_report(
                    gattservice_subevent_hid_report_get_service_index(packet),
                    gattservice_subevent_hid_report_get_report(packet),
                    gattservice_subevent_hid_report_get_report_len(packet));
                break;

            default:
                break;
        }
    }
#pragma endregion

    gatt_client_characteristic_t characteristic;
    uint8_t att_status;
    static uint8_t boot_protocol_mode = 0;

    switch (bt_state) {
        case W4_FIND_HID_SERVICE:
            switch (packet_packet_type) 
            {
                case GATT_EVENT_SERVICE_QUERY_RESULT:
                    // store service (we expect only one)
                    gatt_event_service_query_result_get_service(packet, &hid_service);
                    break;

                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("Query failed, ATT Error 0x%02x.\n", att_status);
                        handle_outgoing_connection_error();
                        break;
                    }
                    uint8_t result = gatt_client_discover_characteristics_for_service(&handle_gatt_client_event, connection_handle, &hid_service);
                    printf("Find required HID Service Characteristics result: %u\n", result);                    
                    set_app_state(W4_FIND_HID_CHARACTERISTICS);
                    break;

                default:
                    break;
            }
            break;

        case W4_FIND_HID_CHARACTERISTICS:
            switch (packet_packet_type)
            {
                case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
                    // get characteristic
                    gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
                    VERBOSE_LOG("GATT characteristic result. Characteristic: %u\n", characteristic.uuid16);
                    
                    switch (characteristic.uuid16){
                        case ORG_BLUETOOTH_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT:
                            VERBOSE_LOG("Found CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT, value handle 0x%04x\n", characteristic.value_handle);
                            memcpy(&boot_keyboard_input_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT:
                            VERBOSE_LOG("Found CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT, value handle 0x%04x\n", characteristic.value_handle);
                            memcpy(&boot_mouse_input_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_PROTOCOL_MODE:
                            VERBOSE_LOG("Found CHARACTERISTIC_PROTOCOL_MODE\n");
                            memcpy(&protocol_mode_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_HID_INFORMATION:
                            VERBOSE_LOG("HID information characteristic recieved\n");
                            memcpy(&hid_information_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_HID_CONTROL_POINT:
                            VERBOSE_LOG("HID control point characteristic recieved\n");
                            memcpy(&hid_controlpoint_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_REPORT_MAP:
                            VERBOSE_LOG("Report map characteristic recieved\n");
                            memcpy(&report_map_characteristic, &characteristic, characteristic_size);
                            break;

                        case ORG_BLUETOOTH_CHARACTERISTIC_REPORT:
                            VERBOSE_LOG("Report characteristic recieved\n");
                            memcpy(&report_characteristic, &characteristic, characteristic_size);
                            break;
                            
                        default:
                            //set_app_state(W4_ENABLE_BOOT_KEYBOARD);
                            //gatt_client_write_client_characteristic_configuration(&handle_gatt_client_event, connection_handle, &boot_keyboard_input_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                            break;
                    }
                    break;

                case GATT_EVENT_QUERY_COMPLETE:
                    VERBOSE_LOG("GATT query complete 1\n");
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("Query failed, ATT Error 0x%02x.\n", att_status);
                        handle_outgoing_connection_error();
                        break;
                    }
                    //printf("Enable Notifications for Boot Keyboard Input Report..\n");
                    //set_app_state(W4_ENABLE_BOOT_KEYBOARD);
                    //gatt_client_write_client_characteristic_configuration(&handle_gatt_client_event, connection_handle, &boot_keyboard_input_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    
                    printf("Enable Notifications for whatever characteristic..\n");
                    gatt_client_write_client_characteristic_configuration(&handle_gatt_client_event, connection_handle, &empty_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    // setup listener
                    gatt_client_listen_for_characteristic_value_updates(&default_notifications, &handle_notification_event, connection_handle, &empty_characteristic);
                    set_app_state(READY);
                    
                    // We might need this for new devices, not sure how to unbind with my current device
                    //// store device as bonded
                    //if (btstack_tlv_singleton_impl)
                    //{
                    //    btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t*)&remote_device, sizeof(remote_device));
                    //}


                    break;

                default:
                    break;
            }
            break;

        case W4_ENABLE_BOOT_KEYBOARD:
            switch (packet_packet_type){
                case GATT_EVENT_QUERY_COMPLETE:
                    VERBOSE_LOG("GATT query complete 2\n");
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("Query failed, ATT Error 0x%02x.\n", att_status);
                        handle_outgoing_connection_error();
                        break;
                    }
                    // setup listener
                    gatt_client_listen_for_characteristic_value_updates(&keyboard_notifications, &handle_boot_keyboard_event, connection_handle, &boot_keyboard_input_characteristic);
                    //
                    printf("Enable Notifications for Boot Mouse Input Report..\n");
                    set_app_state(W4_ENABLE_BOOT_MOUSE);
                    uint8_t status = gatt_client_write_client_characteristic_configuration(&handle_gatt_client_event, connection_handle, &boot_mouse_input_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_INDICATION);
                    printf("Characteristic write status: %d\n", status);
                    
                    break;
                default:
                    break;
            }
            break;

        case W4_ENABLE_BOOT_MOUSE:
            printf("case boot mouse\n");
            switch (packet_packet_type) {
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("Query failed, ATT Error 0x%02x.\n", att_status);
                        handle_outgoing_connection_error();
                        break;
                    }
                    // setup listener
                    gatt_client_listen_for_characteristic_value_updates(&mouse_notifications, &handle_boot_mouse_event, connection_handle, &boot_mouse_input_characteristic);

                    // switch to boot mode
                    printf("Set Protocol Mode to Boot Mode..\n");
                    gatt_client_write_value_of_characteristic_without_response(connection_handle, protocol_mode_characteristic.value_handle, 1, &boot_protocol_mode);

                    // store device as bonded
                    if (btstack_tlv_singleton_impl){
                        btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t *) &remote_device, sizeof(remote_device));
                    }
                    // done
                    printf("Ready - please start typing or mousing..\n");
                    set_app_state(READY);
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

/* Packet Handler */
static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{    
    UNUSED(channel);
    UNUSED(size);

    uint8_t packet_packet_type = hci_event_packet_get_type(packet);
    FUNCTION_CALL_LOG("hci_packet_handler() called. packet_type: %d, packet-packet type: %d\n", packet_type, packet_packet_type);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    uint16_t appearance = 0;
    char name[64] = { 0 };
    
    switch (packet_packet_type) {

        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            btstack_assert(bt_state == W4_WORKING);
            hog_start_connect();
            break;

        case GAP_EVENT_ADVERTISING_REPORT:
            if (bt_state != W4_FIND_HID_DEVICE) break;
            if (adv_event_contains_hid_service(packet) == false) break;
            // stop scan
            gap_stop_scan();
            // store remote device address and type
            gap_event_advertising_report_get_address(packet, remote_device.addr);
            remote_device.addr_type = gap_event_advertising_report_get_address_type(packet);
            adv_event_get_data(packet, &appearance, name);
            // connect
            printf("Found, connect to device with %s address %s ...\n", remote_device.addr_type == 0 ? "public" : "random" , bd_addr_to_str(remote_device.addr));
            hog_connect();
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            if (bt_state != READY) break;
                    
            connection_handle = HCI_CON_HANDLE_INVALID;
            switch (bt_state){
                case READY:
                    printf("\nDisconnected, try to reconnect...\n");
                    set_app_state(W4_TIMEOUT_THEN_RECONNECT);
                    break;
                default:
                    printf("\nDisconnected, start over...\n");
                    set_app_state(W4_TIMEOUT_THEN_SCAN);
                    break;
            }
            // set timer
            btstack_run_loop_set_timer(&connection_timer, 100);
            btstack_run_loop_set_timer_handler(&connection_timer, &hog_reconnect_timeout);
            btstack_run_loop_add_timer(&connection_timer);
            break;

        case HCI_EVENT_LE_META:
            // wait for connection complete
            if (hci_event_le_meta_get_subevent_code(packet) != HCI_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            if (bt_state != W4_CONNECTED) return;
            btstack_run_loop_remove_timer(&connection_timer);
            connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            // request security
            set_app_state(W4_ENCRYPTED);
            sm_request_pairing(connection_handle);
            break;

        default:
            //printf("Unhandled HCI event packet type: %u\n", packet_packet_type);
            break;
    }
    
}

 /**
  * The SM packet handler receives Security Manager Events required for pairing.
  * It also receives events generated during Identity Resolving
  *
  * @param packet_type
  * @param channel
  * @param packet
  * @param size
  */
static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    
    uint8_t packet_packet_type = hci_event_packet_get_type(packet);
    FUNCTION_CALL_LOG("sm_packet_handler() called. packet_type: %d, packet-packet type: %d\n", packet_type, packet_packet_type);
    
    UNUSED(channel);
    UNUSED(size);
    hci_con_handle_t con_handle;    
    bd_addr_t addr;
    uint8_t device;

    if (packet_type != HCI_EVENT_PACKET) {
        printf("sm_packet_handler: unsupported packet type: %#x\n", packet_type);
        return;
    }   
    

    switch (packet_packet_type) {

        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("Just works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;

        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            printf("Confirming numeric comparison: %"PRIu32"\n", sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;

        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            printf("Display Passkey: %"PRIu32"\n", sm_event_passkey_display_number_get_passkey(packet));
            break;

        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            printf("SM_EVENT_IDENTITY_RESOLVING_STARTED\n");
            break;

        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            sm_event_identity_created_get_address(packet, addr);
            printf("Identity resolving failed for %s\n\n", bd_addr_to_str(addr));
            break;

        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
            printf("Identity resolved: type %u address %s\n",
                 sm_event_identity_resolving_succeeded_get_identity_addr_type(packet), bd_addr_to_str(addr));
            break;

        case SM_EVENT_PAIRING_STARTED:
            printf("SM_EVENT_PAIRING_STARTED\n");
            break;

        case SM_EVENT_IDENTITY_CREATED:
            sm_event_identity_created_get_identity_address(packet, addr);
            printf("Identity created: type %u address %s\n", sm_event_identity_created_get_identity_addr_type(packet),
                 bd_addr_to_str(addr));
            break;

        case SM_EVENT_REENCRYPTION_STARTED:
            sm_event_reencryption_complete_get_address(packet, addr);
            printf("Bonding information exists for addr type %u, identity addr %s -> start re-encryption\n",
                 sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(addr));
            break;

        case SM_EVENT_REENCRYPTION_COMPLETE:
            con_handle = sm_event_reencryption_complete_get_handle(packet);

            switch (sm_event_reencryption_complete_get_status(packet)) {

                case ERROR_CODE_SUCCESS:
                    printf("Re-encryption complete, success\n");
                    set_app_state(W4_FIND_HID_SERVICE);
                    gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connection_handle, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE);
                    break;

                case ERROR_CODE_CONNECTION_TIMEOUT:
                    printf("Re-encryption failed, timeout\n");
                    gap_disconnect(con_handle); // changed from hog
                    break;

                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    printf("Re-encryption failed, disconnected\n");
                    gap_delete_bonding(packet_packet_type, addr);
                    gap_disconnect(con_handle); // changed from hog
                    break;

                case ERROR_CODE_PIN_OR_KEY_MISSING:
                    printf("Re-encryption failed, bonding information missing\n\n");
                    printf("Assuming remote lost bonding information\n");
                    printf("Deleting local bonding information and start new pairing...\n");
                    sm_event_reencryption_complete_get_address(packet, addr);
                    packet_packet_type = sm_event_reencryption_started_get_addr_type(packet);
                    gap_delete_bonding(packet_packet_type, addr);
                    sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
                    break;

                default:
                    break;
            }
            break;

        case SM_EVENT_PAIRING_COMPLETE:

            switch (sm_event_pairing_complete_get_status(packet)){

                case ERROR_CODE_SUCCESS:
                    printf("Pairing complete, success\n");
                    // continue - query primary services
                    printf("Search for HID service.\n");
                    set_app_state(W4_FIND_HID_SERVICE);
                    gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connection_handle, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE);
                    break;

                case ERROR_CODE_CONNECTION_TIMEOUT:
                    printf("Pairing failed, timeout\n");
                    break;

                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    printf("Pairing failed, disconnected\n");
                    break;

                case ERROR_CODE_AUTHENTICATION_FAILURE:
                    printf("Pairing failed, reason = %u\n", sm_event_pairing_complete_get_reason(packet));
                    break;

                default:
                    break;
            }
            break;

        default:
            //printf("Unhandled SM event packet type: %u\n", packet_packet_type);
            break;
    }
}


int btstack_main(GattEventHandler event_callback)
{
    gatt_event_callback = event_callback;

    // HCI = Host Controller Interface
    // Is a low level protocol that handles stuff such as setting a device to be discoverable or initiating connections. 
    // Here we set the handler:
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Register for events from Security Manager
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    // L2CAP = Logical Link Control and Adaptation Protocol
    // A data link layer bluetooth protocol
    l2cap_init();

    // SM = Security Manager Protocol
    // It handles pairing, bonding, encryption, identity, authentication.
    sm_init();
    // sm_set_secure_connections_only_mode(true);
    // sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);


    // GATT = Generic Attribute Profile
    // It builds upon the Attribute Protocol (ATT).
    // It contains information about which attributes are available on a specific use case.
    // One example could be a temperature sensor which has predifined services and characteristics.
    gatt_client_init();

    /*hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));*/

    /* LISTING_END */

    // Disable stdout buffering
	setvbuf(stdin, NULL, _IONBF, 0);

    set_app_state(W4_WORKING);

    // Turn on the device
    hci_power_control(HCI_POWER_ON);
    return 0;
}