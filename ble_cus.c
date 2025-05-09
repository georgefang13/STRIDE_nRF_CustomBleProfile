#include "sdk_common.h"
#include "ble_cus.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t* p_cus, ble_evt_t const* p_ble_evt) {
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t* p_cus, ble_evt_t const* p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t* p_cus, ble_evt_t const* p_ble_evt) {
    NRF_LOG_INFO("on_write");
    ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check if the Custom value CCCD is written to and that the value is the appropriate length,
    // i.e 2 bytes. Client Characteristic Configuration Descriptor This case is required in order
    // for app_timer_start() to be called in main.c's on_cus_evt() via evt_handler() below
    if ((p_evt_write->handle == p_cus->custom_value_handles.cccd_handle) &&
        (p_evt_write->len == 2)) {
        NRF_LOG_INFO("CCCD written, call application event handler");
        if (p_cus->evt_handler != NULL) {
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)) {
                NRF_LOG_INFO(
                    "Custom value CCCD written to and service notification is enabled, BLE_CUS_EVT_NOTIFICATION_ENABLED");
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            } else {
                NRF_LOG_INFO(
                    "Custom value CCCD written to and service notification is NOT enabled, BLE_CUS_EVT_NOTIFICATION_DISABLED");
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }

    if (p_evt_write->handle == p_cus->custom_value_handles.value_handle) {
        // This should not be possible as the Custom Value is read-only
        NRF_LOG_INFO("Custom Value Characteristic written to");
        NRF_LOG_INFO("Attempt to write custom data\n");
        NRF_LOG_INFO("Value=%u\n", *p_evt_write->data);
    }

    if (p_evt_write->handle == p_cus->custom_gpio_handles.value_handle) {
        NRF_LOG_INFO("Custom GPIO Characteristic written to");
        if (p_evt_write->len == GPIO_PAYLOAD_SIZE_BYTES) {
            uint8_t led4_data_received = p_evt_write->data[GPIO_PAYLOAD_INDEX_LED4];
            uint8_t gpio11_data_received = p_evt_write->data[GPIO_PAYLOAD_INDEX_GPIO11_OUTPUT];

            bool want_led4_on = (bool)led4_data_received;  // no real data checking, non-zero is considered 'on'
            bool is_led4_on = nrf_gpio_pin_out_read(LED_4) == LEDS_ACTIVE_STATE;
            NRF_LOG_INFO("led4_data_received: %u", led4_data_received);
            NRF_LOG_INFO("want_led4_on: %i, is_led4_on: %i", want_led4_on, is_led4_on);
            if (want_led4_on != is_led4_on) {
                nrf_gpio_pin_toggle(LED_4);
            }

            bool want_gpio11_on = (bool)gpio11_data_received;  // no real data checking, non-zero is considered 'on'
            bool is_gpio11_on = (bool)nrf_gpio_pin_out_read(GPIO_P0_11_DIG_OUT);
            NRF_LOG_INFO("gpio11_data_received: %u", gpio11_data_received);
            NRF_LOG_INFO("want_gpio11_on: %i, is_gpio11_on: %i", want_gpio11_on, is_gpio11_on);
            if (want_gpio11_on != is_gpio11_on) {
                nrf_gpio_pin_toggle(GPIO_P0_11_DIG_OUT);
            }
        } else {
            NRF_LOG_INFO("CUSTOM_GPIO_CHAR_UUID written to but with unexpected size: %u",
                         p_evt_write->len);
        }
    }
}

void ble_cus_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context) {
    ble_cus_t* p_cus = (ble_cus_t*)p_context;

    // NRF_LOG_INFO("BLE event received. Event type = %d", p_ble_evt->header.evt_id);
    if (p_cus == NULL || p_ble_evt == NULL) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE event received. BLE_GAP_EVT_CONNECTED");
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE event received. BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_INFO("BLE event received. BLE_GATTS_EVT_WRITE");
            on_write(p_cus, p_ble_evt);
            break;
            /* Handling this event is not necessary
                    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
                        NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.");
                        break;
            */
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_cus_t* p_cus, const ble_cus_init_t* p_cus_init) {
    uint32_t err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    // char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
    // attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = VALUE_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = VALUE_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value,
                                               &p_cus->custom_value_handles);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Custom GPIO characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_gpio_char_add(ble_cus_t* p_cus, const ble_cus_init_t* p_cus_init) {
    uint32_t err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_GPIO_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = GPIO_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = GPIO_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value,
                                               &p_cus->custom_gpio_handles);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for initializing the custom value.
 *
 * @details The application calls this function to init the custom data.
 *
 * @note
 *
 * @param[in]   p_cus          Custom Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_init(ble_cus_t* p_cus) {
    NRF_LOG_INFO("In custom_value_init.");
    if (p_cus == NULL) {
        return NRF_ERROR_NULL;
    }

    uint8_t value_buffer[VALUE_PAYLOAD_SIZE_BYTES] = {0};
#if (HELLO_WORLD)
    memcpy(value_buffer, HELLO_WORLD_STR, sizeof(value_buffer));
#else
    for (int i = 0; i < VALUE_PAYLOAD_SIZE_BYTES; i++) {
        // Init by VALUE_PAYLOAD_SIZE_BYTES less so that first call custom_value_update() call has
        // the expected values
        value_buffer[i] = (uint8_t)((i - VALUE_PAYLOAD_SIZE_BYTES) % 256);
    }
#endif
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = VALUE_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);
    gatts_value.offset = 0;
    gatts_value.p_value = value_buffer;

    // Update database.
    return sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_value_handles.value_handle,
                                  &gatts_value);
}

/**@brief Function for initializing the custom LED data.
 *
 * @details The application calls this function to init the custom LED data.
 *
 * @note
 *
 * @param[in]   p_cus          Custom Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_gpio_init(ble_cus_t* p_cus) {
    NRF_LOG_INFO("In custom_gpio_init.");
    if (p_cus == NULL) {
        return NRF_ERROR_NULL;
    }

    uint8_t value_buffer[GPIO_PAYLOAD_SIZE_BYTES] = {0};
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = GPIO_PAYLOAD_SIZE_BYTES * sizeof(uint8_t);
    gatts_value.offset = 0;
    gatts_value.p_value = value_buffer;

    // Update database.
    return sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_gpio_handles.value_handle,
                                  &gatts_value);
}

uint32_t ble_cus_init(ble_cus_t* p_cus, const ble_cus_init_t* p_cus_init) {
    if (p_cus == NULL || p_cus_init == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->evt_handler = p_cus_init->evt_handler;
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code =
        sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add Custom Value characteristic
    err_code = custom_value_char_add(p_cus, p_cus_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = custom_value_init(p_cus);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = custom_gpio_char_add(p_cus, p_cus_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = custom_gpio_init(p_cus);

    return err_code;
}

/**@brief Function for update the custom data.
 *
 * @details This is where the value is actually updated, i.e. where logic should live to gather,
 * summarize, modify, etc.
 *
 * @note
 *
 * @param[in]   value_buffer          pointer to the data to be updated
 */
static void custom_value_update(uint8_t* value_buffer) {
    if (value_buffer) {
#if (HELLO_WORLD)
        static char pre_string[VALUE_PAYLOAD_SIZE_BYTES];
        static char post_string[VALUE_PAYLOAD_SIZE_BYTES];
        pre_string[0] = '\0';
        post_string[0] = '\0';
        memcpy(pre_string, value_buffer, sizeof(pre_string));
        char first_letter = value_buffer[0];
        memmove(value_buffer, value_buffer + 1, VALUE_PAYLOAD_SIZE_BYTES - 1);
        value_buffer[VALUE_PAYLOAD_SIZE_BYTES - 2] = first_letter;
        memcpy(post_string, value_buffer, sizeof(pre_string));
        NRF_LOG_DEBUG(" pre_string  > %s", pre_string);
        NRF_LOG_INFO(" post_string > %s", post_string);
#else  // #if (HELLO_WORLD)
# define MAX_DEBUG_SIZE_BYTES 10
#if (VALUE_PAYLOAD_SIZE_BYTES <= MAX_DEBUG_SIZE_BYTES)
// Don't create large strings if the data is big
#define STR_LEN (9 + VALUE_PAYLOAD_SIZE_BYTES * 4 + 1)
        static char indexes_string[STR_LEN];
        static char pre_string[STR_LEN];
        static char post_string[STR_LEN];
        indexes_string[0] = '\0';
        pre_string[0] = '\0';
        post_string[0] = '\0';
        char* indexes_ptr = indexes_string;
        char* pre_ptr = pre_string;
        char* post_ptr = post_string;
        for (uint8_t i = 0; i < VALUE_PAYLOAD_SIZE_BYTES; i++) {
            indexes_ptr += sprintf(indexes_ptr, "%3u,", i);
            pre_ptr += sprintf(pre_ptr, "%3u,", value_buffer[i]);
            // The next line is the only real work here, all else is debug related
            value_buffer[i] = (value_buffer[i] + VALUE_PAYLOAD_SIZE_BYTES) % 256;
            post_ptr += sprintf(post_ptr, "%3u,", value_buffer[i]);
        }
        NRF_LOG_INFO(" i    > %s", indexes_string);
        NRF_LOG_INFO(" pre  > %s", pre_string);
        NRF_LOG_INFO(" post > %s", post_string);
#else  // #if (VALUE_PAYLOAD_SIZE_BYTES < MAX_DEBUG_SIZE_BYTES)
        for (uint8_t i = 0; i < VALUE_PAYLOAD_SIZE_BYTES; i++) {
            value_buffer[i] = (value_buffer[i] + VALUE_PAYLOAD_SIZE_BYTES) % 256;
        }
#endif  // #if (VALUE_PAYLOAD_SIZE_BYTES < MAX_DEBUG_SIZE_BYTES)
#endif  // #if (HELLO_WORLD)
    } else {
        NRF_LOG_INFO("NULL value_buffer");
    }
}

uint32_t ble_cus_custom_value_update(ble_cus_t* p_cus) {
    NRF_LOG_DEBUG("In ble_cus_custom_value_update.");
    if (p_cus == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    uint8_t value_buffer[VALUE_PAYLOAD_SIZE_BYTES] = {0};
    ble_gatts_value_t gatts_value = {
        .len = VALUE_PAYLOAD_SIZE_BYTES, .offset = 0, .p_value = &(value_buffer[0])};
    err_code = sd_ble_gatts_value_get(p_cus->conn_handle, p_cus->custom_value_handles.value_handle,
                                      &gatts_value);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("  sd_ble_gatts_value_get: Len=%i, Offset=%i, err_code=%i", gatts_value.len,
                  gatts_value.offset, err_code);

    // Now actually update the data
    custom_value_update(value_buffer);

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->custom_value_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_INFO("sd_ble_gatts_hvx result: %x.", err_code);
        }
    } else {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE.");
    }

    return err_code;
}

uint32_t ble_cus_led4_update(ble_cus_t* p_cus) {
    NRF_LOG_INFO("In ble_cus_led4_update.");
    if (p_cus == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    uint8_t gpio_buffer[GPIO_PAYLOAD_SIZE_BYTES] = {0};
    ble_gatts_value_t gatts_value = {
        .len = GPIO_PAYLOAD_SIZE_BYTES, .offset = 0, .p_value = &(gpio_buffer[0])};
    err_code = sd_ble_gatts_value_get(p_cus->conn_handle, p_cus->custom_gpio_handles.value_handle,
                                      &gatts_value);

    nrf_gpio_pin_toggle(LED_4);
    bool led4_new_state = nrf_gpio_pin_out_read(LED_4) == LEDS_ACTIVE_STATE;
    NRF_LOG_INFO("led4_new_state: %u", led4_new_state);
    gpio_buffer[GPIO_PAYLOAD_INDEX_LED4] = led4_new_state ? 1 : 0;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_gpio_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("err_code: %u", err_code);
        return err_code;
    }

    err_code = ble_cus_gpio_data_notify(p_cus);

    return err_code;
}

uint32_t ble_cus_gpio_data_notify(ble_cus_t* p_cus) {
    NRF_LOG_DEBUG("In ble_cus_gpio_data_notify.");
    if (p_cus == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    // Send value if connected and notifying.
    if (p_cus->conn_handle != BLE_CONN_HANDLE_INVALID) {
        uint8_t gpio_buffer[GPIO_PAYLOAD_SIZE_BYTES] = {0};
        ble_gatts_value_t gatts_value = {
            .len = GPIO_PAYLOAD_SIZE_BYTES, .offset = 0, .p_value = &(gpio_buffer[0])};
        err_code = sd_ble_gatts_value_get(p_cus->conn_handle, p_cus->custom_gpio_handles.value_handle,
                                          &gatts_value);
        APP_ERROR_CHECK(err_code);

        // Update the input pin's value if necessary
        uint8_t pin12 = (uint8_t)nrf_gpio_pin_read(GPIO_P0_12_DIG_IN_PULLUP);
        if (gpio_buffer[GPIO_PAYLOAD_INDEX_GPIO12_INPUT] != pin12) {
            gpio_buffer[GPIO_PAYLOAD_INDEX_GPIO12_INPUT] = pin12;
            // Update database.
            err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->custom_gpio_handles.value_handle,
                                              &gatts_value);
            if (err_code != NRF_SUCCESS) {
                NRF_LOG_INFO("err_code: %u", err_code);
                return err_code;
            }
        }

        NRF_LOG_INFO("led4(out) %u, pin11(out) %u, pin12(in) %u",
            gpio_buffer[GPIO_PAYLOAD_INDEX_LED4],
            gpio_buffer[GPIO_PAYLOAD_INDEX_GPIO11_OUTPUT],
            gpio_buffer[GPIO_PAYLOAD_INDEX_GPIO12_INPUT]);

        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->custom_gpio_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_INFO("sd_ble_gatts_hvx result: %x.", err_code);
        }
    } else {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE.");
    }

    return err_code;
}
