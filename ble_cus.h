#ifndef BLE_CUS_H__
#define BLE_CUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// HELLO_WORLD will send a string in custom data, otherwise just generic data
#define HELLO_WORLD 1
#if (HELLO_WORLD)
// Start string offset by 1 so that first custom_value_update() call sets to expected value
#define HELLO_WORLD_STR "_Hello_World"
#define VALUE_PAYLOAD_SIZE_BYTES sizeof(HELLO_WORLD_STR)
#else
#define VALUE_PAYLOAD_SIZE_BYTES 3
#endif
#define GPIO_PAYLOAD_INDEX_LED4 0
#define GPIO_PAYLOAD_INDEX_GPIO11_OUTPUT 1
#define GPIO_PAYLOAD_INDEX_GPIO12_INPUT 2
#define GPIO_PAYLOAD_SIZE_BYTES 3

#define GPIO_P0_11_DIG_OUT 11  // P0.11
#define GPIO_P0_12_DIG_IN_PULLUP 12  // P0.12

/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_DEF(_name)  \
    static ble_cus_t _name; \
    NRF_SDH_BLE_OBSERVER(_name##_obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_cus_on_ble_evt, &_name)

// CUSTOM_SERVICE_UUID_BASE f364adc9-00b0-4240-ba50-05ca45bf8abc

#define CUSTOM_SERVICE_UUID_BASE                                                                  \
    {                                                                                             \
        0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, 0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, \
            0xF3                                                                                  \
    }

#define CUSTOM_SERVICE_UUID 0x1400
#define CUSTOM_VALUE_CHAR_UUID 0x1401  // read-only
#define CUSTOM_GPIO_CHAR_UUID 0x1402    // read and write

/**@brief Custom Service event type. */
typedef enum {
    // Custom value notification enabled event
    BLE_CUS_EVT_NOTIFICATION_ENABLED,
    // Custom value notification disabled event
    BLE_CUS_EVT_NOTIFICATION_DISABLED,
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED
} ble_cus_evt_type_t;

/**@brief Custom Service event. */
typedef struct {
    // Type of event
    ble_cus_evt_type_t evt_type;
} ble_cus_evt_t;

// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_s ble_cus_t;

/**@brief Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t)(ble_cus_t* p_bas, ble_cus_evt_t* p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    // Event handler to be called for handling events in the Custom Service
    ble_cus_evt_handler_t evt_handler;
    // Initial custom value
    uint8_t initial_custom_value;
    // Initial security level for Custom characteristics attribute
    ble_srv_cccd_security_mode_t custom_value_char_attr_md;
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s {
    // Event handler to be called for handling events in the Custom Service
    ble_cus_evt_handler_t evt_handler;
    // Handle of Custom Service (as provided by the BLE stack)
    uint16_t service_handle;
    // Handles related to the Custom characteristics
    ble_gatts_char_handles_t custom_value_handles;
    ble_gatts_char_handles_t custom_gpio_handles;
    // Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if
    // not in a connection)
    uint16_t conn_handle;
    uint8_t uuid_type;
};

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t* p_cus, const ble_cus_init_t* p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_cus_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note
 *
 * @param[in]   p_cus          Custom Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_custom_value_update(ble_cus_t* p_cus);

/**@brief Function for updating the LED data.
 *
 * @details The application calls this function when the LED value should be updated. If
 *          notification has been enabled, the GPIO characteristic is sent to the client.
 *
 * @note
 *
 * @param[in]   p_cus          Custom Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_led4_update(ble_cus_t* p_cus);

/**@brief Function for updating the LED data.
 *
 * @details  If notification has been enabled, the GPIO characteristic is sent to the client.
 *
 * @note
 *
 * @param[in]   p_cus          Custom Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_gpio_data_notify(ble_cus_t* p_cus);
#endif  // BLE_CUS_H__
