/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"


void ble_os_on_ble_evt(ble_os_t * p_os, ble_evt_t * p_ble_evt)
{
    // OUR_JOB: Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_os->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_os->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        
        case BLE_GATTS_EVT_WRITE:
            nrf_gpio_pin_toggle(22);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t our_descriptor_add(ble_os_t * p_our_service, uint16_t char_handle)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
    ble_gatts_attr_md_t    attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;
    
    ble_uuid_t          attr_uuid;
    attr_uuid.uuid      = BLE_UUID_DESCRIPTOR_CHAR_USER_DESC;
    ble_uuid128_t       base_uuid = {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}; // 128-bit base UUID
    err_code = sd_ble_uuid_vs_add(&base_uuid, &attr_uuid.type);
    APP_ERROR_CHECK(err_code); 
    //BLE_UUID_BLE_ASSIGN(attr_uuid, BLE_UUID_DESCRIPTOR_CHAR_USER_DESC);
    
        
    //uint8_t attr_value[]  = "Your User description";
    uint8_t attr_value[]  = "Livingroom";
    const uint16_t attr_len   = sizeof(attr_value);
    
    ble_gatts_attr_t    attr;
    memset(&attr, 0, sizeof(attr));
    
    attr.init_len       = attr_len;
    attr.max_len        = attr_len;
    attr.init_offs      = 0;
    attr.p_value        = attr_value;
    attr.p_attr_md      = &attr_md;
    attr.p_uuid         = &attr_uuid;
    
    err_code = sd_ble_gatts_descriptor_add(char_handle, &attr, &p_our_service->descr_handle);
    APP_ERROR_CHECK(err_code); 
    
    return err_code;
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t * p_our_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Add characteristic UUID to the BLE stack
//    ble_uuid_t          char_uuid;    
    //BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR);
    ble_uuid_t        char_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    char_uuid.uuid = 0x2A1C;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Initiate variable holding metadata for the Client Characteristic Configuration Descriptor
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    
    // OUR_JOB: Initiate variable holding characteristic metadata
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;
    
    
    // OUR_JOB: Initiate variable holding the characteristic attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    
    // OUR_JOB: Initiate variable holding the characteristic value properties
    ble_gatts_attr_t    attr_char_value;
    uint8_t initial_value = 0xAA;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(int32_t);
    attr_char_value.max_len   = sizeof(int32_t);
    attr_char_value.p_value   = &initial_value;

    // OUR_JOB: Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_our_service->char_handles);
    APP_ERROR_CHECK(err_code);   
    
    our_descriptor_add(p_our_service, p_our_service->char_handles.value_handle);
    APP_ERROR_CHECK(err_code);   
    
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // FROM_SERVICE_TUTORIAL: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_our_service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    our_char_add(p_our_service);
}


void our_characteristic_update(ble_os_t *p_os, int32_t * characteristic_value)
{
    uint32_t err_code;
    static int32_t previous_value = 0;
    // OUR_JOB: Update characteristic values
    //int32_t new_value = *characteristic_value;
 

    if (p_os->conn_handle != BLE_CONN_HANDLE_INVALID)
    {    
        if(*characteristic_value != previous_value)
        {
            //nrf_gpio_pin_toggle(LED_1);
            uint16_t hvx_len = 4;
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_os->char_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvx_len;
            hvx_params.p_data = (uint8_t*)characteristic_value;

            err_code = sd_ble_gatts_hvx(p_os->conn_handle, &hvx_params);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_CHECK(err_code);
            }      
            previous_value = *characteristic_value;
        }
    }
		
    
    
}
