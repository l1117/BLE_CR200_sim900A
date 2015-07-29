/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

//#include <stdint.h>
//#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_nus.h"
//#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "simple_uart.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define WAKEUP_BUTTON_ID                0                                           /**< Button used to wake up the application. */

#define DEVICE_NAME                     "CR200_UART"                               /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                5000                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of simultaneously GPIOTE users. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define GTM900_power_pin								18
#define UART_TX_BUF_SIZE                128                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                512                                         /**< UART RX buffer size. */
#define CR200_BUF_SIZE   								1024*4
#define TIME_PERIOD 										3600       																	
#define UART_POWER  				3    //10   //5TM Power

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static uint32_t	 timer_counter = 0;
static app_timer_id_t           weakup_timer_id;                                   
static bool weakup_flag = false ;
static uint8_t cr200_data[CR200_BUF_SIZE] ;
static uint16_t cr200_data_index = 0 ;
static uint8_t cr200_stat = 1 ;
static uint8_t rx_data[256];
static uint16_t month_days[12]={0,31,31+28,31+28+31,31+28+31+30,31+28+31+30+31,
										31+28+31+30+31+30,31+28+31+30+31+30+31,31+28+31+30+31+30+31+31,
										31+28+31+30+31+30+31+31+30,31+28+31+30+31+30+31+31+30+31,
										31+28+31+30+31+30+31+31+30+31+30};
static bool send_ok = true;
//#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
//        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)


/**@brief Function for assert macro callback.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting the advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;  //
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
				if (p_data[i] == '\\' & p_data[i+1] == 'r') p_data[++i]=0x0D;
        simple_uart_put(p_data[i]);
    }
		cr200_data_index = 0;
		uint16_t index_shift = 0;
		uint16_t i = 0 , count =0;
    while (count++ < 100){
				while(simple_uart_get_with_timeout(1,&cr200_data[cr200_data_index])) cr200_data_index++;
				for (; i < cr200_data_index; i++){
						length = i + 1 - index_shift;
						if ((cr200_data[i] == '\n') || (i == (cr200_data_index-1)) || length == 20) {
								ble_nus_string_send(p_nus, cr200_data+index_shift, length);
								index_shift = i+1;
								count = 0;
						}
				}
		}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//    APP_ERROR_CHECK(err_code);
}


/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_sec_keyset_t      s_sec_keyset;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

					nrf_gpio_pin_set (UART_POWER);
		  			NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
					NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
//					simple_uart_config(NULL,16,NULL,8,false);
					simple_uart_config(NULL,0,NULL,1,false);
					NRF_UART0->BAUDRATE      = (UART_BAUDRATE_BAUDRATE_Baud9600 << UART_BAUDRATE_BAUDRATE_Pos);

            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//            APP_ERROR_CHECK(err_code);

					NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
		  			NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
					nrf_gpio_pin_clear (UART_POWER);


            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();



            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            s_sec_keyset.keys_periph.p_enc_key = NULL;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params,
                                                   &s_sec_keyset);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            // TODO: Adoptation to s110v8.0.0, is this needed anymore ?
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (s_sec_keyset.keys_periph.p_enc_key != NULL)
            {
                p_enc_info = &s_sec_keyset.keys_periph.p_enc_key->enc_info;
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device.
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            { 
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);
                // Configure buttons with sense level low as wakeup source.
                err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
                APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();    
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

bool cr200_uart(void)
{
	uint16_t err_code = 0;
	while ((cr200_data[cr200_data_index-1]!='>') ){
				simple_uart_put(0x0d);
				while(simple_uart_get_with_timeout(10,&cr200_data[cr200_data_index])) cr200_data_index++;
				if (++err_code > 100) return false;
				}
	cr200_data_index = 0 ;
	err_code = 0;
// get last record
	while (cr200_data_index < CR200_BUF_SIZE) {
				simple_uart_put('4');
				simple_uart_put(0x0d);
				while(simple_uart_get_with_timeout(10,&cr200_data[cr200_data_index])) cr200_data_index++;
				if (++err_code > 100) return false;
				}
	cr200_data_index = 0 ;
	err_code = 0;

	while (cr200_data_index < CR200_BUF_SIZE) {
				simple_uart_put('6');
				simple_uart_put(0x0d);
				while(simple_uart_get_with_timeout(10,&cr200_data[cr200_data_index])) cr200_data_index++;
				if (++err_code > 100) return false;
				}
	return true;
}

void send_string_no_answer(char * S)
{
   while(*S)
    {
        simple_uart_put(*S++);
				}
}
bool send_string(char * S,char * Respond)
{
   bool ret = false;
	uint8_t Rcount=0;  
	 while (* (Respond+Rcount)) Rcount++;
		Rcount--;
	
//		Respond-=2;
//	 char * SR; 
		
		uint8_t r=0,i=Rcount;
		while(simple_uart_get_with_timeout(1,rx_data));
		memset(rx_data,0,256);
   while(*S)
    {
        simple_uart_put(*S++);
				}

		while(simple_uart_get_with_timeout(10000,rx_data+i)){
//				i=10;
//				SR=Respond;rx_data[8]='4';rx_data[9]='O';rx_data[10]='K';
				if (i>3 && rx_data[i-4]=='E' && rx_data[i-3]=='R' && rx_data[i-2]=='R' && rx_data[i-1]=='O' && rx_data[i]=='R')	break ;
				for (r=0; r<Rcount; r++) 
						if (*(Respond+r) != (rx_data[i-Rcount+r])) break ;
				if (r==Rcount) break;
//				if (rx_data[i-3]=='O' && rx_data[i-2]=='K' && rx_data[i-1]==0x0D && rx_data[i]==0x0A) {ret=true; break ;}
//				if (rx_data[i-3]=='C' && rx_data[i-2]=='O' && rx_data[i-1]=='N' && rx_data[i]=='N') {ret=true; break ;}
				i++;
				}
		if (r==Rcount) {
				ret=true; 
				while(simple_uart_get_with_timeout(1,rx_data+i++));
				}
		else ret=false;
		return ret;
}

 char char_hex( char bHex){
		bHex&=0x0f;
    if((bHex<10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}
void battery_start(uint32_t AnalogInput)
{
    uint32_t err_code=0;
//		AnalogInput=ADC_CONFIG_PSEL_AnalogInput4 ;
    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (AnalogInput                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

    // Enable ADC interrupt
//    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
//    APP_ERROR_CHECK(err_code);

//    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
//    APP_ERROR_CHECK(err_code);

//    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
//    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;


				uint8_t adc_delay=0;
        while (!NRF_ADC->EVENTS_END && adc_delay<0xff)  adc_delay++;
//        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(NRF_ADC->RESULT); // + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        NRF_ADC->TASKS_STOP     = 1;
				NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Disabled;
}

void gprs_gtm900()
{
//GTM900C 
//		simple_uart_config(NULL, 8, NULL, 9, false);
			send_string("AT\r\n","OK");
			send_string("AT\r\n","OK");
			send_string("ATE1\r\n","OK");
			if(!send_string("AT+CPIN?\r\n","READY")) return; //error

			send_string("AT+CLTS=1\r\n","OK");  //Get date time.
			if ((send_string("","DST:")||send_string("","DST:"))	
						&& send_string("AT+CCLK?\r\n","OK") && (rx_data[21]=='1' && rx_data[22]>='5')){
					rx_data[24] = (rx_data[24]-'0')*10 + (rx_data[25]-'0');  	//month
					rx_data[27] = (rx_data[27]-'0')*10 + (rx_data[28]-'0');		//day
					rx_data[30] = (rx_data[30]-'0')*10 + (rx_data[31]-'0');		//hour
					rx_data[33] = (rx_data[33]-'0')*10 + (rx_data[34]-'0');		//minus
					rx_data[36] = (rx_data[36]-'0')*10 + (rx_data[37]-'0');		//second
//					timer_counter = ((((month_days[rx_data[24]-1]+rx_data[27])*24 + rx_data[30])*60 + rx_data[33])*60 + rx_data[36]);
					}
			send_string("AT+CLTS=0\r\n","OK");  //Get date time.
			send_string("ATE0\r\n","OK");


			send_string("at+csq\r\n","OK");  	//ÐÅºÅÖÊÁ¿
//			send_string("at%ipclose=1\r\n","OK");
			if (!send_string("at+cgatt=1\r\n","OK")) return;
			send_string("AT+CIPCSGP=1,\"cmnet\",\"guest\",\"guest\"\r\n","OK");

			send_string("at+CSTT\r\n","OK");
			send_string("AT+CIICR\r\n","OK");

			send_string("AT+CIFSR\r\n",".");
			send_string("AT+CIPCLOSE=1\r\n","OK");

			if(!(send_string("AT+CIPSTART=\"TCP\",\"159.226.251.11\",25\r\n","cstnet")||send_string("","cstnet"))) return;

//send mail
			send_string("AT+CIPSPRT=1\r\n","OK");
			send_string("AT+CIPSEND\r\n",">");			
			send_string("ehlo helo\r\n\x1a","coremail");			//ehlo helo
			send_string("AT+CIPSEND\r\n",">");		
			send_string("auth login\r\n\x1a","SEND");		//auth login
			send_string("AT+CIPSEND\r\n",">");		
			send_string("Y2NkYw==\r\n\x1a","SEND");				//user name: ccdc
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("Y2NkY2NzdG5ldA==\r\n\x1a","successful")) 
							return;//passwore: ********

			send_string("AT+CIPSEND\r\n",">");		
			send_string("MAIL FROM:<lye@cstnet.cn>\r\n\x1a","SEND");//mail from lye@cstnet.cn
			send_string("AT+CIPSEND\r\n",">");		
			send_string("RCPT to:<ccdc@cstnet.cn>\r\n\x1a","SEND");//RCPT to:<ccdc@cstnet.cn>
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("DATA\r\n\x1a","SEND")) return;//DATA

			send_string("CR200_UART:AT+CREG=2\r\n","OK");


			send_string("AT+CREG?\r\n","OK");

			char s,subject[40];
			for(s=3;s<27;s++) subject[s]=rx_data[s];
			subject[27]='C';
			subject[28]='R'; //char_hex(pstorage_block_id>>12);
			subject[29]='2';  //char_hex((pstorage_block_id>>8));
			subject[30]='0';  //char_hex((pstorage_block_id>>4));
			subject[31]='0';  //char_hex(pstorage_block_id);
			battery_start(ADC_CONFIG_PSEL_AnalogInput5);
			uint16_t batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
			subject[32]=';';
			subject[33]=char_hex(batt_lvl_in_milli_volts>>12);
			subject[34]=char_hex((batt_lvl_in_milli_volts>>8));
			subject[35]=char_hex((batt_lvl_in_milli_volts>>4));
			subject[36]=char_hex(batt_lvl_in_milli_volts);

			send_string("AT+CIPSEND\r\n",">");		


			send_string_no_answer("Subject: ");//Subject: 

			for (uint8_t i=3;i<37;i++) {
					simple_uart_put((subject[i]));//+CREG: 2,1,"11D4","2048"
					}

			send_string("\r\n\r\n\x1a","OK");
			send_string("AT+CREG=0\r\n","OK");
			send_string("AT+CIPSEND\r\n",">");
		for (uint16_t i=0; i<CR200_BUF_SIZE;i++){
				simple_uart_put(cr200_data[i]);
				if (cr200_data[i-2] == '6' && cr200_data[i-1] == '\r' && cr200_data[i] == '\n'){
						send_string("\x1a","OK");	
						if (!send_string("AT+CIPSEND\r\n",">")) if (!send_string("AT+CIPSEND\r\n",">")) return;	
						}
				}
			if (send_string("\r\n\x1a","OK") &&
					send_string("AT+CIPSEND\r\n",">") &&
					send_string("\r\n\x2e\r\n\x1a","OK")) {
					send_string("AT+CIPCLOSE=1\r\n","OK");
					}
			else return;
			send_ok = true;
}


/**@brief  Application main function.
 */
static void weakup_timeout_handler(void * p_context)
{		
    UNUSED_PARAMETER(p_context);
		uint32_t err_code;
//		send_ok = false;
		if (++timer_counter > 0) { weakup_flag = true ; timer_counter = 0;}  
		else if (!send_ok ) weakup_flag = true;
}

int main(void)
{

//    uint8_t start_string[] = START_STRING;
    uint32_t err_code;
    
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    ble_stack_init();
		err_code = app_timer_create(&weakup_timer_id,
																APP_TIMER_MODE_REPEATED,
																weakup_timeout_handler);
		APP_ERROR_CHECK(err_code);
//    uart_init();
//		NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);



//    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
//                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
//                        NULL);
//    APP_ERROR_CHECK(err_code);
//    err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
//    APP_ERROR_CHECK(err_code);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
//    sec_params_init();
    
//    printf("%s",start_string);

		NRF_GPIO->PIN_CNF[UART_POWER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
		NRF_GPIO->PIN_CNF[GTM900_power_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
																							| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
																							| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
																							| (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
																							| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
		nrf_gpio_pin_clear(GTM900_power_pin);

    advertising_start();
    cr200_data[0]=0;
		cr200_stat = 0;
		err_code = app_timer_start(weakup_timer_id,  APP_TIMER_TICKS(TIME_PERIOD*1000, 0), NULL) ;
		APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
			if (weakup_flag){
					nrf_gpio_pin_set (UART_POWER);
		  			NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
					NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
//					simple_uart_config(NULL,16,NULL,8,false);
					simple_uart_config(NULL,0,NULL,1,false);
					NRF_UART0->BAUDRATE      = (UART_BAUDRATE_BAUDRATE_Baud9600 << UART_BAUDRATE_BAUDRATE_Pos);

					err_code=cr200_uart();

					NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
		  			NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
					nrf_gpio_pin_clear (UART_POWER);
		
					if (err_code ) {
						nrf_gpio_pin_clear(GTM900_power_pin);
						nrf_delay_ms(100);
						nrf_gpio_pin_set(GTM900_power_pin);
		  			NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
		  			NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);

						simple_uart_config(NULL, 24, NULL, 23, false);  // for GPS box PCB
						NRF_UART0->BAUDRATE      = (UART_BAUDRATE_BAUDRATE_Baud38400 << UART_BAUDRATE_BAUDRATE_Pos);

						gprs_gtm900();
						if (send_string("\r\n\x1a","OK")  &&
								send_string("AT+CIPSEND\r\n",">") &&
								send_string("\r\n\x2e\r\n\x1a","OK") &&
								send_string("AT+CIPCLOSE=1\r\n","OK")) {};
						send_string("AT+CIPSHUT\r\n","OK");
						send_string("AT+CGATT=0\r\n","OK");
						send_string("AT+CPOWD=1\r\n","POWER");
					NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);

						NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
						nrf_gpio_pin_clear(GTM900_power_pin);
						}
						weakup_flag = false;
				}
        power_manage();

    }
}

