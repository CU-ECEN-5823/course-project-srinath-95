/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

#include "src/log.h"
#include "src/cmu.h"
#include "src/gpio.h"
#include "src/timer.h"
#include "src/display.h"
#include "src/i2c.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

int Button_state =0;
int resp = 0;
extern PB0_pressed;
int PB0_dummy=0;

int32_t data;
int32_t beatsPerMinute;

//#define MESH_BUTTON_CLIENT_MODEL_ID 0x1001
static uint16 _elem_index = 0x00;
static uint8 trid = 0;
static uint8 conn_handle = 0xFF;

int connections_count = 0;

//Sleep Mode
const int lowest_sleep_mode = 0;

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;


#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define FRIEND_ESTABLISH 100

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
//	gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
//	gecko_bgapi_class_sm_init();
//	mesh_native_bgapi_init();

	// gecko_initCoexHAL(); //added

	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();
}

void publish_data_to_friend()
{
	struct mesh_generic_state pulse_data;
	pulse_data.kind = mesh_generic_state_level;
	pulse_data.level.level = data;

	LOG_INFO("\n Entered the publish_data_to_friend");

	resp = mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,_elem_index,&pulse_data,0,0);
	LOG_INFO("\n Entered the publish_data_to_friend code %x", resp);
	if(resp)
	{
		LOG_INFO("\n Error in update publish_data_to_friend %x", resp);
	}
	else
	{
		LOG_INFO("Update publish_data_to_friend Success\n ");
		 resp = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
														_elem_index,
														mesh_generic_state_level);

		 LOG_INFO("\n Publish publish_data_to_friend Success %x", resp);
		 if(resp)
		 {
			 LOG_INFO("\n Error in publishing publish_data_to_friend %x", resp);
		 }
		 else
		 {
			 LOG_INFO("\n Success in publishing data to friend");
		 }
	}
}

void Button_status_send()
{
	LOG_INFO("\n Entered button status function");
	publish_data_to_friend();
}
void Heart_rate_Send_data()
{
	LOG_INFO("\n Entered button status function");
	publish_data_to_friend();
}

/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
//	gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();

}


void lpn_init(void)
{
  uint16 res;
  // Initialize LPN functionality.
  res = gecko_cmd_mesh_lpn_init()->result;
  if (res) {
    printf("\n LPN_Init failed (0x%x)", res);
    return;
  }

  res = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
  if (res) {
    printf("LPN_confg failed (0x%x)\r\n", res);
    return;
  }

  res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;
  if (res != 0) {
    printf("Friend Establish status %x\r\n", res);
  }
}

void lpn_init2(void)
{
	 /* Initialize mesh lib */
	 mesh_lib_init(malloc, free, 8);
	 lpn_init();
}


void gecko_main_init()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Initialize log
    logInit();

    // Initialize gpio
    gpioInit();


    // Initialize stack
    //gecko_init(&config);

    // Initialize CMU
    cmu_init();

    displayInit();

    timers_init();


  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);

  if( DeviceUsesClientModel() ) {
	  gecko_bgapi_classes_init_client_lpn();
  } else {
	  gecko_bgapi_classes_init_server_friend();
  }

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

}

static void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  LOG_INFO("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res)
  {
	  LOG_INFO("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }
}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	printf("\n Entered Boot id");

    	 if (GPIO_PinInGet(gpioPortF,6) == 0)
    	 {
    		 displayPrintf(DISPLAY_ROW_NAME ,"FAC RESET");
    		 LOG_INFO("\n Enter fac reset section");

    		 gecko_cmd_flash_ps_erase_all();
    		 gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
    	 }
    	 else
    	 {
    		 LOG_INFO("\n Enter non factory reset section");

    		 displayPrintf(DISPLAY_ROW_NAME ,"LPN-HR");

    		 struct gecko_msg_system_get_bt_address_rsp_t *address_t;
				address_t = gecko_cmd_system_get_bt_address();
				displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",address_t->address.addr[5],address_t->address.addr[4],address_t->address.addr[3],address_t->address.addr[2],address_t->address.addr[1],address_t->address.addr[0]);
    		 struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

    		 set_device_name(&pAddr->address);
			 gecko_cmd_mesh_node_init();
    	 }

      break;

    case gecko_evt_hardware_soft_timer_id:
          switch (evt->data.evt_hardware_soft_timer.handle) {
            case TIMER_ID_FACTORY_RESET:
              // reset the device to finish factory reset
              gecko_cmd_system_reset(0);
              break;

            case TIMER_ID_RESTART:
              // restart timer expires, reset the device
              gecko_cmd_system_reset(0);
              break;

            case FRIEND_ESTABLISH:
			 {
			   printf("Finding friend...\r\n");
			  uint16_t res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

			   if (res != 0) {
				 printf("Friend establish error code: %x\r\n", res);
			   }
			 }
			 break;

            default:
            	break;
          }
          break;

    case gecko_evt_mesh_node_initialized_id:
    	 resp = gecko_cmd_mesh_generic_server_init()->result;
		if(resp)
		{
			printf("\n Server_Init Failed %x", resp);
		}

    	LOG_INFO("\n Entered gecko_evt_mesh_node_initialized_id");
      if (!evt->data.evt_mesh_node_initialized.provisioned) {
        // The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
    	  resp = gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
    	  printf("\n Result gecko_evt_mesh_node_initialized_id: 0x%x",resp);
    	  displayPrintf(DISPLAY_ROW_CONNECTION ,"Un-Provisioned");
      }


      if(evt->data.evt_mesh_node_initialized.provisioned)
		{
    	  lpn_init2();
		}

      break;
    case gecko_evt_mesh_node_provisioning_started_id:
    	 displayPrintf(DISPLAY_ROW_CONNECTION ,"Provisioning");
    	 LOG_INFO("\n Provisioning ");

    	break;

    case gecko_evt_mesh_node_provisioned_id:
    	displayPrintf(DISPLAY_ROW_CONNECTION ,"Provisioned");
    	LOG_INFO("\n Provisioned");
    	lpn_init2();
       	break;

    case gecko_evt_mesh_node_provisioning_failed_id:
    	LOG_INFO("\n Provisioning failed");
    	displayPrintf(DISPLAY_ROW_CONNECTION ,"Provisioning Failed");
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
       	break;

    case gecko_evt_mesh_generic_server_client_request_id:

    		LOG_INFO("\n Entered Server client request id ");
    		uint8 button_stat = 0;
    		button_stat = evt->data.evt_mesh_generic_server_client_request.parameters.data[0];
    		if(button_stat == 1)
			{
    			LOG_INFO("\n Button Pressed");
				displayPrintf(DISPLAY_ROW_PASSKEY ,"Button Pressed");
			}
			else
			{
				LOG_INFO("\n Button Released");
				displayPrintf(DISPLAY_ROW_PASSKEY ,"Button Released");
			}
    		//mesh_lib_generic_server_event_handler();

    	break;

    case gecko_evt_mesh_node_reset_id:

    	gecko_cmd_flash_ps_erase_all();
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);

    	break;


    case gecko_evt_system_external_signal_id:
    	if(evt->data.evt_system_external_signal.extsignals & INTERRUPT_COMP0)
    	{
    		CORE_DECLARE_IRQ_STATE;
    		CORE_ENTER_CRITICAL();
    		INTERRUPT_COMP0 = 0;
    		CORE_EXIT_CRITICAL();

    		adc_reading(&data);

    		if(data != NULL)
    		{
    			Heart_rate_Send_data();
    		}

			beatsPerMinute = 60 /(data / 1000.0);


			LOG_INFO("\n Heart beat data per minute= %d",beatsPerMinute);
    	}

//    	if(evt->data.evt_system_external_signal.extsignals & INTERRUPT_BUTTON)
//    	{
//    		LOG_INFO("\n Entered the Interrupt button external event");
//    		Button_status_send();
//    	}
    	if(evt->data.evt_system_external_signal.extsignals & INTERRUPT_BUTTON)
    	{
    		LOG_INFO("\n Entered the on off publish section");

    		CORE_DECLARE_IRQ_STATE;
    		CORE_ENTER_CRITICAL();
    		INTERRUPT_BUTTON = false;
    		CORE_EXIT_CRITICAL();
    		PB0_dummy = 1 - PB0_pressed;
    		Button_state = PB0_dummy;

    		LOG_INFO("\n The state of the button is %d",Button_state);

			trid +=1;
			Button_status_send();
//			struct mesh_generic_request custom;
//			custom.kind = mesh_generic_request_on_off;
//			custom.on_off = Button_state;
//    		resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,_elem_index, trid, &custom, 0,0, 0);

//    		LOG_INFO("\n Result of publish: %d",resp);
    	}

	break;

    case gecko_evt_le_connection_opened_id:
          printf("evt:gecko_evt_le_connection_opened_id\r\n");
          connections_count++;
          conn_handle = evt->data.evt_le_connection_opened.connection;

          break;
    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
    	gecko_cmd_flash_ps_erase_all();
        gecko_cmd_system_reset(2);
      }
      if(connections_count >0)
      {
    	  if(--connections_count == 0)
    	  {
    		     lpn_init();
    	  }
      }


      break;

    case gecko_evt_mesh_generic_server_state_changed_id:
       mesh_lib_generic_server_event_handler(evt);
       break;

    case gecko_evt_mesh_lpn_friendship_established_id:
          LOG_INFO("\n Friendship established\n");

          break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
         LOG_INFO("friendship failed\r\n");
         if(connections_count == 0)
         {
        	 resp  = gecko_cmd_hardware_set_soft_timer(2 * 32768, FRIEND_ESTABLISH, 1)->result;
        	          if (resp) {
        	         	 LOG_INFO("\n gecko_cmd_hardware_set_soft_timer failed  %x\n", resp);
        	          }
         }

         break;


    case gecko_evt_mesh_lpn_friendship_terminated_id:
    	LOG_INFO("friendship terminated\r\n");
    	 if(connections_count == 0)
    	         {
           resp  = gecko_cmd_hardware_set_soft_timer(2 * 32768, FRIEND_ESTABLISH, 1)->result;
           if (resp) {
        	   LOG_INFO("\n gecko_cmd_hardware_set_soft_timer failed  %x\n", resp);
           }
    	         }
          break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);
        /* Close connection to enter to DFU OTA mode*/
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;


       break;

    default:
      break;
  }
}


