/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : App/app_thread.c
  * Description        : Thread Application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "utilities_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_thread.h"
#include "stm32wbxx_core_interface_def.h"
#include "openthread_api_wb.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32_lpm.h"
#include "cmsis_os.h"
#if (CFG_USB_INTERFACE_ENABLE != 0)
#include "vcp.h"
#include "vcp_conf.h"
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_hal.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define C_SIZE_CMD_STRING       256U
#define C_PANID                 0x1812U
#define C_CHANNEL_NB            14U

/* FreeRtos stacks attributes */
const osThreadAttr_t ThreadMsgM0ToM4Process_attr = {
    .name = CFG_THREAD_MSG_M0_TO_M4_PROCESS_NAME,
    .attr_bits = CFG_THREAD_MSG_M0_TO_M4_PROCESS_ATTR_BITS,
    .cb_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_MEM,
    .cb_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_SIZE,
    .stack_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_MEM,
    .priority = CFG_THREAD_MSG_M0_TO_M4_PROCESS_PRIORITY,
    .stack_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACk_SIZE
};

const osThreadAttr_t ThreadCliProcess_attr = {
     .name = CFG_THREAD_CLI_PROCESS_NAME,
     .attr_bits = CFG_THREAD_CLI_PROCESS_ATTR_BITS,
     .cb_mem = CFG_THREAD_CLI_PROCESS_CB_MEM,
     .cb_size = CFG_THREAD_CLI_PROCESS_CB_SIZE,
     .stack_mem = CFG_THREAD_CLI_PROCESS_STACK_MEM,
     .priority = CFG_THREAD_CLI_PROCESS_PRIORITY,
     .stack_size = CFG_THREAD_CLI_PROCESS_STACk_SIZE
 };

/* USER CODE BEGIN PD */
#define C_RESOURCE_PH       "ph"
#define C_RESOURCE_EC       "ec"
#define C_RESOURCE_TEMP     "temp"


/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
static void APP_THREAD_CheckWirelessFirmwareInfo(void);
static void APP_THREAD_DeviceConfig(void);
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext);
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode);
static void Send_CLI_To_M0(void);
static void Send_CLI_Ack_For_OT(void);
static void HostTxCb( void );
static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);
#if (CFG_HW_LPUART1_ENABLED == 1)
extern void MX_LPUART1_UART_Init(void);
#endif
#if (CFG_HW_USART1_ENABLED == 1)
extern void MX_USART1_UART_Init(void);
#endif
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint32_t ProcessCmdString(uint8_t* buf , uint32_t len);
#else
static void RxCpltCallback(void);
#endif

/* FreeRTos wrapper functions */
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument);
static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument);

/* USER CODE BEGIN PFP */
/* Dummy request/response handlers */
static void APP_THREAD_DummyReqHandler(void     * p_context,
                            otCoapHeader        * pHeader,
                            otMessage           * pMessage,
                            const otMessageInfo * pMessageInfo); 
static void APP_THREAD_DummyRespHandler(void    * p_context,
                            otCoapHeader        * pHeader,
                            otMessage           * pMessage,
                            const otMessageInfo * pMessageInfo,
                            otError             Result);

/* pH resource request handler */ 
static void APP_THREAD_ph_ReqHandler(otCoapHeader   * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo);
static otError APP_THREAD_ph_RespSend(otCoapHeader  * pRequestHeader,
                            const otMessageInfo     * pMessageInfo);

/* EC resource request handler */ 
static void APP_THREAD_ec_ReqHandler(otCoapHeader   * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo);
static otError APP_THREAD_ec_RespSend(otCoapHeader  * pRequestHeader,
                            const otMessageInfo     * pMessageInfo);

/* Temperature resource request handler */ 
static void APP_THREAD_temp_ReqHandler(otCoapHeader * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo);
static otError APP_THREAD_temp_RespSend(otCoapHeader * pRequestHeader,
                            const otMessageInfo      * pMessageInfo);

otError APP_THREAD_MethodNotAllowed_RespSend(otCoapHeader   * pHeader,
                            const otMessageInfo             * pMessageInfo);
/* USER CODE END PFP */

/* Private variables -----------------------------------------------*/
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint8_t TmpString[C_SIZE_CMD_STRING];
static uint8_t VcpRxBuffer[sizeof(TL_CmdSerial_t)];        /* Received Data over USB are stored in this buffer */
static uint8_t VcpTxBuffer[sizeof(TL_EvtPacket_t) + 254U]; /* Transmit buffer over USB */
#else
static uint8_t aRxBuffer[C_SIZE_CMD_STRING];
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

static uint8_t CommandString[C_SIZE_CMD_STRING];
static __IO uint16_t indexReceiveChar = 0;
static __IO uint16_t CptReceiveCmdFromUser = 0;

static TL_CmdPacket_t *p_thread_otcmdbuffer;
static TL_EvtPacket_t *p_thread_notif_M0_to_M4;
static __IO uint32_t  CptReceiveMsgFromM0 = 0;
static volatile int FlagReceiveAckFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_TH_Config_t ThreadConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ThreadNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliCmdBuffer;

static osThreadId_t OsTaskMsgM0ToM4Id;      /* Task managing the M0 to M4 messaging        */
static osThreadId_t OsTaskCliId;            /* Task used to manage CLI comamnd             */

/* USER CODE BEGIN PV */
static otCoapResource OT_Resource_ph = {C_RESOURCE_PH, 
                            APP_THREAD_DummyReqHandler,
                            (void*) APP_THREAD_ph_ReqHandler,
                            NULL};
static otCoapResource OT_Resource_ec = {C_RESOURCE_EC,
                            APP_THREAD_DummyReqHandler,
                            (void*) APP_THREAD_ec_ReqHandler,
                            NULL};
static otCoapResource OT_Resource_temp = {C_RESOURCE_TEMP,
                            APP_THREAD_DummyReqHandler,
                            (void*) APP_THREAD_temp_ReqHandler,
                            NULL};

static otCoapHeader  OT_Header = {0};
static otMessage   * pOT_Message = NULL;

/* USER CODE END PV */

/* Functions Definition ------------------------------------------------------*/

void APP_THREAD_Init( void )
{
  /* USER CODE BEGIN APP_THREAD_INIT_1 */

  /* USER CODE END APP_THREAD_INIT_1 */

  SHCI_CmdStatus_t ThreadInitStatus;
  
  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_THREAD_CheckWirelessFirmwareInfo();

#if (CFG_USB_INTERFACE_ENABLE != 0)
  VCP_Init(&VcpTxBuffer[0], &VcpRxBuffer[0]);
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

  /* Register cmdbuffer */
  APP_THREAD_RegisterCmdBuffer(&ThreadOtCmdBuffer);

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_DISABLE);

  /* Init config buffer and call TL_THREAD_Init */
  APP_THREAD_TL_THREAD_INIT();

  /* Configure UART for sending CLI command from M4 */
  APP_THREAD_Init_UART_CLI();

  /* Send Thread start system cmd to M0 */
  ThreadInitStatus = SHCI_C2_THREAD_Init();
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ThreadInitStatus);

  /* USER CODE BEGIN APP_THREAD_INIT_TIMER */

  /* USER CODE END APP_THREAD_INIT_TIMER */

  /* Create the different FreeRTOS tasks requested to run this Thread application*/
  OsTaskMsgM0ToM4Id = osThreadNew(APP_THREAD_FreeRTOSProcessMsgM0ToM4Task, NULL,&ThreadMsgM0ToM4Process_attr);

  /* USER CODE BEGIN APP_THREAD_INIT_FREERTOS */

  /* USER CODE END APP_THREAD_INIT_FREERTOS */

  /* Configure the Thread device at start */
  APP_THREAD_DeviceConfig();

 /* USER CODE BEGIN APP_THREAD_INIT_2 */

 /* USER CODE END APP_THREAD_INIT_2 */
}

/**
  * @brief  Trace the error or the warning reported.
  * @param  ErrId :
  * @param  ErrCode
  * @retval None
  */
void APP_THREAD_Error(uint32_t ErrId, uint32_t ErrCode)
{
  /* USER CODE BEGIN APP_THREAD_Error_1 */

  /* USER CODE END APP_THREAD_Error_1 */
  switch(ErrId)
  {
  case ERR_REC_MULTI_MSG_FROM_M0 :
    APP_THREAD_TraceError("ERROR : ERR_REC_MULTI_MSG_FROM_M0 ", ErrCode);
    break;
  case ERR_THREAD_SET_STATE_CB :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_STATE_CB ",ErrCode);
    break;
   case ERR_THREAD_SET_CHANNEL :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_CHANNEL ",ErrCode);
    break;
  case ERR_THREAD_SET_PANID :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_PANID ",ErrCode);
    break;
  case ERR_THREAD_IPV6_ENABLE :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_IPV6_ENABLE ",ErrCode);
    break;
  case ERR_THREAD_START :
    APP_THREAD_TraceError("ERROR: ERR_THREAD_START ", ErrCode);
    break;
  case ERR_THREAD_ERASE_PERSISTENT_INFO :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_ERASE_PERSISTENT_INFO ",ErrCode);
    break;
  case ERR_THREAD_CHECK_WIRELESS :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_CHECK_WIRELESS ",ErrCode);
    break;
  /* USER CODE BEGIN APP_THREAD_Error_2 */
  case ERR_NEW_MSG_ALLOC :
    APP_THREAD_TraceError("ERROR : ERR_NEW_MSG_ALLOC ",ErrCode);
    break;
  case ERR_THREAD_COAP_START :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_COAP_START ",ErrCode);
    break;
  case ERR_THREAD_COAP_ADD_RESSOURCE :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_COAP_ADD_RESSOURCE ",ErrCode);
    break;
  case ERR_THREAD_MESSAGE_READ :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_MESSAGE_READ ",ErrCode);
    break;
  case ERR_THREAD_COAP_SEND_RESPONSE :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_COAP_SEND_RESPONSE ",ErrCode);
    break;
  case ERR_THREAD_COAP_APPEND :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_COAP_APPEND ",ErrCode);
    break;
  case ERR_THREAD_COAP_SEND_REQUEST :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_COAP_SEND_REQUEST ",ErrCode);
    break;
  case ERR_TIMER_INIT :
    APP_THREAD_TraceError("ERROR : ERR_TIMER_INIT ",ErrCode);
    break;
  case ERR_TIMER_START :
    APP_THREAD_TraceError("ERROR : ERR_TIMER_START ",ErrCode);
    break;
  /* USER CODE END APP_THREAD_Error_2 */
  default :
    APP_THREAD_TraceError("ERROR Unknown ", 0);
    break;
    }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief Thread initialization.
 * @param  None
 * @retval None
 */
static void APP_THREAD_DeviceConfig(void)
{
  otError error;
  error = otInstanceErasePersistentInfo(NULL);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_ERASE_PERSISTENT_INFO,error);
  }
  otInstanceFinalize(NULL);
  otInstanceInitSingle();
  error = otSetStateChangedCallback(NULL, APP_THREAD_StateNotif, NULL);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_STATE_CB,error);
  }
  error = otLinkSetChannel(NULL, C_CHANNEL_NB);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_CHANNEL,error);
  }
  error = otLinkSetPanId(NULL, C_PANID);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_PANID,error);
  }
  error = otIp6SetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_IPV6_ENABLE,error);
  }
  error = otThreadSetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_START,error);
  }

  /* USER CODE BEGIN DEVICECONFIG */
  /* Start the COAP server */
  error = otCoapStart(NULL, OT_DEFAULT_COAP_PORT);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_COAP_START,error);
  }
  /* Add pH CoAP resource */
  error = otCoapAddResource(NULL, &OT_Resource_ph);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_COAP_ADD_RESSOURCE,error);
  }
  /* Add EC CoAP resource */
  error = otCoapAddResource(NULL, &OT_Resource_ec);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_COAP_ADD_RESSOURCE,error);
  }
  /* Add Temperature CoAP resource */
  error = otCoapAddResource(NULL, &OT_Resource_temp);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_COAP_ADD_RESSOURCE,error);
  }
  /* USER CODE END DEVICECONFIG */
}

/**
 * @brief Thread notification when the state changes.
 * @param  aFlags  : Define the item that has been modified
 *         aContext: Context
 *
 * @retval None
 */
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pContext);

  /* USER CODE BEGIN APP_THREAD_STATENOTIF */

  /* USER CODE END APP_THREAD_STATENOTIF */

  if ((NotifFlags & (uint32_t)OT_CHANGED_THREAD_ROLE) == (uint32_t)OT_CHANGED_THREAD_ROLE)
  {
    switch (otThreadGetDeviceRole(NULL))
    {
    case OT_DEVICE_ROLE_DISABLED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DISABLED */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      /* USER CODE END OT_DEVICE_ROLE_DISABLED */
      break;
    case OT_DEVICE_ROLE_DETACHED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DETACHED */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      /* USER CODE END OT_DEVICE_ROLE_DETACHED */
      break;
    case OT_DEVICE_ROLE_CHILD:
      /* USER CODE BEGIN OT_DEVICE_ROLE_CHILD */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      /* USER CODE END OT_DEVICE_ROLE_CHILD */
      break;
    case OT_DEVICE_ROLE_ROUTER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_ROUTER */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      /* USER CODE END OT_DEVICE_ROLE_ROUTER */
      break;
    case OT_DEVICE_ROLE_LEADER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_LEADER */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      /* USER CODE END OT_DEVICE_ROLE_LEADER */
      break;
    default:
      /* USER CODE BEGIN DEFAULT */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      /* USER CODE END DEFAULT */
      break;
    }
  }
}

/**
  * @brief  Warn the user that an error has occurred.In this case,
  *         the LEDs on the Board will start blinking.
  *
  * @param  pMess  : Message associated to the error.
  * @param  ErrCode: Error code associated to the module (OpenThread or other module if any)
  * @retval None
  */
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode)
{
  /* USER CODE BEGIN TRACE_ERROR */
  APP_DBG("**** Fatal error = %s (Err = %d)", pMess, ErrCode);
  for(;;)
  {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      HAL_Delay(250);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(250);
      HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
      HAL_Delay(250);
  }
  /* USER CODE END TRACE_ERROR */
}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Thread
 *        and display associated informations
 * @param  None
 * @retval None
 */
static void APP_THREAD_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t* p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch(p_wireless_info->StackType)
    {
    case INFO_STACK_TYPE_THREAD_FTD :
      APP_DBG("FW Type : Thread FTD");
      break;
    case INFO_STACK_TYPE_THREAD_MTD :
      APP_DBG("FW Type : Thread MTD");
      break;
    case INFO_STACK_TYPE_BLE_THREAD_FTD_STATIC :
      APP_DBG("FW Type : Static Concurrent Mode BLE/Thread");
      break;
    default :
      /* No Thread device supported ! */
      APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
      break;
    }
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * FREERTOS WRAPPER FUNCTIONS
 *
*************************************************************/
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */

    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */
    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
    APP_THREAD_ProcessMsgM0ToM4();
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */

    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */
  }
}

static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */
    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
    Send_CLI_To_M0();
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */
  }
}

/* USER CODE BEGIN FREERTOS_WRAPPER_FUNCTIONS */

/* USER CODE END FREERTOS_WRAPPER_FUNCTIONS */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
/**
 * @brief This function is used to handle the pH requests
 *
 * @param pHeader header pointer
 * @param pMessage message pointer
 * @param pMessageInfo message info pointer
 * @retval None
 */
static void APP_THREAD_ph_ReqHandler(otCoapHeader   * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo)
{
    do
    {
        if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET)
        {
            if (APP_THREAD_ph_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
        else
        {
            if (APP_THREAD_MethodNotAllowed_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
    } 
    while (false);
}

/**
 * @brief This function is used to send the pH response
 *
 * @param pHeader header pointer
 * @param pMessage message pointer
 * @param pMessageInfo message info pointer
 * @retval None
 */
static otError APP_THREAD_ph_RespSend(otCoapHeader  * pRequestHeader,
                            const otMessageInfo     * pMessageInfo)
{
    otError error = OT_ERROR_NONE;
    do
    {
        otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
        otCoapHeaderSetToken(&OT_Header,
                            otCoapHeaderGetToken(pRequestHeader),
                            otCoapHeaderGetTokenLength(pRequestHeader));
        otCoapHeaderAppendContentFormatOption(&OT_Header,
                            OT_COAP_OPTION_CONTENT_FORMAT_TEXT_PLAIN);
        otCoapHeaderSetPayloadMarker(&OT_Header);

        pOT_Message = otCoapNewMessage(NULL, &OT_Header);
        if (pOT_Message == NULL)
        {
            APP_THREAD_Error(ERR_NEW_MSG_ALLOC,error);
        }

        char * str_ptr = ADC_ChannelGet_str(1);
        error = otMessageAppend(pOT_Message, (void *) str_ptr, sizeof(char)*strlen(str_ptr));
        if (error != OT_ERROR_NONE)
        {
            APP_THREAD_Error(ERR_THREAD_COAP_APPEND,error);
            break;
        }

        error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
        if (error != OT_ERROR_NONE && pOT_Message != NULL)
        {
            otMessageFree(pOT_Message);
            APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE,error);
        }


    } while (false);

    return error;
}

/* EC resource request handler */ 
static void APP_THREAD_ec_ReqHandler(otCoapHeader   * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo)
{
    do
    {
        APP_DBG(" ********* APP_THREAD_ec_ReqHandler \r\n");
        if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET)
        {
            if (APP_THREAD_ec_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
        else
        {
            if (APP_THREAD_MethodNotAllowed_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
    } 
    while (false);
}

static otError APP_THREAD_ec_RespSend(otCoapHeader  * pRequestHeader,
                            const otMessageInfo     * pMessageInfo)
{
    otError error = OT_ERROR_NONE;

    do
    {
        otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
        otCoapHeaderSetToken(&OT_Header,
                            otCoapHeaderGetToken(pRequestHeader),
                            otCoapHeaderGetTokenLength(pRequestHeader));
        otCoapHeaderAppendContentFormatOption(&OT_Header,
                            OT_COAP_OPTION_CONTENT_FORMAT_TEXT_PLAIN);
        otCoapHeaderSetPayloadMarker(&OT_Header);

        pOT_Message = otCoapNewMessage(NULL, &OT_Header);
        if (pOT_Message == NULL)
        {
            APP_THREAD_Error(ERR_NEW_MSG_ALLOC,error);
        }

        char * str_ptr = ADC_ChannelGet_str(2);
        error = otMessageAppend(pOT_Message, (void *) str_ptr, sizeof(char)*strlen(str_ptr));
        if (error != OT_ERROR_NONE)
        {
            APP_THREAD_Error(ERR_THREAD_COAP_APPEND,error);
            break;
        }

        error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
        if (error != OT_ERROR_NONE && pOT_Message != NULL)
        {
            otMessageFree(pOT_Message);
            APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE,error);
        }


    } while (false);

    return error;
}

/* Temperature resource request handler */ 
static void APP_THREAD_temp_ReqHandler(otCoapHeader * pHeader,
                            otMessage               * pMessage,
                            const otMessageInfo     * pMessageInfo)
{
    do
    {
        APP_DBG(" ********* APP_THREAD_temp_ReqHandler \r\n");
        if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET)
        {
            if (APP_THREAD_temp_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
        else
        {
            if (APP_THREAD_MethodNotAllowed_RespSend(pHeader, pMessageInfo) != OT_ERROR_NONE)
            {
                APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE, 0);
            }
        }
    } 
    while (false);
}

static otError APP_THREAD_temp_RespSend(otCoapHeader * pRequestHeader,
                            const otMessageInfo      * pMessageInfo)
{
    otError error = OT_ERROR_NONE;

    do
    {
        otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
        otCoapHeaderSetToken(&OT_Header,
                            otCoapHeaderGetToken(pRequestHeader),
                            otCoapHeaderGetTokenLength(pRequestHeader));
        otCoapHeaderAppendContentFormatOption(&OT_Header,
                            OT_COAP_OPTION_CONTENT_FORMAT_TEXT_PLAIN);
        otCoapHeaderSetPayloadMarker(&OT_Header);

        pOT_Message = otCoapNewMessage(NULL, &OT_Header);
        if (pOT_Message == NULL)
        {
            APP_THREAD_Error(ERR_NEW_MSG_ALLOC,error);
        }

        char * str_ptr = ADC_ChannelGet_str(3);
        error = otMessageAppend(pOT_Message, (void *) str_ptr, sizeof(char)*strlen(str_ptr));
        if (error != OT_ERROR_NONE)
        {
            APP_THREAD_Error(ERR_THREAD_COAP_APPEND,error);
            break;
        }

        error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
        if (error != OT_ERROR_NONE && pOT_Message != NULL)
        {
            otMessageFree(pOT_Message);
            APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE,error);
        }


    } while (false);

    return error;

}


/**
 * @brief This function is used to send MethodNotAllowed response
 *
 * @param pHeader header pointer
 * @param pMessage message pointer
 * @param pMessageInfo message info pointer
 * @retval None
 */
otError APP_THREAD_MethodNotAllowed_RespSend(otCoapHeader  * pRequestHeader,
                            const otMessageInfo     * pMessageInfo)
{
    otError error = OT_ERROR_NONE;

    do
    {
        otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT,
                            OT_COAP_CODE_METHOD_NOT_ALLOWED);
        otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
        otCoapHeaderSetToken(&OT_Header,
                            otCoapHeaderGetToken(pRequestHeader),
                            otCoapHeaderGetTokenLength(pRequestHeader));

        pOT_Message = otCoapNewMessage(NULL, &OT_Header);
        if (pOT_Message == NULL)
        {
            APP_THREAD_Error(ERR_NEW_MSG_ALLOC,error);
        }

        error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
        if (error != OT_ERROR_NONE && pOT_Message != NULL)
        {
            otMessageFree(pOT_Message);
            APP_THREAD_Error(ERR_THREAD_COAP_SEND_RESPONSE,error);
        }


    } while (false);

    return error;
}
/**
  * @brief Dummy request handler
  * @param
  * @retval None
  */
static void APP_THREAD_DummyReqHandler(void    * p_context,
                           otCoapHeader        * pHeader,
                           otMessage           * pMessage,
                           const otMessageInfo * pMessageInfo)
{
}

/**
 * @brief This function is used to handle a dummy response handler
 *
 * @param p_context  context
 * @param pHeader  coap header
 * @param pMessage message
 * @paramp pMessageInfo otMessage information
 * @param Result error status
 * @retval None
 */
static void APP_THREAD_DummyRespHandler(void    * p_context,
                            otCoapHeader        * pHeader,
                            otMessage           * pMessage,
                            const otMessageInfo * pMessageInfo,
                            otError             Result)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(p_context);
  UNUSED(pHeader);
  UNUSED(pMessage);
  UNUSED(pMessageInfo);
  UNUSED(Result);
}
/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_THREAD_RegisterCmdBuffer(TL_CmdPacket_t* p_buffer)
{
  p_thread_otcmdbuffer = p_buffer;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)p_thread_otcmdbuffer->cmdserial.cmd.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdRspPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)((TL_EvtPacket_t *)p_thread_otcmdbuffer)->evtserial.evt.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_NotificationPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)(p_thread_notif_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the Ot commands from the
 *         M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void Ot_Cmd_Transfer(void)
{
  /* OpenThread OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_thread_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  uint32_t l_size = ((Thread_OT_Cmd_Request_t*)(p_thread_otcmdbuffer->cmdserial.cmd.payload))->Size * 4U + 8U;
  p_thread_otcmdbuffer->cmdserial.cmd.plen = l_size;

  TL_OT_SendCmd();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();
}

/**
 * @brief  This function is called when acknowledge from OT command is received from the M0+.
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_OT_CmdEvtReceived( TL_EvtPacket_t * Otbuffer )
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_NotReceived( TL_EvtPacket_t * Notbuffer )
{
  p_thread_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
  * @brief  This function is called before sending any ot command to the M0
  *         core. The purpose of this function is to be able to check if
  *         there are no notifications coming from the M0 core which are
  *         pending before sending a new ot command.
  * @param  None
  * @retval None
  */
void Pre_OtCmdProcessing(void)
{

}

/**
  * @brief  This function waits for getting an acknowledgment from the M0.
  *
  * @param  None
  * @retval None
  */
static void Wait_Getting_Ack_From_M0(void)
{
    while (FlagReceiveAckFromM0 == 0)
   {
   }
   FlagReceiveAckFromM0 = 0;
}

/**
  * @brief  Receive an acknowledgment from the M0+ core.
  *         Each command send by the M4 to the M0 are acknowledged.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Ack_From_M0(void)
{
    FlagReceiveAckFromM0 = 1;
}

/**
  * @brief  Receive a notification from the M0+ through the IPCC.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Notification_From_M0(void)
{
  CptReceiveMsgFromM0++;
  osThreadFlagsSet(OsTaskMsgM0ToM4Id,1);
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
#else
static void RxCpltCallback(void)
{
  /* Filling buffer and wait for '\r' char */
  if (indexReceiveChar < C_SIZE_CMD_STRING)
  {
    CommandString[indexReceiveChar++] = aRxBuffer[0];
    if (aRxBuffer[0] == '\r')
    {
      CptReceiveCmdFromUser = 1U;

      /* UART task scheduling*/
      osThreadFlagsSet(OsTaskCliId,1);
    }
  }

  /* Once a character has been sent, put back the device in reception mode */
  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1U, RxCpltCallback);
}
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief Process the command strings.
 *        As soon as a complete command string has been received, the task
 *        in charge of sending the command to the M0 is scheduled
 * @param  None
 * @retval None
 */
static uint32_t  ProcessCmdString( uint8_t* buf , uint32_t len )
{
  uint32_t i,j,tmp_start;
  tmp_start = 0;
  uint32_t res = 0;

  i= 0;
  while ((buf[i] != '\r') && (i < len))
  {
    i++;
  }

  if (i != len)
  {
    memcpy(CommandString, buf,(i+1));
    indexReceiveChar = i + 1U; /* Length of the buffer containing the command string */
    osThreadFlagsSet(OsTaskCliId,1)
    tmp_start = i;
    for (j = 0; j < (len - tmp_start - 1U) ; j++)
    {
      buf[j] = buf[tmp_start + j + 1U];
    }
    res = len - tmp_start - 1U;
  }
  else
  {
    res = len;
  }
  return res; /* Remaining characters in the temporary buffer */
}
#endif/* (CFG_USB_INTERFACE_ENABLE != 0) */

/**
 * @brief Process sends receive CLI command to M0.
 * @param  None
 * @retval None
 */
static void Send_CLI_To_M0(void)
{
  memset(ThreadCliCmdBuffer.cmdserial.cmd.payload, 0x0U, 255U);
  memcpy(ThreadCliCmdBuffer.cmdserial.cmd.payload, CommandString, indexReceiveChar);
  ThreadCliCmdBuffer.cmdserial.cmd.plen = indexReceiveChar;
  ThreadCliCmdBuffer.cmdserial.cmd.cmdcode = 0x0;

  /* Clear receive buffer, character counter and command complete */
  CptReceiveCmdFromUser = 0;
  indexReceiveChar = 0;
  memset(CommandString, 0, C_SIZE_CMD_STRING);

  TL_CLI_SendCmd();
}

/**
 * @brief Send notification for CLI TL Channel.
 * @param  None
 * @retval None
 */
static void Send_CLI_Ack_For_OT(void)
{

  /* Notify M0 that characters have been sent to UART */
  TL_THREAD_CliSendAck();
}

/**
 * @brief Perform initialization of CLI UART interface.
 * @param  None
 * @retval None
 */
void APP_THREAD_Init_UART_CLI(void)
{
  OsTaskCliId = osThreadNew(APP_THREAD_FreeRTOSSendCLIToM0Task, NULL,&ThreadCliProcess_attr);

  #if (CFG_USB_INTERFACE_ENABLE != 0)
  #else
  MX_USART1_UART_Init();
  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1, RxCpltCallback);
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */
}

/**
 * @brief Perform initialization of TL for THREAD.
 * @param  None
 * @retval None
 */
void APP_THREAD_TL_THREAD_INIT(void)
{
  ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*)&ThreadOtCmdBuffer;
  ThreadConfigBuffer.p_ThreadNotAckBuffer = (uint8_t*)ThreadNotifRspEvtBuffer;
  ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*)&ThreadCliCmdBuffer;

  TL_THREAD_Init( &ThreadConfigBuffer );
}

/**
 * @brief  This function is called when notification on CLI TL Channel from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_CliNotReceived( TL_EvtPacket_t * Notbuffer )
{
  TL_CmdPacket_t* l_CliBuffer = (TL_CmdPacket_t*)Notbuffer;
  uint8_t l_size = l_CliBuffer->cmdserial.cmd.plen;

  /* WORKAROUND: if string to output is "> " then respond directly to M0 and do not output it */
  if (strcmp((const char *)l_CliBuffer->cmdserial.cmd.payload, "> ") != 0)
  {
    /* Write to CLI UART */
#if (CFG_USB_INTERFACE_ENABLE != 0)
    VCP_SendData( l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#else
    HW_UART_Transmit_IT(CFG_CLI_UART, l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#endif /*USAGE_OF_VCP */
  }
  else
  {
    Send_CLI_Ack_For_OT();
  }
}

/**
 * @brief  End of transfer callback for CLI UART sending.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void HostTxCb(void)
{
  Send_CLI_Ack_For_OT();
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
void APP_THREAD_ProcessMsgM0ToM4(void)
{
  if (CptReceiveMsgFromM0 != 0)
  {
    /* If CptReceiveMsgFromM0 is > 1. it means that we did not serve all the events from the radio */
    if (CptReceiveMsgFromM0 > 1U)
    {
      APP_THREAD_Error(ERR_REC_MULTI_MSG_FROM_M0, 0);
    }
    else
    {
      OpenThread_CallBack_Processing();
    }
    /* Reset counter */
    CptReceiveMsgFromM0 = 0;
  }
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief  This function is called when thereare some data coming
 *         from the Hyperterminal via the USB port
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  Buf: Buffer of data received
 * @param  Len: Number of data received (in bytes)
 * @retval Number of characters remaining in the buffer and not yet processed
 */
void VCP_DataReceived(uint8_t* Buf , uint32_t *Len)
{
  uint32_t i,flag_continue_checking = TRUE;
  uint32_t char_remaining = 0;
  static uint32_t len_total = 0;

  /* Copy the characteres in the temporary buffer */
  for (i = 0; i < *Len; i++)
  {
    TmpString[len_total++] = Buf[i];
  }

  /* Process the buffer commands one by one     */
  /* A command is limited by a \r caracaters    */
  while (flag_continue_checking == TRUE)
  {
    char_remaining = ProcessCmdString(TmpString,len_total);
    /* If char_remaining is equal to len_total, it means that the command string is not yet
     * completed.
     * If char_remaining is equal to 0, it means that the command string has
     * been entirely processed.
     */
    if ((char_remaining == 0) || (char_remaining == len_total))
    {
      flag_continue_checking = FALSE;
    }
    len_total = char_remaining;
  }
}
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
