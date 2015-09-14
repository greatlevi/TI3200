/**
******************************************************************************
* @file     zc_hf_adpter.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    HANDSHAKE
******************************************************************************
*/

#ifndef  __ZC_HF_ADPTER_H__ 
#define  __ZC_HF_ADPTER_H__

#include <cc_types.h>
#include <zc_common.h>
#include <zc_protocol_controller.h>
#include <zc_module_interface.h>

#include "uart_if.h"
#include "wlan.h"


typedef struct 
{
    u32 u32Interval;  
    u8 u8TimerIndex;
    u8 u8ValidFlag;
}TI_Timer;

typedef enum
{
  NO_SW,
  SW1 = 0x1, /* SW1/Reset Button */
  SW2 = 0x2, /* SW2/GP22/Pin15 */
  SW3 = 0x4  /* SW3/GP13/Pin4 */
}eSwNum;
#if 0
// Status bits - These are used to set/reset the corresponding bits in 
// given variable
typedef enum{
    STATUS_BIT_NWP_INIT = 0, // If this bit is set: Network Processor is 
                             // powered up
                             
    STATUS_BIT_CONNECTION,   // If this bit is set: the device is connected to 
                             // the AP or client is connected to device (AP)
                             
    STATUS_BIT_IP_LEASED,    // If this bit is set: the device has leased IP to 
                             // any connected client

    STATUS_BIT_IP_AQUIRED,   // If this bit is set: the device has acquired an IP
    
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration 
                                  // process is started from SmartConfig app

    STATUS_BIT_P2P_DEV_FOUND,    // If this bit is set: the device (P2P mode) 
                                 // found any p2p-device in scan

    STATUS_BIT_P2P_REQ_RECEIVED, // If this bit is set: the device (P2P mode) 
                                 // found any p2p-negotiation request

    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
                                  // connection to client(or reverse way) is failed

    STATUS_BIT_PING_DONE         // If this bit is set: the device has completed
                                 // the ping operation

}e_StatusBits;
#endif

// check the error code and handle it
#define ASSERT_ON_ERROR(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        return error_code;\
                 }\
            }


#define USER_FILE_NAME          "record.txt"

#define    TASK_STACK_SIZE                   2048
#define    UART_BAUD_RATE                    115200
#define    SYSCLK                             80000000
#define    BUF_SIZE                           2048

#define    PCT_TIMER_RECONNECT              (0)
#define    PCT_TIMER_REACCESS               (1)
#define    PCT_TIMER_SENDMOUDLE             (2)
#define    PCT_TIMER_SENDHEART              (3)
#define    PCT_TIMER_REGISTER               (4)


#define TI_MAX_SOCKET_LEN    (1000)
#define TI_STOP_TIMEOUT      (200)

#define WLAN_DEL_ALL_PROFILES    (0xFF)

#define TI_SUCCESS              0

#define SSID_NAME           "HW_test"    /* AP SSID */
#define SECURITY_TYPE       SL_SEC_TYPE_WPA/* Security type (OPEN or WEP or WPA*/
#define SECURITY_KEY        "87654321"      /* Password of the secured AP */
#define SSID_LEN_MAX        32
#define BSSID_LEN_MAX       6


#define SYS_CLK				    80000000
#define MILLISECONDS_TO_TICKS(ms)   ((SYS_CLK/1000) * (ms))

#define GET_STATUS_BIT(status_variable, bit) (0 != (status_variable & (1<<(bit))))
#define IS_CONNECTED(status_variable)        GET_STATUS_BIT(status_variable,\
                                                         1)
#define IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable,\
                                                          3)                                                       
#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))




typedef void (*P_INT_HANDLER)(void);



//typedef void * cc_hndl;

#ifdef __cplusplus
extern "C" {
#endif


void TI_Init(void);
void TI_WakeUp(void);
void TI_Sleep(void);
//void TI_ReadDataFromFlash(void);
void TI_WriteDataToFlash(u8 *pu8Data, u16 u16Len);
//s8 TI_get_timer_id(cc_hndl htimer);
long TI_RebootNwp(void);
int TI_ResetNwp(void);
void BoardInit(void);
void PinMuxConfig(void);
void  InitTerm();
void TI_StartDevice(void);
void TI_Cloudfunc(void* arg);
//void TI_RecvFromMcu(void);
void TI_CloudRecvfunc(void* arg);
long WlanConnect(void);
void TI_Message(const char *str);
void Uart0Send(u8  *Buffer, u16  NByte);
void Uart1Send(u8  *Buffer, u16  NByte);
void TI_Printf(const char *pu8format, ...);
void TI_RecvFromMcu(void *arg) ;
void uart0_handler(void);
void TI_StopTimer(u8 u8TimerIndex);
u32 TI_SetTimer(u8 u8Type, u32 u32Interval, u8 *pu8TimeIndex);
void TI_ReadDataFromFlash(void);
void TI_ClearRec(void);
void TI_TimerInit(void);
void TI_StartTimer(u8 u8TimerIndex, u32 u32Interval);
void TI_timer_callback(void);
void TI_BcInit(void);
void PinMuxConfig(void);
void Button_IF_Init(P_INT_HANDLER S2InterruptHdl);
void GPIOs2IntHandler();
void Button_IF_DisableInterrupt(unsigned char ucSwitch);
void SW2_SmartConfig(void);
long SmartConfigConnect(void);



#ifdef __cplusplus
}
#endif
#endif
/******************************* FILE END ***********************************/

