/**
******************************************************************************
* @file     zc_hf_adpter.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    HANDSHAKE
******************************************************************************
*/
#ifndef  __ZC_TI_ADPTER_H__ 
#define  __ZC_TI_ADPTER_H__

#include <zc_common.h>
#include <zc_protocol_controller.h>
#include <zc_module_interface.h>
#include <bmd.h>
#include "uart_if.h"
#include "wlan.h"
#include "common.h"


#define NUM_DESCS                         30
#define STATUS_BIT_DISCONNECTED          9

#define RCTRL_MSG_FLAG		               0x02030405
#define RCTRL_MSG_PREFIX		           "\2\3\4\5"
#define AC_PAYLOADLENOFFSET              9
#define UART0RX_RING_LEN                 1024   
#define USER_FILE_NAME                   "record.txt"
#define OTA_IMAGE_NAME                   "/sys/mcuimg2.bin"

#define    IMAGE_MAX_SIZE                (1024 * 100)
#define    TASK_STACK_SIZE               2048
#define    UART_BAUD_RATE                115200
#define    SYSCLK                         80000000
#define    TI_SPAWN_TASK_PRIORITY        (9)

#define    PCT_TIMER_RECONNECT          (0)
#define    PCT_TIMER_REACCESS           (1)
#define    PCT_TIMER_SENDMOUDLE         (2)
#define    PCT_TIMER_SENDHEART          (3)
#define    PCT_TIMER_REGISTER           (4)
#define    PCT_TIMER_REBOOT              (5)

#define TI_MAX_SOCKET_LEN               (1000)
#define TI_STOP_TIMEOUT                  (200)

#define WLAN_DEL_ALL_PROFILES           (0xFF)

#define TI_SUCCESS                        0

#define SSID_NAME                         "HW_test"          /* AP SSID */
#define SECURITY_TYPE                     SL_SEC_TYPE_WPA    /* Security type (OPEN or WEP or WPA*/
#define SECURITY_KEY                      "87654321"          /* Password of the secured AP */
#define SSID_LEN_MAX                      32
#define BSSID_LEN_MAX                     6

#define SYS_CLK				                80000000
#define MILLISECONDS_TO_TICKS(ms)        ((SYS_CLK/1000) * (ms))

#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & (1<<(bit))))
#define TI_IS_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, 1)
#define TI_IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable, 3)
#define TI_IS_DISCONNECTED(status_variable)      GET_STATUS_BIT(status_variable, STATUS_BIT_DISCONNECTED)

#define SET_STATUS_BIT(status_variable, bit)     status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)     status_variable &= ~(1<<(bit))

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

typedef enum {
    PKT_UNKNOWN,
    PKT_ATCMD,
    PKT_PUREDATA,
    PKT_ZADATA,
    PKT_PRINTCMD,
    PKT_HR01DATA
}PKT_TYPE;

typedef struct {
    PKT_TYPE pkt_type;
    u16   pkt_len;
}PKT_FIFO;//packet infor is in sequence with index[0,num_desc-1] which mapping the sequence in rx

typedef struct {
    PKT_TYPE  cur_type;              //receiving packet:which packet type is receiving current? 
    u8     cur_num;               //receiving packet:current index of receiving packet
    u8     pkt_num;               //completely packets:packet number in ring buffer
    PKT_FIFO  infor[NUM_DESCS];      //completely packets:FIFO,packet infor in ring
}PKT_DESC; 

typedef struct tag_RCTRL_STRU_MSGHEAD
{
    u32 MsgFlag;         
    ZC_MessageHead  struMessageHead;
}RCTRL_STRU_MSGHEAD;

typedef struct
{
    BUFFER_INFO                    Rx_Buffer;  //receive buffer
    PKT_DESC                       Rx_desc;    //description       
    BUFFER_INFO                    Tx_Buffer;  //transmit buffer    
} UARTStruct;


#ifdef __cplusplus
extern "C" {
#endif

void TI_Init(void);
void TI_WakeUp(void);
void TI_Sleep(void);
void TI_WriteDataToFlash(u8 *pu8Data, u16 u16Len);
void BoardInit(void);
void PinMuxConfig(void);
void  InitTerm(void);
void TI_StartDevice(void);
void TI_Cloudfunc(void* arg);
void TI_CloudRecvfunc(void* arg);
long WlanConnect(void);
void TI_Message(const char *str);
void Uart0Send(u8  *Buffer, u16  NByte);
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
long SmartConfigConnect(void);
void UARTRx_Buf_Init(UARTStruct *qp, u8 *rxbuf, u16 len);
void UartInit(void);
void RebootMCU(void);
s32 TI_FindIdleImage(void);


#ifdef __cplusplus
}
#endif
#endif
/******************************* FILE END ***********************************/

