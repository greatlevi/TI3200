/**
******************************************************************************
* @file     zc_ti_adpter.c
* @authors  hx
* @version  V1.0.0
* @date     2015
* @brief    Event
******************************************************************************
*/
/* for flash read/write */
#include "simplelink.h"
/* for mutex */
#include "user.h"
/* for uart read/write */
#include "uart_drv.h"
#include "prcm.h"
/* for create task */
#include "osi.h"
/* for board init */
#include "rom_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "pin.h"
#include "socket.h"
#include "hw_memmap.h"
#include "uart.h"
#include "gpio_if.h"
#include "timer.h"
#include <zc_protocol_controller.h>
#include <zc_timer.h>
#include <zc_module_interface.h>
#include <zc_hf_adpter.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdarg.h>

PTC_ModuleAdapter g_struHfAdapter;
MSG_Buffer g_struRecvBuffer;
MSG_Buffer g_struRetxBuffer;
MSG_Buffer g_struClientBuffer;
MSG_Queue  g_struRecvQueue;
ZC_UartBuffer g_struUartBuffer;
TI_Timer g_struTiTimer[ZC_TIMER_MAX_NUM];
MSG_Buffer g_struSendBuffer[MSG_BUFFER_SEND_MAX_NUM];
MSG_Queue  g_struSendQueue;
u8 g_u8MsgBuildBuffer[MSG_BULID_BUFFER_MAXLEN];
u8 g_u8ClientSendLen = 0;
u8 g_u8recvbuffer[TI_MAX_SOCKET_LEN];
u16 g_u16TiTimerCount[ZC_TIMER_MAX_NUM];
u8  g_u8BcSendBuffer[100];
u32 g_u32BcSleepCount = 800;
unsigned char UARTDataBuffer[1024];
unsigned long UARTDataCount = 0;
u16 g_u16TcpMss;
u16 g_u16LocalPort;
_SlLockObj_t g_struTimermutex;
_SlSyncObj_t g_TaskSem;
sockaddr_in struRemoteAddr;

extern PTC_ProtocolCon  g_struProtocolController;
extern uVectorEntry __vector_table;
extern volatile unsigned long  g_ulStatus;
/*************************************************
* Function: TI_ReadDataFromFlash
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History: 2015-8-15
*************************************************/
void TI_ReadDataFromFlash(void) 
{
	long			RetVal; 
    u32 u32MagicFlag = 0xFFFFFFFF;
	long			DeviceFileHandle = -1;

	/* 打开文件 */
	RetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                         FS_MODE_OPEN_READ,
                         NULL, &DeviceFileHandle);
	if (0 > RetVal)
	{
		TI_Printf("open file failed!\r\n");
		return;
	}
	/* 读取 */
	RetVal = sl_FsRead(DeviceFileHandle, 0, (unsigned char *)(&u32MagicFlag), sizeof(u32MagicFlag));
	if (RetVal != sizeof(u32MagicFlag))
	{
		TI_Printf("read head failed!\r\n");
		sl_FsClose(DeviceFileHandle, 0, 0, 0);
		return;
	}
    if (ZC_MAGIC_FLAG == u32MagicFlag)
    {   
        RetVal = sl_FsRead(DeviceFileHandle, 0, (unsigned char *)(&g_struZcConfigDb), sizeof(ZC_ConfigDB));
		if (RetVal != sizeof(ZC_ConfigDB))
		{
			TI_Printf("read body failed!\r\n");
			sl_FsClose(DeviceFileHandle, 0, 0, 0);
			return;
		}
    }
    else
    {
        TI_Printf("no para, use default\r\n");
	}
	/* 关闭文件 */
    RetVal = sl_FsClose(DeviceFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != RetVal)
    {
        TI_Printf("close file failed!\r\n");
    }
	return;
}

/*************************************************
* Function: TI_WriteDataToFlash
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_WriteDataToFlash(u8 *pu8Data, u16 u16Len)
{
	long			RetVal; 
	//char*			DeviceFileName = "record.txt";
	long			DeviceFileHandle = -1;
	/* 打开文件 */

	RetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                         FS_MODE_OPEN_WRITE,
                         NULL, &DeviceFileHandle);
	if (RetVal < 0)
	{
		ZC_Printf("write:open file failed!\r\n");
        RetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME, \
                           FS_MODE_OPEN_CREATE(1024, \
                           _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                           NULL, &DeviceFileHandle);
		//ASSERT_ON_ERROR(RetVal);
	}
	else
	{
		ZC_Printf("write:open file success!\r\n");
	}
	/* 写文件 */
	RetVal = sl_FsWrite(DeviceFileHandle, 0, pu8Data, (u32)u16Len);
	if (u16Len != RetVal)
	{
		ZC_Printf("write file failed!\r\n");
		return;		
	}
	ZC_Printf("write finished\r\n");
    RetVal = sl_FsClose(DeviceFileHandle, 0, 0, 0);
	
	return;
		
}
/*************************************************
* Function: TI_TimerInit
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_TimerInit(void)
{
	u32 u32i;
	
	for (u32i = 0; u32i < ZC_TIMER_MAX_NUM; u32i++)
	{
		g_struTiTimer[u32i].u8ValidFlag = 0;
	}

	/* 使用了1个定时器的资源 */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_TIMERA0);
    MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerPrescaleSet(TIMERA0_BASE, TIMER_A, 0);	

	MAP_TimerIntRegister(TIMERA0_BASE, TIMER_A, TI_timer_callback);
    MAP_TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);

	MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, MILLISECONDS_TO_TICKS(1000));
	MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
	
}
/*************************************************
* Function: TI_StopTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_StopTimer(u8 u8TimerIndex)
{
    g_struTiTimer[u8TimerIndex].u8ValidFlag = 0;
}
/*************************************************
* Function: TI_SetTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_SetTimer(u8 u8Type, u32 u32Interval, u8 *pu8TimeIndex)
{
    u8 u8TimerIndex;
    u32 u32Retval;
    u32Retval = TIMER_FindIdleTimer(&u8TimerIndex);

    if (ZC_RET_OK == u32Retval)
    {
        TIMER_AllocateTimer(u8Type, u8TimerIndex, (u8*)&g_struTiTimer[u8TimerIndex]);
        g_struTiTimer[u8TimerIndex].u32Interval = u32Interval / 1000;
        g_struTiTimer[u8TimerIndex].u8ValidFlag = 1;
        g_u16TiTimerCount[u8TimerIndex] = 0;
		*pu8TimeIndex = u8TimerIndex;
    }
    return u32Retval;
}
/*************************************************
* Function: TI_timer_callback
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_timer_callback(void) 
{
    unsigned long ulInts;
	u32  u32i = 0;

    ulInts = MAP_TimerIntStatus(TIMERA0_BASE, true);

    MAP_TimerIntClear(TIMERA0_BASE, ulInts);
	
    for (u32i = 0; u32i < ZC_TIMER_MAX_NUM; u32i++)
    {
        if (g_struTiTimer[u32i].u8ValidFlag)
        {
            if (g_struTiTimer[u32i].u32Interval == g_u16TiTimerCount[u32i]++)
            {
                g_u16TiTimerCount[u32i] = 0;
                TIMER_TimeoutAction(u32i);
				TIMER_StopTimer(u32i);
            }
        }
    }
}
/*************************************************
* Function: TI_FirmwareUpdateFinish
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_FirmwareUpdateFinish(u32 u32TotalLen)
{
	return 0;
}
/*************************************************
* Function: TI_FirmwareUpdate
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_FirmwareUpdate(u8 *pu8FileData, u32 u32Offset, u32 u32DataLen)
{
    return ZC_RET_OK;
}
/*************************************************
* Function: TI_SendDataToMoudle
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_SendDataToMoudle(u8 *pu8Data, u16 u16DataLen)
{
    u8 u8MagicFlag[4] = {0x02,0x03,0x04,0x05};

	Uart0Send(u8MagicFlag, 4);
	Uart0Send(pu8Data, u16DataLen);
    return ZC_RET_OK;
}

/*************************************************
* Function: TI_Rest
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Rest(void)
{
    g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;            
    TI_WriteDataToFlash((u8 *)&g_struZcConfigDb, sizeof(ZC_ConfigDB));
    
    TI_ResetNwp();

}
/*************************************************
* Function: TI_SendTcpData
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_SendTcpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    send(u32Fd, pu8Data, u16DataLen, 0);
}
/*************************************************
* Function: TI_SendUdpData
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_SendUdpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    sendto(u32Fd, (char*)pu8Data, u16DataLen, 0,
        (struct sockaddr *)pstruParam->pu8AddrPara,
        sizeof(sockaddr)); 
}
/*************************************************
* Function: uart0_handler
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void uart0_handler(void) 
{
	u8 ch = 0;
	unsigned long UART_STATUS = 0;

	UART_STATUS = UARTIntStatus(UARTA0_BASE, 1);  /* 获取经过屏蔽的中断的状态 */
		
	if ((UART_STATUS & UART_INT_RX))
	{
	    while(UARTCharsAvail(UARTA0_BASE))
	    {
	        ch = MAP_UARTCharGetNonBlocking(UARTA0_BASE);
	        
			UARTDataBuffer[UARTDataCount++] = ch;
		}
		MAP_UARTIntClear(UARTA0_BASE, UART_INT_RX);
	}
	sl_SyncObjSignal(&g_TaskSem);
}
/*************************************************
* Function: TI_RecvFromMcu
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/	
void TI_RecvFromMcu(void *arg) 
{
	while (1)
	{
		//TI_Printf("task2\r\n");
		osi_SyncObjWait(&g_TaskSem, OSI_WAIT_FOREVER);
		ZC_Moudlefunc((u8*)UARTDataBuffer, UARTDataCount);
		UARTDataCount = 0;
	}		
}
/*************************************************
* Function: TI_GetMac
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_GetMac(u8 *pu8Mac)
{
    unsigned char macAddressLen = SL_MAC_ADDR_LEN;
	u8 u8MacAddr[SL_MAC_ADDR_LEN] = {0};
	
    // Get the MAC address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL, &macAddressLen, u8MacAddr);

	ZC_HexToString(pu8Mac, u8MacAddr, SL_MAC_ADDR_LEN);
}
/*************************************************
* Function: TI_Reboot
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Reboot(void)
{
    TI_ResetNwp();
}
/*************************************************
* Function: TI_ConnectToCloud
* Description: 收到mcu接入云端请求的行为
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_ConnectToCloud(PTC_Connection *pstruConnection)
{
    int fd; 
    SlSockAddrIn_t addr;
    in_addr struIp;
    int retval;
    u16 port;
    memset((char*)&addr, 0, sizeof(addr));

    if (1 == g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig)
    {
        port = g_struZcConfigDb.struSwitchInfo.u16ServerPort;
        struIp.s_addr = g_struZcConfigDb.struSwitchInfo.u32ServerIp;
        retval = TI_SUCCESS;
    }
    else
    {
        port = ZC_CLOUD_PORT;
		retval = sl_NetAppDnsGetHostByName((_i8 *)g_struZcConfigDb.struCloudInfo.u8CloudAddr,
											strlen((const char *)g_struZcConfigDb.struCloudInfo.u8CloudAddr),
											(u32 *)&struIp,
											SL_AF_INET);
#if 0
		retval = gethostbyname("test.ablecloud.cn",
									//strlen(g_struZcConfigDb.struCloudInfo.u8CloudAddr),
									17,
									(u32 *)&struIp, 
									SL_AF_INET);
#endif
    }

    if (TI_SUCCESS != retval)
    {
    	TI_Printf("sl_NetAppDnsGetHostByName error\n\r");
        return ZC_RET_ERROR;
    }
    TI_Printf("connect ip = 0x%x!\r\n",struIp.s_addr);

    addr.sin_family = AF_INET;
    addr.sin_port = sl_Htons(port);
    addr.sin_addr.s_addr = sl_Htonl(struIp.s_addr);

    fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd < 0)
        return ZC_RET_ERROR;
	
	retval = connect(fd, ( SlSockAddr_t *)&addr, sizeof(addr));
    if (retval < 0)
    {
    	TI_Printf("connect ret is %d\n\r", retval);
        close(fd);
        if (g_struProtocolController.struCloudConnection.u32ConnectionTimes++ > 20)
        {
           g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
        }

        return ZC_RET_ERROR;
    }
	TI_Printf("connect ok!\r\n");
    g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;

    g_struProtocolController.struCloudConnection.u32Socket = fd;

    ZC_Rand(g_struProtocolController.RandMsg);

    return ZC_RET_OK;
}

/*************************************************
* Function: TI_ListenClient
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 TI_ListenClient(PTC_Connection *pstruConnection)
{
    int fd; 
    sockaddr_in servaddr;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0)
        return ZC_RET_ERROR;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(pstruConnection->u16Port);  /* 9689 */
    if (bind(fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        close(fd);
        return ZC_RET_ERROR;
    }
    
    if (listen(fd, 0) < 0)
    {
        close(fd);
        return ZC_RET_ERROR;
    }

    ZC_Printf("Tcp Listen Port = %d\r\n", pstruConnection->u16Port);
    g_struProtocolController.struClientConnection.u32Socket = fd;

    return ZC_RET_OK;
}

/*************************************************
* Function: TI_Printf
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Printf(const char *pu8format, ...)
{
    char buffer[100 + 1]={0};
    va_list arg;
    va_start (arg, pu8format);
    vsnprintf(buffer, 100, pu8format, arg);
    va_end (arg);
	TI_Message(buffer);
}
/*************************************************
* Function: TI_BcInit
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_BcInit()
{
    sockaddr_in addr; 

    addr.sin_family = AF_INET; 
    addr.sin_port = htons(ZC_MOUDLE_PORT);    /* 7689 */
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    g_Bcfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
    if(g_Bcfd < 0)
    {
        TI_Printf("socket error, g_Bcfd is %d\r\n", g_Bcfd);
        return;
    }

	/* 没有配置广播的接口 ti 没有SO_BROADCAST这个宏 */
    bind(g_Bcfd, (sockaddr*)&addr, sizeof(addr)); 
	
    g_struProtocolController.u16SendBcNum = 0;

    memset((char*)&struRemoteAddr, 0, sizeof(struRemoteAddr));
    struRemoteAddr.sin_family = AF_INET; 
    struRemoteAddr.sin_port = htons(ZC_MOUDLE_BROADCAST_PORT);   /* 8689 */
    struRemoteAddr.sin_addr.s_addr = 0xFFFFFFFF;
    g_pu8RemoteAddr = (u8*)&struRemoteAddr;
    g_u32BcSleepCount = 100;

    return;
}
/*************************************************
* Function: TI_StartDevice
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_StartDevice(void)
{
	int RetVal;
    RetVal = sl_Start(NULL, NULL, NULL);
    if(RetVal < 0 || ROLE_STA != RetVal)
    {
		TI_Printf("sl_Start error\n\r");
        while (1);
    }
	else
	{
		TI_Printf("start device success\n\r");
	}
	return;
}
/*************************************************
* Function: TI_Init
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Init()
{
	s32 RetVal;

	sl_LockObjCreate(&g_struTimermutex, "timer_mutex");
	sl_SyncObjCreate(&g_TaskSem, "task_sem");
	//InitTerm();

	TI_TimerInit();
    TI_Printf("TI Init\r\n");	
    //网络通信接口
    g_struHfAdapter.pfunConnectToCloud = TI_ConnectToCloud;
    g_struHfAdapter.pfunListenClient = TI_ListenClient;
    g_struHfAdapter.pfunSendTcpData = TI_SendTcpData; 
    g_struHfAdapter.pfunSendUdpData = TI_SendUdpData;     
    g_struHfAdapter.pfunUpdate = TI_FirmwareUpdate;  
    //设备内部通信接口
    g_struHfAdapter.pfunSendToMoudle = TI_SendDataToMoudle; 
    //定时器类接口
    g_struHfAdapter.pfunSetTimer = TI_SetTimer;   
    g_struHfAdapter.pfunStopTimer = TI_StopTimer;        
    //存储类接口
    g_struHfAdapter.pfunUpdateFinish = TI_FirmwareUpdateFinish;
    g_struHfAdapter.pfunWriteFlash = TI_WriteDataToFlash;
    //系统类接口    
    g_struHfAdapter.pfunRest = TI_Rest;    
    g_struHfAdapter.pfunGetMac = TI_GetMac;
    g_struHfAdapter.pfunReboot = TI_Reboot;
    g_struHfAdapter.pfunMalloc = mem_Malloc;
    g_struHfAdapter.pfunFree = mem_Free;
    g_struHfAdapter.pfunPrintf = TI_Printf;
    g_u16TcpMss = 1000;
	
    PCT_Init(&g_struHfAdapter);
    
    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;

	RetVal = osi_TaskCreate(TI_Cloudfunc, 
				            "TI_Cloudfunc",
				             TASK_STACK_SIZE,
				             NULL,
				             3,
				             NULL);
	if (RetVal < 0)
	{
		TI_Printf("create TI_Cloudfunc error\r\n");
		return;
	}
	/* 读串口 */
	RetVal = osi_TaskCreate(TI_RecvFromMcu, 
				            "TI_RecvFromMcu",
				             TASK_STACK_SIZE,
				             NULL,
				             2,
				             NULL);
	if (RetVal < 0)
	{
		TI_Printf("create TI_RecvFromMcu error\r\n");
		return;
	}

	osi_start();    
}
/*************************************************
* Function: TI_WakeUp
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_WakeUp()
{
    PCT_WakeUp();
}
/*************************************************
* Function: TI_Sleep
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Sleep()
{
    u32 u32Index;
    
    close(g_Bcfd);

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        close(g_struProtocolController.struClientConnection.u32Socket);
        g_struProtocolController.struClientConnection.u32Socket = PCT_INVAILD_SOCKET;
    }

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struCloudConnection.u32Socket)
    {
        close(g_struProtocolController.struCloudConnection.u32Socket);
        g_struProtocolController.struCloudConnection.u32Socket = PCT_INVAILD_SOCKET;
    }
    
    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
            close(g_struClientInfo.u32ClientFd[u32Index]);
            g_struClientInfo.u32ClientFd[u32Index] = PCT_INVAILD_SOCKET;
        }
    }

    PCT_Sleep();
    
    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;
}
/*************************************************
* Function: TI_RebootNwp
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
long TI_RebootNwp(void)
{
	long lRetVal = -1;
	/* Restart Network processor */
	lRetVal = sl_Stop(TI_STOP_TIMEOUT);
	
	lRetVal = sl_Start(NULL, NULL, NULL);
	if (lRetVal < 0)
	{
		ZC_Printf("start error!\n");
	}
	return lRetVal;
}
/*************************************************
* Function: TI_ResetNwp
* Description: 重置wifi密码
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
int TI_ResetNwp(void)
{
	SmartConfigConnect();
}
/*************************************************
* Function: BoardInit
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
/*************************************************
* Function: PinMuxConfig
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void PinMuxConfig(void)
{

	/* 
	Pin#, PORT IO, Function, Direction
	3,GP12,UART0_TX,
	4,GP13,UART0_RX,
	55,GP01,UART1_TX,
	57,GP02,UART1_RX,
	*/
    // Enable Peripheral Clocks 
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);

    // Configure PIN_03 for UART0 UART0_TX
    PinTypeUART(PIN_03, PIN_MODE_7);
    // Configure PIN_04 for UART0 UART0_RX
    PinTypeUART(PIN_04, PIN_MODE_7);
    // Configure PIN_55 for UART1 UART1_TX
    PinTypeUART(PIN_55, PIN_MODE_6);
    // Configure PIN_57 for UART1 UART1_RX
    PinTypeUART(PIN_57, PIN_MODE_6);
    MAP_UARTConfigSetExpClk(UARTA0_BASE,
    						MAP_PRCMPeripheralClockGet(PRCM_UARTA0), 
                  			UART_BAUD_RATE, 
                  			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
    MAP_UARTConfigSetExpClk(UARTA1_BASE,
							MAP_PRCMPeripheralClockGet(PRCM_UARTA1), 
							UART_BAUD_RATE,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));

}
/*************************************************
* Function: InitTerm
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void InitTerm()
{

	MAP_UARTIntRegister(UARTA0_BASE, uart0_handler); //enable interrupts
	MAP_UARTIntEnable(UARTA0_BASE, UART_INT_RX);

	UARTFIFOLevelSet(UARTA0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
}
/*************************************************
* Function: WlanConnect
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
long WlanConnect(void)
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    if (lRetVal < 0)
	{
		TI_Printf("sl_WlanConnect error\n");
	}

    return 0;
   
}
/*************************************************
* Function: TI_Message
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Message(const char *str)
{
#ifndef NOTERM
    if(str != NULL)
    {
        while(*str!='\0')
        {
            MAP_UARTCharPut(UARTA1_BASE,*str++);
        }
    }
#endif
}
/*************************************************
* Function: Uart0Send
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void Uart0Send(u8  *Buffer, u16  NByte)
{
    while (NByte) {
        if ( UARTSpaceAvail(UARTA0_BASE) ) {
            MAP_UARTCharPut(UARTA0_BASE, *Buffer++);
            NByte--;
        }
    }
    while (UARTBusy(UARTA0_BASE) ) {
        ;
    }
}
/*************************************************
* Function: Uart1Send
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void Uart1Send(u8  *Buffer, u16  NByte)
{
    while (NByte) {
        if ( UARTSpaceAvail(UARTA1_BASE) ) {
            UARTCharPutNonBlocking(UARTA1_BASE, *Buffer++);
            NByte--;
        }
    }
    while (UARTBusy(UARTA1_BASE) ) {
        ;
    }
}
/*************************************************
* Function: TI_Cloudfunc
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_Cloudfunc(void* arg) 
{
    int fd;
    u32 u32Timer = 0;

	TI_StartDevice();
	TI_ReadDataFromFlash();
	//TI_ClearRec();

    TI_BcInit();
	InitTerm();
    while(1) 
    {
    	//TI_Printf("task1\r\n");
    	if (IS_IP_ACQUIRED(g_ulStatus))
		{
			TI_WakeUp();
			CLR_STATUS_BIT(g_ulStatus, 3);
		}
        fd = g_struProtocolController.struCloudConnection.u32Socket;
        PCT_Run();
        TI_CloudRecvfunc(NULL);
        if (PCT_STATE_DISCONNECT_CLOUD == g_struProtocolController.u8MainState)
        {
            close(fd);
            if (0 == g_struProtocolController.struCloudConnection.u32ConnectionTimes)
            {
                u32Timer = 1000;
            }
            else
            {
                u32Timer = rand();
                u32Timer = (PCT_TIMER_INTERVAL_RECONNECT) * (u32Timer % 10 + 1);
            }
            PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
            g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
            g_struUartBuffer.u32RecvLen = 0;
        }
        else
        {
            MSG_SendDataToCloud((u8*)&g_struProtocolController.struCloudConnection);
        }
        ZC_SendBc();
		osi_Sleep(10);
    } 
}
/*************************************************
* Function: TI_CloudRecvfunc
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void TI_CloudRecvfunc(void* arg) 
{
	s8 s8ret;
    s32 s32RecvLen=0; 
    fd_set fdread;
    u32 u32Index;
    u16 u16Len = 0; 
    u32 u32ActiveFlag = 0;
    struct sockaddr_in cliaddr;
    int connfd;
    extern u8 g_u8ClientStart;
    u32 u32MaxFd = 0;
    struct timeval timeout; 
    struct sockaddr_in addr;
    int tmp=1;    

    ZC_StartClientListen();  /* 端口号9689，直连 */

    u32ActiveFlag = 0;
    
    timeout.tv_sec= 0; 
    timeout.tv_usec= 1000; 
    
    FD_ZERO(&fdread);

    FD_SET(g_Bcfd, &fdread);
    u32MaxFd = u32MaxFd > g_Bcfd ? u32MaxFd : g_Bcfd;
	
	/* 用于监听直连 */
    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        FD_SET(g_struProtocolController.struClientConnection.u32Socket, &fdread);
        u32MaxFd = u32MaxFd > g_struProtocolController.struClientConnection.u32Socket ? u32MaxFd : g_struProtocolController.struClientConnection.u32Socket;
        u32ActiveFlag = 1;
    }
    /* 连接云端 */
    if ((g_struProtocolController.u8MainState >= PCT_STATE_WAIT_ACCESSRSP) 
    && (g_struProtocolController.u8MainState < PCT_STATE_DISCONNECT_CLOUD))
    {
        FD_SET(g_struProtocolController.struCloudConnection.u32Socket, &fdread);
        u32MaxFd = u32MaxFd > g_struProtocolController.struCloudConnection.u32Socket ? u32MaxFd : g_struProtocolController.struCloudConnection.u32Socket;
        u32ActiveFlag = 1;
    }

    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
            FD_SET(g_struClientInfo.u32ClientFd[u32Index], &fdread);
            u32MaxFd = u32MaxFd > g_struClientInfo.u32ClientFd[u32Index] ? u32MaxFd : g_struClientInfo.u32ClientFd[u32Index];
            u32ActiveFlag = 1;            
        }
    }

    if (0 == u32ActiveFlag)
    {
        return;
    }
    
    s8ret = select(u32MaxFd + 1, &fdread, NULL, NULL, &timeout);
    if(s8ret <= 0)
    {
       return;
    }
    if ((g_struProtocolController.u8MainState >= PCT_STATE_WAIT_ACCESSRSP) 
    && (g_struProtocolController.u8MainState < PCT_STATE_DISCONNECT_CLOUD))
    {
        if (FD_ISSET(g_struProtocolController.struCloudConnection.u32Socket, &fdread))
        {
            s32RecvLen = recv(g_struProtocolController.struCloudConnection.u32Socket, g_u8recvbuffer, TI_MAX_SOCKET_LEN, 0); 
            
            if(s32RecvLen > 0) 
            {
                ZC_Printf("recv data len = %d\r\n", s32RecvLen);
                MSG_RecvDataFromCloud(g_u8recvbuffer, s32RecvLen);
            }
            else
            {
                ZC_Printf("recv error, len = %d\n",s32RecvLen);
                PCT_DisConnectCloud(&g_struProtocolController);
                
                g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
                g_struUartBuffer.u32RecvLen = 0;
            }
        }
        
    }

    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
        	/* g_struClientInfo.u32ClientFd 是tcp的新句柄 */
            if (FD_ISSET(g_struClientInfo.u32ClientFd[u32Index], &fdread))
            {
                s32RecvLen = recv(g_struClientInfo.u32ClientFd[u32Index], g_u8recvbuffer, TI_MAX_SOCKET_LEN, 0); 
                if (s32RecvLen > 0)
                {
                    ZC_RecvDataFromClient(g_struClientInfo.u32ClientFd[u32Index], g_u8recvbuffer, s32RecvLen);
                }
                else
                {   
                    ZC_ClientDisconnect(g_struClientInfo.u32ClientFd[u32Index]);
                    close(g_struClientInfo.u32ClientFd[u32Index]);
                }
                
            }
        }
        
    }

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        if (FD_ISSET(g_struProtocolController.struClientConnection.u32Socket, &fdread))
        {
            connfd = accept(g_struProtocolController.struClientConnection.u32Socket,(struct sockaddr *)&cliaddr, &u16Len);

            if (ZC_RET_ERROR == ZC_ClientConnect((u32)connfd))
            {
                close(connfd);
            }
            else
            {
                ZC_Printf("accept client = %d\n", connfd);
            }
        }
    }
	/* 局域网发现，收到别的设备发过来消息，同时发送自己的信息 */
    if (FD_ISSET(g_Bcfd, &fdread))
    {
        tmp = sizeof(addr); 
        s32RecvLen = recvfrom(g_Bcfd, g_u8BcSendBuffer, 100, 0, (struct sockaddr *)&addr, (socklen_t*)&tmp); 
        if(s32RecvLen > 0) 
        {
            ZC_SendClientQueryReq(g_u8BcSendBuffer, (u16)s32RecvLen);
        } 
    }
 
}
/*************************************************
* Function: SmartConfigConnect
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
long SmartConfigConnect(void)
{
    unsigned char policyVal;
    long lRetVal = -1;
    //
    // Clear all profiles 
    // This is of course not a must, it is used in this example to make sure
    // we will connect to the new profile added by SmartConfig
    //
    lRetVal = sl_WlanProfileDel(WLAN_DEL_ALL_PROFILES);
    ASSERT_ON_ERROR(lRetVal);

    //set AUTO policy
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                      SL_CONNECTION_POLICY(1,0,0,0,1),
                      &policyVal,
                      1 /*PolicyValLen*/);
    ASSERT_ON_ERROR(lRetVal);

	lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
			MAP_UtilsDelay(40000000);
        }
    }
	//MAP_UtilsDelay(80000000 * 2);
    TI_Sleep();
    TI_BcInit();
    //
    // Start SmartConfig
    // This example uses the unsecured SmartConfig method
    //
    lRetVal = sl_WlanSmartConfigStart(0,                /*groupIdBitmask*/
                           SMART_CONFIG_CIPHER_NONE,    /*cipher*/
                           0,                           /*publicKeyLen*/
                           0,                           /*group1KeyLen*/
                           0,                           /*group2KeyLen */
                           NULL,                        /*publicKey */
                           NULL,                        /*group1Key */
                           NULL);                       /*group2Key*/  
                        
    ASSERT_ON_ERROR(lRetVal);

	TI_Printf("waiting smartconfig\r\n");
    return TI_SUCCESS;
}

/******************************* FILE END ***********************************/


