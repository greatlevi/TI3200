#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zc_ti_adpter.h>
#include <zc_common.h>
#include "wlan.h"
#include "device.h"
#include "socket.h"
#include "osi.h"



volatile unsigned long  g_ulStatus = 0;
extern u32 g_u32GloablIp;

#ifdef USE_FREERTOS
/*************************************************
* Function: vAssertCalled
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    //Handle Assert here
    while(1)
    {

    }
}
/*************************************************
* Function: vApplicationIdleHook
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void vApplicationIdleHook( void)
{
    //Handle Idle Hook for Profiling, Power Management etc
}
/*************************************************
* Function: vApplicationMallocFailedHook
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void vApplicationMallocFailedHook()
{
    //Handle Memory Allocation Errors
    while(1)
    {

    }
}
/*************************************************
* Function: vApplicationStackOverflowHook
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void vApplicationStackOverflowHook(OsiTaskHandle *pxTask, 
                                   signed char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {

    }
}
#endif //USE_FREERTOS  
/*************************************************
* Function: SimpleLinkWlanEventHandler
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    switch(pSlWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
            TI_Printf("wifi sta connected!!\r\n");
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            break;
        case SL_WLAN_DISCONNECT_EVENT:
            TI_Printf("wifi sta disconnected!!\r\n");
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            TI_Printf("wifi sta disconnected end!!\r\n"); 
            break;
        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
            TI_Printf("smart config success!!\r\n"); 
            break;
        default:
            ZC_Printf("[WLAN EVENT] Unexpected event \r\n");
            break;
    }
}
/*************************************************
* Function: SimpleLinkNetAppEventHandler
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{

    SlIpV4AcquiredAsync_t *pEventData = NULL;
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
            //Ip Acquired Event Data
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_u32GloablIp = pEventData->ip;
            TI_Printf("dhcp ok ip is 0x%08x\r\n", g_u32GloablIp); 
            break;
        default:
            break;
    }
}
/*************************************************
* Function: SimpleLinkHttpServerCallback
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}
/*************************************************
* Function: SimpleLinkGeneralEventHandler
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    TI_Printf("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status, 
               pDevEvent->EventData.deviceEvent.sender);
}
/*************************************************
* Function: SimpleLinkSockEventHandler
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
}
/*************************************************
* Function: main
* Description: 
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
int main(void)
{
    BoardInit();
    PinMuxConfig();
    InitTerm();
    TI_Printf("HELLO TI4\n\r");
    // Start the SimpleLink Host
    VStartSimpleLinkSpawnTask(TI_SPAWN_TASK_PRIORITY);
    TI_Init();
    return 1;
}
