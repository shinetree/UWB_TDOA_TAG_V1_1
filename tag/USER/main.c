#include <stdlib.h>
#include <stdio.h>
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"
#include "port.h"
#include "instance.h"
#include "deca_types.h"
#include "deca_spi.h"
#include "device_info.h" 
#include "stmflash.h"
#include "stm32f10x_tim.h"

#include "dma.h"
#include "deca_regs.h"
#include "hw_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


#define TX_ANT_DLY 0
#define RX_ANT_DLY 0
#define BLINK_FRAME_SN_IDX 1
#define TX_DELAY_MS 200
#define compensate_flag 0           /*²¹³¥±êÇ©±êÖ¾  1Îª²¹³¥ 0Îª¶¨Î»*/
#define TAG_DDRL 0xD0;         /*µÍÎ»±êÇ©µØÖ·*/
#define TAG_DDRH 0x01;         /*¸ßÎ»±êÇ©µØÖ·*/
#define Frequency_parameter 20 /*·¢ËÍÆµÂÊ µ¥Î»HZ*/
uint64_t timer3_tick_ms=0; //Timer3¶¨Ê±Æ÷¼ÆÊý±äÁ¿(ms)

u8 SendBuff[130];
u8 USB_RxBuff[30];
static uint8 tx_msg[] = {0xC5, 0,0xc1, 0xc1, 0xee, 0xee, 'V', 'E', 0, 0};     /*²¹³¥Ö¡Ê*/
static uint8 location_compensate[] ={0xC5, 0,0xc1, 0xc1, 0xff, 0xff, 'V', 'E', 0, 0};    /*¶¨Î»Ö¡Ê*/
#define DUMMY_BUFFER_LEN 600
static uint8 dummy_buffer[DUMMY_BUFFER_LEN];

__IO unsigned long time32_incr;

 static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    1,
	  1057 
};
 struct base_task_t
{
  TaskHandle_t          thread;
  QueueHandle_t        evtq;
  
  bool             isactive;
};
 struct base_task_t base_task;

void vApplicationIdleHook( void )
{
	vTaskResume(base_task.thread);
}

//void SysTick_Handler(void)
//{
//	time32_incr++;	
//}
void vApplicationTickHook(void)
{
	time32_incr++;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  __nop;
}
uint64_t task_tick_old = 0;
uint64_t task_tick_new = 0;

void  base_task_thead(void * arg)
{
    uint8 t1[5];
    int time_slot = 0;
    base_task.isactive = 1;
    task_tick_new = timer3_tick_ms;

    while (base_task.isactive)
    {
        led_off(LED_ALL);
        vTaskDelay(10);
        if(compensate_flag==0)
        {
            task_tick_new = timer3_tick_ms;
            time_slot += (task_tick_new - task_tick_old);

            if(time_slot > TX_DELAY_MS )  
            {
                time_slot = 0;
                dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); 
                dwt_writetxfctrl(sizeof(tx_msg), 0);     
                dwt_starttx(DWT_START_TX_IMMEDIATE);
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                {
                };
                led_on(LED_ALL);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
//				dwt_readtxtimestamp(t1);
//			    memcpy(&tx_msg[4], t1, 5);
//				USB_TxWrite(&tx_msg[1],1);
                USB_TxWrite(tx_msg,7);
                tx_msg[1]++;
                dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
            }                
        }
        else
        {
            dwt_writetxdata(sizeof(location_compensate), location_compensate, 0); 
            dwt_writetxfctrl(sizeof(location_compensate), 0);     
            dwt_starttx(DWT_START_TX_IMMEDIATE);
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
            {
            };
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            vTaskDelay(1000/Frequency_parameter);
            dwt_readtxtimestamp(t1);
            memcpy(&location_compensate[4], t1, 5);
            USB_TxWrite(&location_compensate[1],1);
            USB_TxWrite(&location_compensate[4],5);
            location_compensate[1]++;
        }
        task_tick_old = task_tick_new;        
    }
}
uint32_t base_task_start ()
{
  if (base_task.isactive)
    return 1;

  // Start execution.
  if (pdPASS != xTaskCreate (base_task_thead, "BASE", 256, &base_task, 4, &base_task.thread))
  {
    return 1;
  }
  return 0;
}
void Timer3_Init(u16 arr,u16 psc)
{
     TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
     NVIC_InitTypeDef NVIC_InitStructure;
 
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
     TIM_TimeBaseStructure.TIM_Period = (arr - 1); 
     TIM_TimeBaseStructure.TIM_Prescaler =(psc-1); 
     TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
     TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
  
      
     TIM_ITConfig(  
         TIM3, 
         TIM_IT_Update  |  
         TIM_IT_Trigger,  
         ENABLE  
         );
      
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3??
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
     NVIC_Init(&NVIC_InitStructure);  
 
     TIM_Cmd(TIM3, ENABLE);  
                              
 }
 
 void TIM3_IRQHandler(void)   
 {
     if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
         {
            TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  
            timer3_tick_ms += 10;
         }
}

int main(void)
{
    int i = 0;
    u16 ShortAddress16=0xaaaa;
    u16 panID=0xdeca;
    u8 source[8];
    uint8 eui64[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};	
    delay_init();
    uart_init(115200);	 //´®¿Ú³õÊ¼»¯Îª115200 	
    GPIO_Configuration();//³õÊ¼»¯ÓëLEDÁ¬½ÓµÄÓ²¼þ½Ó¿Ú
    SPI_Configuration();
    peripherals_init();
    Timer3_Init(100,7200);  //10ms
    delay_ms(1000);
    Flash_Configuration();
    delay_ms(200);
    USB_Config();
    MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,130);//DMA1Í¨µÀ4,ÍâÉèÎª´®¿Ú1,´æ´¢Æ÷ÎªSendBuff,³¤¶È130.  
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ  
    MYDMA_Enable(DMA1_Channel4);//¿ªÊ¼Ò»´ÎDMA´«Êä
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
    port_SPIx_clear_chip_select();
    delay_ms(110);
    port_SPIx_set_chip_select();
    dwt_softreset();
    reset_DW1000(); 	
    if (dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM)  == DWT_ERROR)
    {	
        led_on(LED_ALL);
        i=200;
        while (i--)
        {
            led_on(LED_ALL);
            delay_ms(100);
            led_off(LED_ALL);
            delay_ms(100);
        };
    }
    dwt_setleds(3) ; 
    dwt_configure(&config,DWT_LOADXTALTRIM);	
    eui64[0]=TAG_DDRL;
    eui64[1]=TAG_DDRH;
    dwt_seteui(eui64);
    dwt_setpanid(panID);
    dwt_setaddress16(ShortAddress16);	

    led_on(LED_ALL);
    port_EnableEXT_IRQ();
//	dwt_setrxantennadelay(RX_ANT_DLY);
//  dwt_settxantennadelay(TX_ANT_DLY);	

    tx_msg[2]=TAG_DDRL;         //²¹³¥Ö¡ºÍblinkÖ¡µØÖ·ÐÞ¸Ä
    tx_msg[3]=TAG_DDRH;
    tx_msg[8]=Frequency_parameter;

    location_compensate[5]=TAG_DDRL;
    location_compensate[6]=TAG_DDRH;
    dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);
    dwt_entersleepaftertx(1);    //×Ô¶¯Ë¯Ãß
    base_task_start();
    vTaskStartScheduler();
    while(1)
    {
       
    }
		
}

	


