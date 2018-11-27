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
#include "dma.h"
#include "deca_regs.h"
#include "hw_config.h"
#define TX_ANT_DLY 0
#define RX_ANT_DLY 0
#define BLINK_FRAME_SN_IDX 1
#define TX_DELAY_MS 1000
#define compensate_flag 0           /*²¹³¥±êÇ©±êÖ¾  1Îª²¹³¥ 0Îª¶¨Î»*/
#define TAG_DDRL 0xD0;         /*µÍÎ»±êÇ©µØÖ·*/
#define TAG_DDRH 0x01;         /*¸ßÎ»±êÇ©µØÖ·*/
#define Frequency_parameter 8 /*·¢ËÍÆµÂÊ µ¥Î»HZ*/

u8 SendBuff[130];
u8 USB_RxBuff[30];
static uint8 tx_msg[] = {0xC5, 0,0xc1, 0xc1, 0xee, 0xee, 'V', 'E', 0, 0};     /*²¹³¥Ö¡Ê*/
static uint8 location_compensate[] ={0xC5, 0,0xc1, 0xc1, 0xff, 0xff, 'V', 'E', 0, 0};    /*¶¨Î»Ö¡Ê*/
#define DUMMY_BUFFER_LEN 600
static uint8 dummy_buffer[DUMMY_BUFFER_LEN];

#define TAG_DDRL 0xD0;         /*µÍÎ»±êÇ©µØÖ·*/
#define TAG_DDRH 0x01;         /*¸ßÎ»±êÇ©µØÖ·*/
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



int main(void)
{
		int i = 0;
	  u16 ShortAddress16=0xaaaa;
	  u16 panID=0xdeca;
	  u8 source[8];
	  uint8 eui64[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};	
		uint8 t1[5];	
		uart_init(115200);	 //´®¿Ú³õÊ¼»¯Îª115200 	
		GPIO_Configuration();//³õÊ¼»¯ÓëLEDÁ¬½ÓµÄÓ²¼þ½Ó¿Ú
		SPI_Configuration();
		peripherals_init();
		Sleep(1000);
		Flash_Configuration();
		Sleep(200);
		USB_Config();
		MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,130);//DMA1Í¨µÀ4,ÍâÉèÎª´®¿Ú1,´æ´¢Æ÷ÎªSendBuff,³¤¶È130.  
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ  
		MYDMA_Enable(DMA1_Channel4);//¿ªÊ¼Ò»´ÎDMA´«Êä
		SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
		reset_DW1000(); 	
		port_SPIx_clear_chip_select();
		Sleep(110);
		port_SPIx_set_chip_select();
		if (dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM)  == DWT_ERROR)
		{	
			led_on(LED_ALL);
			i=200;
			while (i--)
			{
				led_on(LED_ALL);
				Sleep(100);
				led_off(LED_ALL);
				Sleep(100);
			};
		}
 	  dwt_setleds(3) ; 
    dwt_configure(&config,DWT_LOADXTALTRIM);	
    eui64[0]=TAG_DDRL;
	  eui64[1]=TAG_DDRH;
	  dwt_seteui(eui64);
		led_on(LED_ALL);
 	  port_EnableEXT_IRQ();
		if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
		{	
			led_on(LED_ALL);
			i=200;
			while (i--)
			{
				led_on(LED_ALL);
				Sleep(1000);
				led_off(LED_ALL);
			};
		}
    dwt_configure(&config,DWT_LOADXTALTRIM);	
//	  dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);     //¹ýÂËÖ¡£¬Ö»Ö§³Ö·¢ËÍ£¬²»½ÓÊÜ»ùÕ¾ÐÅÏ¢
    dwt_setpanid(panID);
    memcpy(eui64, &ShortAddress16, 2);
    dwt_seteui(eui64);
		dwt_geteui(source);
    dwt_setaddress16(ShortAddress16);	
//		dwt_setrxantennadelay(RX_ANT_DLY);
//    dwt_settxantennadelay(TX_ANT_DLY);	
		
		tx_msg[2]=TAG_DDRL;         //²¹³¥Ö¡ºÍblinkÖ¡µØÖ·ÐÞ¸Ä
		tx_msg[3]=TAG_DDRH;
		tx_msg[8]=Frequency_parameter;
		
		location_compensate[5]=TAG_DDRL;
		location_compensate[6]=TAG_DDRH;
		dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
		dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);
		dwt_entersleepaftertx(1);    //×Ô¶¯Ë¯Ãß
		while (1)
	  {
			 if(compensate_flag==0)
			 {
					dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); 
					dwt_writetxfctrl(sizeof(tx_msg), 0);     
					dwt_starttx(DWT_START_TX_IMMEDIATE);
					while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
					{
					};
					led_off(LED_ALL);
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
					Sleep(1000/Frequency_parameter);
					led_on(LED_ALL);
//					dwt_readtxtimestamp(t1);
//					memcpy(&tx_msg[4], t1, 5);
//					USB_TxWrite(&tx_msg[1],1);
					USB_TxWrite(tx_msg,7);
					tx_msg[1]++;
					dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
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
					Sleep(1000/Frequency_parameter);
					dwt_readtxtimestamp(t1);
					memcpy(&location_compensate[4], t1, 5);
					USB_TxWrite(&location_compensate[1],1);
					USB_TxWrite(&location_compensate[4],5);
					location_compensate[1]++;
			 }				 
	}
}

	


