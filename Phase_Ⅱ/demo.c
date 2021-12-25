//*****************************************************************************
//
// Copyright: 2020-2021, ÉÏº£½»Í¨´óÑ§¹¤³ÌÊµ¼ùÓë¿Æ¼¼´´ĞÂIII-A½ÌÑ§×é
// File name: ???.c
// Description: 
//    1.¸ÃÊ¾ÀıÕ¹Ê¾ÈçºÎÀûÓÃAIN2/PE1¶Ë¿ÚÊµÏÖµ¥¶ËÊäÈëµ¥´ÎADC²ÉÑù,²ÉÑùÆµÂÊ25Hz£»
//    2.×ó²àËÄ¸öÊıÂë¹ÜÏÔÊ¾ADC²ÉÑùÖµ[0-4095]£»
//    3.ÓÒ²àÈı¸öÊıÂë¹ÜÏÔÊ¾µçÑ¹Öµ[0.00-3.30V]£»
//    4.×¢Òâ£ºÊäÈëµçÑ¹Öµ·¶Î§±ØĞëÎª[0-3.3V]£¬·ñÔò»áÉÕ»µ¶Ë¿Ú¡£
// Author:	ÉÏº£½»Í¨´óÑ§¹¤³ÌÊµ¼ùÓë¿Æ¼¼´´ĞÂIII-A½ÌÑ§×é
// Version: 1.0.0.20200924 
// Date£º2020-09-24
// History£º
//
//*****************************************************************************

//*****************************************************************************
//
// Í·ÎÄ¼ş
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // »ùÖ·ºê¶¨Òå
#include "inc/hw_types.h"         // Êı¾İÀàĞÍºê¶¨Òå£¬¼Ä´æÆ÷·ÃÎÊº¯Êı
#include "driverlib/debug.h"      // µ÷ÊÔÓÃ
#include "driverlib/gpio.h"       // Í¨ÓÃIO¿Úºê¶¨Òå
#include "driverlib/pin_map.h"    // TM4CÏµÁĞMCUÍâÎ§Éè±¸¹Ü½Åºê¶¨Òå
#include "driverlib/sysctl.h"	  // ÏµÍ³¿ØÖÆ¶¨Òå
#include "driverlib/systick.h"    // SysTick Driver Ô­ĞÍ
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver Ô­ĞÍ
#include "driverlib/adc.h"        // ÓëADCÓĞ¹ØµÄ¶¨Òå 

#include "tm1638.h"               // Óë¿ØÖÆTM1638Ğ¾Æ¬ÓĞ¹ØµÄº¯Êı
#include "DAC6571.h"
//*****************************************************************************
//
// ºê¶¨Òå
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickÆµÂÊÎª50Hz£¬¼´Ñ­»·¶¨Ê±ÖÜÆ55ms

#define V_T40ms	 2              // 40msÈí¼ş¶¨Ê±Æ÷Òç³öÖµ£¬2¸ö20ms
#define V_T500ms 25              // 0.1sÈí¼ş¶¨Ê±Æ÷Òç³öÖµ£¬5¸ö20ms
//*****************************************************************************
//
// º¯ÊıÔ­ĞÍÉùÃ÷
//
//*****************************************************************************
void GPIOInit(void);        // GPIO³õÊ¼»¯
void ADC0Init(void);         // ADC³õÊ¼»¯
void ADC1Init(void);
void SysTickInit(void);     // ÉèÖÃSysTickÖĞ¶Ï 
void DevicesInit(void);     // MCUÆ÷¼ş³õÊ¼»¯£¬×¢£º»áµ÷ÓÃÉÏÊöº¯Êı
uint32_t ADC0_Sample(void);  // »ñÈ¡ADC²ÉÑùÖµ
uint32_t ADC1_Sample(void);
//*****************************************************************************
//
// ±äÁ¿¶¨Òå
//
//*****************************************************************************

// Èí¼ş¶¨Ê±Æ÷¼ÆÊı
uint8_t clock40ms = 0;
uint8_t clock100ms = 0;

// Èí¼ş¶¨Ê±Æ÷Òç³ö±êÖ¾
uint8_t clock40ms_flag = 0;
uint8_t	clock100ms_flag = 0; 
uint8_t clock_flag = 0;
// 8Î»ÊıÂë¹ÜÏÔÊ¾µÄÊı×Ö»ò×ÖÄ¸·ûºÅ
// ×¢£º°åÉÏÊıÂëÎ»´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª4¡¢5¡¢6¡¢7¡¢0¡¢1¡¢2¡¢3
uint8_t digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};

// 8Î»Ğ¡Êıµã 1ÁÁ  0Ãğ
// ×¢£º°åÉÏÊıÂëÎ»Ğ¡Êıµã´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª4¡¢5¡¢6¡¢7¡¢0¡¢1¡¢2¡¢3
uint8_t pnt = 0x11;


// DAC6571
uint32_t DAC6571_code = 0;
uint32_t DAC6571_voltage = 0;
uint8_t  DAC6571_flag = 0;


uint8_t key_code = 0;
uint8_t key_cnt = 0;
// 8¸öLEDÖ¸Ê¾µÆ×´Ì¬£¬0Ãğ£¬1ÁÁ
// ×¢£º°åÉÏÖ¸Ê¾µÆ´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª7¡¢6¡¢5¡¢4¡¢3¡¢2¡¢1¡¢0
//     ¶ÔÓ¦Ôª¼şLED8¡¢LED7¡¢LED6¡¢LED5¡¢LED4¡¢LED3¡¢LED2¡¢LED1
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// ÏµÍ³Ê±ÖÓÆµÂÊ 
uint32_t ui32SysClock;

// AIN2(PE1)  ADC²ÉÑùÖµ[0-4095]
uint32_t ui32ADC0Value,ui32ADC1Value;     

// AIN2µçÑ¹Öµ(µ¥Î»Îª0.01V) [0.00-3.30]
uint32_t ui32ADC0Voltage,ui32ADC1Voltage; 
uint8_t i=0,j=0;
float Voltage=0,Current=0;
uint16_t Voltage_out=0,Current_out=0;
float Voltage_before=0,Voltage_now=0;
float Current_before=0,Current_now=0;
float q=0.05;
uint16_t n=50;
uint32_t displayVAorDA=0;
bool display_flag=true;
//*****************************************************************************
//
// Ö÷³ÌĞò
//
//*****************************************************************************
 int main(void)
{

	DevicesInit();            //  MCUÆ÷¼ş³õÊ¼»¯
	
	while (clock100ms < 3);   // ÑÓÊ±>60ms,µÈ´ıTM1638ÉÏµçÍê³É
	TM1638_Init();	          // ³õÊ¼»¯TM1638
	DAC6571_flag = 1;
	while (1)
	{				
        if (DAC6571_flag == 1)   // ??DAC??????
		{
			DAC6571_flag = 0;
			//////////////////////////////
			////change here///////////////
			//////////////////////////////
			DAC6571_voltage=(int)(DAC6571_code*1009/3300-18);
			if(displayVAorDA==true)
						{
						  digit[0] = (Voltage_out /1000) %10;
              digit[1] = (Voltage_out / 100) % 10; // ÏÔÊ¾µçÑ¹Öµ¸öÎ»Êı
              digit[2] = (Voltage_out / 10) % 10;  // ÏÔÊ¾µçÑ¹ÖµÊ®·ÖÎ»Êı
              digit[3] = Voltage_out % 10;         // ÏÔÊ¾µçÑ¹Öµ°Ù·ÖÎ»Êı  
						}
						else
						{
			digit[0] = DAC6571_code / 1000 ; 	  // ?????
			digit[1] = DAC6571_code / 100 % 10;   // ?????
			digit[2] = DAC6571_code / 10 % 10;    // ?????
      digit[3] = DAC6571_code % 10;         // ?????
						}
      DAC6571_Fastmode_Operation(DAC6571_voltage);
		}
		
		
        if (clock_flag == 1)        // ¼ì²é40msÃë¶¨Ê±ÊÇ·ñµ½
        {
            clock_flag = 0;
					ui32ADC0Value = ADC0_Sample();
					ui32ADC1Value = ADC1_Sample();
					
					Voltage_now=q*ui32ADC0Value+(1-q)*Voltage_before;
					Voltage+=Voltage_now;
					Voltage_before=Voltage_now;
					
					Current_now=q*ui32ADC1Value+(1-q)*Current_before;
					Current+=Current_now;
					Current_before=Current_now;
					
            i++;
					if (i==n)
					{
						i=0;
           
			      Voltage_out=Voltage*6679/n/4095+3;
						Voltage=0;
						Current_out=Current*3289/n/4095-12;
						Current=0;
						
						if(displayVAorDA==true)
						{
						  digit[0] = (Voltage_out /1000) %10;
              digit[1] = (Voltage_out / 100) % 10; // ÏÔÊ¾µçÑ¹Öµ¸öÎ»Êı
              digit[2] = (Voltage_out / 10) % 10;  // ÏÔÊ¾µçÑ¹ÖµÊ®·ÖÎ»Êı
              digit[3] = Voltage_out % 10;         // ÏÔÊ¾µçÑ¹Öµ°Ù·ÖÎ»Êı  
						}
						
						
						digit[4] = Current_out / 1000; 	     // ÏÔÊ¾ADC²ÉÑùÖµÇ§Î»Êı
						digit[5] = Current_out / 100 % 10; 	 // ÏÔÊ¾ADC²ÉÑùÖµ°ÙÎ»Êı
						digit[6] = Current_out / 10 % 10; 	 // ÏÔÊ¾ADC²ÉÑùÖµÊ®Î»Êı
						digit[7] = Current_out % 10;           // ÏÔÊ¾ADC²ÉÑùÖµ¸öÎ»Êı						
						
						
						
						
//      digit[4] = ui32ADC0Value / 1000; 	     // ÏÔÊ¾ADC²ÉÑùÖµÇ§Î»Êı
//			digit[5] = ui32ADC0Value / 100 % 10; 	 // ÏÔÊ¾ADC²ÉÑùÖµ°ÙÎ»Êı
//			digit[6] = ui32ADC0Value / 10 % 10; 	 // ÏÔÊ¾ADC²ÉÑùÖµÊ®Î»Êı
//			digit[7] = ui32ADC0Value % 10;           // ÏÔÊ¾ADC²ÉÑùÖµ¸öÎ»Êı
		
					}						
        }
	}
	
}

//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ£ºvoid GPIOInit(void)
// º¯Êı¹¦ÄÜ£ºGPIO³õÊ¼»¯¡£Ê¹ÄÜPortK£¬ÉèÖÃPK4,PK5ÎªÊä³ö£»Ê¹ÄÜPortM£¬ÉèÖÃPM0ÎªÊä³ö¡£
//          £¨PK4Á¬½ÓTM1638µÄSTB£¬PK5Á¬½ÓTM1638µÄDIO£¬PM0Á¬½ÓTM1638µÄCLK£©
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void GPIOInit(void)
{
	//ÅäÖÃTM1638Ğ¾Æ¬¹Ü½Å
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// Ê¹ÄÜ¶Ë¿Ú K	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// µÈ´ı¶Ë¿Ú K×¼±¸Íê±Ï		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// Ê¹ÄÜ¶Ë¿Ú M	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// µÈ´ı¶Ë¿Ú M×¼±¸Íê±Ï		
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);				// ???? L	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};		// ???? L????		

   // ÉèÖÃ¶Ë¿Ú KµÄµÚ4,5Î»£¨PK4,PK5£©ÎªÊä³öÒı½Å		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// ÉèÖÃ¶Ë¿Ú MµÄµÚ0Î»£¨PM0£©ÎªÊä³öÒı½Å   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);	
  GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);

}


//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ£ºvoid ADCInit(void)
// º¯Êı¹¦ÄÜ£ºADC0³õÊ¼»¯¡£
//           Ñ¡ÔñAIN2/PE1×÷ÎªADC²ÉÑùÊäÈë¶Ë¿Ú£¬²ÉÓÃµ¥¶Ë¿ÚÊäÈëµ¥´Î²ÉÑù·½Ê½
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void ADC0Init(void)
{	   
    // Ê¹ÄÜADC0Ä£¿é
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Ê¹ÓÃAIN2/PE1¶Ë¿Ú×÷ÎªADCÊäÈë£¬Ê¹ÄÜ¶Ë¿ÚE1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    // µ¥´Î²ÉÑù(sample sequence 3·½Ê½)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Ê¹ÄÜµ¥´Î²ÉÑù·½Ê½(sample sequence 3)
    ADCSequenceEnable(ADC0_BASE, 3);

    // ÔÚ²ÉÑùÇ°£¬±ØĞëÇå³ıÖĞ¶Ï×´Ì¬±êÖ¾
    ADCIntClear(ADC0_BASE, 3);		
}

void ADC1Init(void)
{	   
    // Ê¹ÄÜADC0Ä£¿é
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // Ê¹ÓÃAIN2,0/PE2,3¶Ë¿Ú×÷ÎªADCÊäÈë£¬Ê¹ÄÜ¶Ë¿ÚE1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    // µ¥´Î²ÉÑù(sample sequence 3·½Ê½)
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Ê¹ÄÜµ¥´Î²ÉÑù·½Ê½(sample sequence 3)
    ADCSequenceEnable(ADC1_BASE, 3);

    // ÔÚ²ÉÑùÇ°£¬±ØĞëÇå³ıÖĞ¶Ï×´Ì¬±êÖ¾
    ADCIntClear(ADC1_BASE, 3);		
}
//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ£ºuint32_t ADC_Sample(void)
// º¯Êı¹¦ÄÜ£º»ñÈ¡ADC²ÉÑùÖµ¡£
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºADC²ÉÑùÖµ[0-4095]
//
//*****************************************************************************
uint32_t ADC0_Sample(void)
{

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //
    uint32_t pui32ADC0Value[1];
	
    // ´¥·¢ADC²ÉÑù
    ADCProcessorTrigger(ADC0_BASE, 3);

    // µÈ´ı²ÉÑù×ª»»Íê³É
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }

    // Çå³ıADCÖĞ¶Ï±êÖ¾
    ADCIntClear(ADC0_BASE, 3);

    // ¶ÁÈ¡ADC²ÉÑùÖµ
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

    return pui32ADC0Value[0];
}

uint32_t ADC1_Sample(void)
{

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //
    uint32_t pui32ADC1Value[1];
	
    // ´¥·¢ADC²ÉÑù
    ADCProcessorTrigger(ADC1_BASE, 3);

    // µÈ´ı²ÉÑù×ª»»Íê³É
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }

    // Çå³ıADCÖĞ¶Ï±êÖ¾
    ADCIntClear(ADC1_BASE, 3);

    // ¶ÁÈ¡ADC²ÉÑùÖµ
    ADCSequenceDataGet(ADC1_BASE, 3, pui32ADC1Value);

    return pui32ADC1Value[0];
}
//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºSysTickInit(void)
// º¯Êı¹¦ÄÜ£ºÉèÖÃSysTickÖĞ¶Ï
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // ÉèÖÃĞÄÌø½ÚÅÄ,¶¨Ê±ÖÜÆÚ20ms
    SysTickEnable();  			// SysTickÊ¹ÄÜ
    SysTickIntEnable();			// SysTickÖĞ¶ÏÔÊĞí
}

//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºDevicesInit(void)
// º¯Êı¹¦ÄÜ£ºCUÆ÷¼ş³õÊ¼»¯£¬°üÀ¨ÏµÍ³Ê±ÖÓÉèÖÃ¡¢GPIO³õÊ¼»¯ºÍSysTickÖĞ¶ÏÉèÖÃ
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void DevicesInit(void)
{
	// Ê¹ÓÃÍâ²¿25MHzÖ÷Ê±ÖÓÔ´£¬¾­¹ıPLL£¬È»ºó·ÖÆµÎª20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | 
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 
	                                   20000000);

  GPIOInit();             // GPIO³õÊ¼»¯
  ADC0Init();              // ADC³õÊ¼»¯
	ADC1Init();
  SysTickInit();          // ÉèÖÃSysTickÖĞ¶Ï
  IntMasterEnable();	  // ×ÜÖĞ¶ÏÔÊĞí
}

//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºvoid SysTick_Handler(void)
// º¯Êı¹¦ÄÜ£ºSysTickÖĞ¶Ï·şÎñ³ÌĞò
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void SysTick_Handler(void)       // ¶¨Ê±ÖÜÆÚÎª20ms
{
 clock_flag=1;
	// 40msÃëÖÓÈí¶¨Ê±Æ÷¼ÆÊı
	if (++clock40ms >= V_T40ms)
	{
		clock40ms_flag = 1; // µ±40msµ½Ê±£¬Òç³ö±êÖ¾ÖÃ1
		clock40ms = 0;
	}

    
	// 0.1ÃëÖÓÈí¶¨Ê±Æ÷¼ÆÊı
	if (++clock100ms >= V_T500ms)
	{
		clock100ms_flag = 1; // µ±0.1Ãëµ½Ê±£¬Òç³ö±êÖ¾ÖÃ1
		clock100ms = 0;
	}
	
	// Ë¢ĞÂÈ«²¿ÊıÂë¹ÜºÍLED
	TM1638_RefreshDIGIandLED(digit, pnt, led);
	
	key_code = TM1638_Readkeyboard();
	
		if (key_code != 0)
	{
		if (key_cnt < 4) key_cnt++;   // ????,4*20ms
		else if (key_cnt == 4)
		{
			
            switch(key_code)
            {
                case 1:     // ?100
                    if (DAC6571_code < DAC6571_code_max - 100) 
				    {
					     DAC6571_code += 100;
					     DAC6571_flag = 1;
				    }
                    break;
                case 4:    // ?100
                    if (DAC6571_code > 100) 
				    {
					    DAC6571_code -= 100;
					    DAC6571_flag = 1;
				    }
                    break;
                case 2:    // ?10
                   if (DAC6571_code < DAC6571_code_max - 10) 
				    {
					     DAC6571_code += 10;
					     DAC6571_flag = 1;
				    }                    
                    break;
                case 5:    // ?10
                   if (DAC6571_code >= 10) 
				    {
					    DAC6571_code -= 10;
					    DAC6571_flag = 1;
				    }
                    break;
                case 3:    // ?1
                   if (DAC6571_code < DAC6571_code_max - 1) 
				    {
					     DAC6571_code += 1;
					     DAC6571_flag = 1;
				    }
                    break;
                case 6:    // ?1
                   if (DAC6571_code >= 1) 
				    {
					    DAC6571_code -= 1;
					    DAC6571_flag = 1;
				    }
                    break;
                case 9:    // ?1

				    {
							if(displayVAorDA==true)
								displayVAorDA=false;
							else
								displayVAorDA=true;
					    DAC6571_flag = 1;
				    }
								    break;
								
								default:
                    break;
            }
            
			key_cnt = 5;   // ??????,?????
		}
	}
	else key_cnt = 0;
       
	//digit[5] = key_code;   // ???

	
	

}
