//*****************************************************************************
//
// Copyright: 2020-2021, �Ϻ���ͨ��ѧ����ʵ����Ƽ�����III-A��ѧ��
// File name: ???.c
// Description: 
//    1.��ʾ��չʾ�������AIN2/PE1�˿�ʵ�ֵ������뵥��ADC����,����Ƶ��25Hz��
//    2.����ĸ��������ʾADC����ֵ[0-4095]��
//    3.�Ҳ������������ʾ��ѹֵ[0.00-3.30V]��
//    4.ע�⣺�����ѹֵ��Χ����Ϊ[0-3.3V]��������ջ��˿ڡ�
// Author:	�Ϻ���ͨ��ѧ����ʵ����Ƽ�����III-A��ѧ��
// Version: 1.0.0.20200924 
// Date��2020-09-24
// History��
//
//*****************************************************************************

//*****************************************************************************
//
// ͷ�ļ�
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // ��ַ�궨��
#include "inc/hw_types.h"         // �������ͺ궨�壬�Ĵ������ʺ���
#include "driverlib/debug.h"      // ������
#include "driverlib/gpio.h"       // ͨ��IO�ں궨��
#include "driverlib/pin_map.h"    // TM4Cϵ��MCU��Χ�豸�ܽź궨��
#include "driverlib/sysctl.h"	  // ϵͳ���ƶ���
#include "driverlib/systick.h"    // SysTick Driver ԭ��
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver ԭ��
#include "driverlib/adc.h"        // ��ADC�йصĶ��� 

#include "tm1638.h"               // �����TM1638оƬ�йصĺ���
#include "DAC6571.h"
//*****************************************************************************
//
// �궨��
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickƵ��Ϊ50Hz����ѭ����ʱ���55ms

#define V_T40ms	 2              // 40ms�����ʱ�����ֵ��2��20ms
#define V_T500ms 25              // 0.1s�����ʱ�����ֵ��5��20ms
//*****************************************************************************
//
// ����ԭ������
//
//*****************************************************************************
void GPIOInit(void);        // GPIO��ʼ��
void ADC0Init(void);         // ADC��ʼ��
void ADC1Init(void);
void SysTickInit(void);     // ����SysTick�ж� 
void DevicesInit(void);     // MCU������ʼ����ע���������������
uint32_t ADC0_Sample(void);  // ��ȡADC����ֵ
uint32_t ADC1_Sample(void);
//*****************************************************************************
//
// ��������
//
//*****************************************************************************

// �����ʱ������
uint8_t clock40ms = 0;
uint8_t clock100ms = 0;

// �����ʱ�������־
uint8_t clock40ms_flag = 0;
uint8_t	clock100ms_flag = 0; 
uint8_t clock_flag = 0;
// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
uint8_t digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};

// 8λС���� 1��  0��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2��3
uint8_t pnt = 0x11;


// DAC6571
uint32_t DAC6571_code = 0;
uint32_t DAC6571_voltage = 0;
uint8_t  DAC6571_flag = 0;


uint8_t key_code = 0;
uint8_t key_cnt = 0;
// 8��LEDָʾ��״̬��0��1��
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// ϵͳʱ��Ƶ�� 
uint32_t ui32SysClock;

// AIN2(PE1)  ADC����ֵ[0-4095]
uint32_t ui32ADC0Value,ui32ADC1Value;     

// AIN2��ѹֵ(��λΪ0.01V) [0.00-3.30]
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
// ������
//
//*****************************************************************************
 int main(void)
{

	DevicesInit();            //  MCU������ʼ��
	
	while (clock100ms < 3);   // ��ʱ>60ms,�ȴ�TM1638�ϵ����
	TM1638_Init();	          // ��ʼ��TM1638
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
              digit[1] = (Voltage_out / 100) % 10; // ��ʾ��ѹֵ��λ��
              digit[2] = (Voltage_out / 10) % 10;  // ��ʾ��ѹֵʮ��λ��
              digit[3] = Voltage_out % 10;         // ��ʾ��ѹֵ�ٷ�λ��  
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
		
		
        if (clock_flag == 1)        // ���40ms�붨ʱ�Ƿ�
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
              digit[1] = (Voltage_out / 100) % 10; // ��ʾ��ѹֵ��λ��
              digit[2] = (Voltage_out / 10) % 10;  // ��ʾ��ѹֵʮ��λ��
              digit[3] = Voltage_out % 10;         // ��ʾ��ѹֵ�ٷ�λ��  
						}
						
						
						digit[4] = Current_out / 1000; 	     // ��ʾADC����ֵǧλ��
						digit[5] = Current_out / 100 % 10; 	 // ��ʾADC����ֵ��λ��
						digit[6] = Current_out / 10 % 10; 	 // ��ʾADC����ֵʮλ��
						digit[7] = Current_out % 10;           // ��ʾADC����ֵ��λ��						
						
						
						
						
//      digit[4] = ui32ADC0Value / 1000; 	     // ��ʾADC����ֵǧλ��
//			digit[5] = ui32ADC0Value / 100 % 10; 	 // ��ʾADC����ֵ��λ��
//			digit[6] = ui32ADC0Value / 10 % 10; 	 // ��ʾADC����ֵʮλ��
//			digit[7] = ui32ADC0Value % 10;           // ��ʾADC����ֵ��λ��
		
					}						
        }
	}
	
}

//*****************************************************************************
//
// ����ԭ�ͣ�void GPIOInit(void)
// �������ܣ�GPIO��ʼ����ʹ��PortK������PK4,PK5Ϊ�����ʹ��PortM������PM0Ϊ�����
//          ��PK4����TM1638��STB��PK5����TM1638��DIO��PM0����TM1638��CLK��
// ������������
// ��������ֵ����
//
//*****************************************************************************
void GPIOInit(void)
{
	//����TM1638оƬ�ܽ�
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// ʹ�ܶ˿� K	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// �ȴ��˿� K׼�����		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// ʹ�ܶ˿� M	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// �ȴ��˿� M׼�����		
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);				// ???? L	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};		// ???? L????		

   // ���ö˿� K�ĵ�4,5λ��PK4,PK5��Ϊ�������		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// ���ö˿� M�ĵ�0λ��PM0��Ϊ�������   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);	
  GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);

}


//*****************************************************************************
//
// ����ԭ�ͣ�void ADCInit(void)
// �������ܣ�ADC0��ʼ����
//           ѡ��AIN2/PE1��ΪADC��������˿ڣ����õ��˿����뵥�β�����ʽ
// ������������
// ��������ֵ����
//
//*****************************************************************************
void ADC0Init(void)
{	   
    // ʹ��ADC0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // ʹ��AIN2/PE1�˿���ΪADC���룬ʹ�ܶ˿�E1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    // ���β���(sample sequence 3��ʽ)
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

    // ʹ�ܵ��β�����ʽ(sample sequence 3)
    ADCSequenceEnable(ADC0_BASE, 3);

    // �ڲ���ǰ����������ж�״̬��־
    ADCIntClear(ADC0_BASE, 3);		
}

void ADC1Init(void)
{	   
    // ʹ��ADC0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ʹ��AIN2,0/PE2,3�˿���ΪADC���룬ʹ�ܶ˿�E1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    // ���β���(sample sequence 3��ʽ)
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

    // ʹ�ܵ��β�����ʽ(sample sequence 3)
    ADCSequenceEnable(ADC1_BASE, 3);

    // �ڲ���ǰ����������ж�״̬��־
    ADCIntClear(ADC1_BASE, 3);		
}
//*****************************************************************************
//
// ����ԭ�ͣ�uint32_t ADC_Sample(void)
// �������ܣ���ȡADC����ֵ��
// ������������
// ��������ֵ��ADC����ֵ[0-4095]
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
	
    // ����ADC����
    ADCProcessorTrigger(ADC0_BASE, 3);

    // �ȴ�����ת�����
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }

    // ���ADC�жϱ�־
    ADCIntClear(ADC0_BASE, 3);

    // ��ȡADC����ֵ
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
	
    // ����ADC����
    ADCProcessorTrigger(ADC1_BASE, 3);

    // �ȴ�����ת�����
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }

    // ���ADC�жϱ�־
    ADCIntClear(ADC1_BASE, 3);

    // ��ȡADC����ֵ
    ADCSequenceDataGet(ADC1_BASE, 3, pui32ADC1Value);

    return pui32ADC1Value[0];
}
//*****************************************************************************
// 
// ����ԭ�ͣ�SysTickInit(void)
// �������ܣ�����SysTick�ж�
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // ������������,��ʱ����20ms
    SysTickEnable();  			// SysTickʹ��
    SysTickIntEnable();			// SysTick�ж�����
}

//*****************************************************************************
// 
// ����ԭ�ͣ�DevicesInit(void)
// �������ܣ�CU������ʼ��������ϵͳʱ�����á�GPIO��ʼ����SysTick�ж�����
// ������������
// ��������ֵ����
//
//*****************************************************************************
void DevicesInit(void)
{
	// ʹ���ⲿ25MHz��ʱ��Դ������PLL��Ȼ���ƵΪ20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | 
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 
	                                   20000000);

  GPIOInit();             // GPIO��ʼ��
  ADC0Init();              // ADC��ʼ��
	ADC1Init();
  SysTickInit();          // ����SysTick�ж�
  IntMasterEnable();	  // ���ж�����
}

//*****************************************************************************
// 
// ����ԭ�ͣ�void SysTick_Handler(void)
// �������ܣ�SysTick�жϷ������
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTick_Handler(void)       // ��ʱ����Ϊ20ms
{
 clock_flag=1;
	// 40ms������ʱ������
	if (++clock40ms >= V_T40ms)
	{
		clock40ms_flag = 1; // ��40ms��ʱ�������־��1
		clock40ms = 0;
	}

    
	// 0.1������ʱ������
	if (++clock100ms >= V_T500ms)
	{
		clock100ms_flag = 1; // ��0.1�뵽ʱ�������־��1
		clock100ms = 0;
	}
	
	// ˢ��ȫ������ܺ�LED
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
