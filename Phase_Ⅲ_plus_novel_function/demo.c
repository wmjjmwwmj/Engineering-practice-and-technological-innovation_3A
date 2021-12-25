//*****************************************************************************
//
// Copyright: 2020-2021, �Ϻ���ͨ��ѧ����ʵ����Ƽ�����III-A��ѧ��
// File name: adc_demo.c
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
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/debug.h"      // ������
#include "driverlib/gpio.h"       // ͨ��IO�ں궨��
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"    // TM4Cϵ��MCU��Χ�豸�ܽź궨��
#include "driverlib/sysctl.h"	  // ϵͳ���ƶ���
#include "driverlib/systick.h"    // SysTick Driver ԭ��
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver ԭ��
#include "driverlib/adc.h"        // ��ADC�йصĶ��� 
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "tm1638.h"               // �����TM1638оƬ�йصĺ���
#include "DAC6571.h"
#include "inc/tm4c1294ncpdt.h"   // TM4C1294NCPDT Register Definitions
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include <ctype.h>               // Character handling functions
#include <string.h>              // C Strings
#include <stdio.h>               // C library to perform Input/Output operations
#include <stdlib.h>              // C Standard General Utilities Library

//*****************************************************************************
// �궨��
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifndef PART_TM4C1294NCPDT
#define PART_TM4C1294NCPDT // pin_map.h ??
#endif


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickƵ��Ϊ50Hz����ѭ����ʱ��20ms

#define V_T40ms	 2              // 40ms�����ʱ�����ֵ��2��20ms
#define V_T100ms 5              // 0.1s�����ʱ�����ֵ��5��20ms
#define V_T1000ms 50              // 1s�����ʱ�����ֵ��50��20ms
const uint8_t CMD_RX_BUF_MAX_SIZE = 60; // ???????????
#define CMD_TX_BUF_MAX_SIZE 60          // ???????????

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
void ADC0_Sample(void);  // ��ȡADC����ֵ
uint32_t ADC1_Sample(void);

void Timer4Init(void);
void TIMER4A_Handler(void);
void display_seg(void);
void S800_UART_Init(void);
void UART0_Handler(void);
void UART_clock(void); 
//void UARTStringPut(uint8_t *cMessage);
void UARTStringPut(uint32_t ui32Base, const char *cMessage); 

//*****************************************************************************
//
// ��������
//
//*****************************************************************************

// �����ʱ������
uint8_t clock40ms = 0;
uint8_t clock100ms = 0;
uint8_t clock1000ms = 0;
// �����ʱ�������־
uint8_t clock40ms_flag = 0;
uint8_t	clock100ms_flag = 0; 
uint8_t	clock1000ms_flag = 0; 
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
uint32_t  ui32ADC0Value1,ui32ADC0Value0,ui32ADC1Value;
//[0]-> Voltage ; [1]-> Current1 ; ADC1-> Current2
// AIN2��ѹֵ(��λΪ0.01V) [0.00-3.30]
uint32_t ui32ADC0Voltage; 
uint8_t i=0,j=0;
float Voltage=0,Current1=0,Current2=0;
uint32_t Voltage_out=0,Current1_out=0,Current2_out=0;
uint32_t power=0;
float Voltage_before=0,Voltage_now=0;
float Current1_before=0,Current1_now=0,Current2_before=0,Current2_now=0;
float q=0.05;
uint16_t n=50;

bool display_flag=false;
uint16_t display_mode=1;

uint32_t f=0;

uint16_t step_flag=0;
bool counting_flag=0;
uint32_t counting_num=0;
uint32_t timer4A_cnt=0;
bool special_mode=0;
uint32_t read_sw1,read_sw2,count=0;
//beep�Ĳ����Լ����Բ���
uint16_t beep_flag=0;
uint16_t beep_count=0;
uint16_t bflag=1;
/**
 * @brief ??????????
 * 
 */
struct cmd_Rx_buf_t
{
    volatile uint8_t size;                                // ?????????
    const uint8_t max_size;                               // ???????
    volatile unsigned char data[CMD_RX_BUF_MAX_SIZE + 1]; // ?????
    volatile bool WriteEnable;                            // True-???;False-???????????????
} cmd_Rx_buf = {0, CMD_RX_BUF_MAX_SIZE, {'\0'}, true};

/**
 * @brief ??????????
 * 
 */
struct cmd_Tx_buf_t
{
    volatile uint8_t size;                                // ?????????
    const uint8_t max_size;                               // ???????
    volatile unsigned char data[CMD_TX_BUF_MAX_SIZE + 1]; // ??
    volatile uint8_t next_trans_index;                    // ????????????. ???size?,???????.
    volatile bool WriteEnable;                            // True-???;False-???????????????
} cmd_Tx_buf = {0, CMD_TX_BUF_MAX_SIZE, {'\0'}, 0, true};

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
    if(clock1000ms_flag==1)
		{
			clock1000ms_flag=0;
			//f=timer4A_cnt*10.138-0.4633;
			
			f=timer4A_cnt*10.069-0.925;
			if(f>47652&&f<579342)
				f=f-300;
			timer4A_cnt=0;
		}
		if(Current1_out>1000)
		{
			beep_flag=1;
		}
		else
		{			
   		beep_flag=0;
		}
        if (clock_flag == 1)        // ���20ms�붨ʱ�Ƿ�
        {
            clock_flag = 0;
					ADC0_Sample();
					ui32ADC1Value = ADC1_Sample();
					
					Voltage_now=q*ui32ADC0Value1+(1-q)*Voltage_before;
					Voltage+=Voltage_now;
					Voltage_before=Voltage_now;
					
					Current1_now=q*ui32ADC0Value0+(1-q)*Current1_before;
					Current1+=Current1_now;
					Current1_before=Current1_now;
					
					Current2_now=q*ui32ADC1Value+(1-q)*Current2_before;
					Current2+=Current2_now;
					Current2_before=Current2_now;
					
            i++;
					if(i%4==0)
					{
						///////DAC output///////
						//DAC6571_code= 300;
						if(!special_mode)
						{
						DAC6571_code=(Current1_out+Current2_out)/2;
							//	DAC6571_code=900;
							led[7]=0;
						}
						else
						{
							DAC6571_code=(Current1_out+Current2_out)*2/3;
							led[7]=1;
						}
						DAC6571_voltage=(int)( DAC6571_code *1024/3300-29);///////
						display_seg();
            DAC6571_Fastmode_Operation(DAC6571_voltage);
					}
					
					if (i==n)
					{
						i=0;
           //1.40250845
			      Voltage_out=Voltage*1.85495/n-752.8181;
						Voltage=0;
						Current1_out=Current1*0.5356/n-11.418;
						Current1=0;
						Current2_out=Current2*0.7975524/n+18.878;
						Current2=0;
						
						display_seg();
					}						
        }
				
	UART_clock();
	
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

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);				// ʹ�ܶ˿� F	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){};		// �ȴ��˿� K׼�����		
   // ���ö˿� K�ĵ�4,5λ��PK4,PK5��Ϊ�������		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// ���ö˿� M�ĵ�0λ��PM0��Ϊ�������   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);	
  GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);
  
		GPIOPinConfigure(GPIO_PM4_T4CCP0);
		GPIOPinTypeTimer(GPIO_PORTM_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

		GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);			//Set PF0~3 as Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);      //Enable PortJ 
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){};   //Wait for the GPIO moduleJ ready GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); //????????
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0); //??J0????
}



void UARTStringPut(uint32_t ui32Base, const char *cMessage)
{
    while (*cMessage != '\0')
        UARTCharPut(ui32Base, *(cMessage++));
}
//void UARTStringPut(uint8_t *cMessage)
//{
//	while(*cMessage!='\0')
//		UARTCharPut(UART0_BASE,*(cMessage++));
//}
void UARTStringPutNonBlocking( const char *cMessage)
{
	while(*cMessage!='\0')// �ַ�'\0'��C������i�ַ����Զ���ӵĽ����ַ���ASCII��Ϊ00H��
	UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}
void UARTStringGetNonBlocking(char *msg)
{
	while(UARTCharsAvail(UART0_BASE))
	{
		*msg++ = UARTCharGetNonBlocking(UART0_BASE);
	}
	*msg = '\0';		//���ַ���������
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
	  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    // ���β���(sample sequence 1��ʽ)
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

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
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1 );
		ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2  | ADC_CTL_IE |
                             ADC_CTL_END);
		//ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);

    // ʹ�ܵ��β�����ʽ(sample sequence 1)
    ADCSequenceEnable(ADC0_BASE, 1);

    // �ڲ���ǰ����������ж�״̬��־
    ADCIntClear(ADC0_BASE, 1);		
}

void ADC1Init(void)
{	   
    // ʹ��ADC0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ʹ��AIN2,0/PE2,3�˿���ΪADC���룬ʹ�ܶ˿�E1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // ���β���(sample sequence 3��ʽ)
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
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
void ADC0_Sample(void)
{

    
    uint32_t pui32ADC0Value[4];
	
    // ����ADC����
    ADCProcessorTrigger(ADC0_BASE, 1);

    // �ȴ�����ת�����
    while(!ADCIntStatus(ADC0_BASE, 1, false))
    {
    }

    // ���ADC�жϱ�־
    ADCIntClear(ADC0_BASE, 1);

    // ��ȡADC����ֵ
    ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

    ui32ADC0Value1 = pui32ADC0Value[1];
		ui32ADC0Value0 = pui32ADC0Value[0];
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


void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

// Configure the UART character format: 115,200, 8-N-1. FIFO����ˮƽ
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	
	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);	//���崥��ˮƽΪ14�ֽ�
  IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX); // ??UART0 RX,RT,TX??
  UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);     // Sets the FIFO level at which interrupts are generated.
  UARTStringPut(UART0_BASE, (const char *)"\r\nHello!\r\n");

  //UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void Timer4Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4))
    {
    }
	
	TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP);
	TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

  TimerMatchSet(TIMER4_BASE, TIMER_A, 10-1);
	TimerIntRegister(TIMER4_BASE,TIMER_A,TIMER4A_Handler);
	
	TimerIntEnable(TIMER4_BASE,TIMER_CAPA_MATCH);
	IntEnable(INT_TIMER4A);
	
	//IntPrioritySet(INT_TIMER0A, 0);
         
    TimerEnable(TIMER4_BASE,TIMER_A);

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
	                                   120000000);

  GPIOInit();             // GPIO��ʼ��
  ADC0Init();  
  ADC1Init();	// ADC��ʼ��
	Timer4Init();
	S800_UART_Init();
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
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // ��0.1�뵽ʱ�������־��1
		clock100ms = 0;
	}
	
	// 0.1������ʱ������
	if (++clock1000ms >= V_T1000ms)
	{
		clock1000ms_flag = 1; // ��0.1�뵽ʱ�������־��1
		clock1000ms = 0;
	}
	
	// ˢ��ȫ������ܺ�LED
	TM1638_RefreshDIGIandLED(digit, pnt, led);
	read_sw1= GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) ;
 read_sw2= GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) ;
 
 if(read_sw1==0&&read_sw2==0)
 {
  if(count<5)
  {
   count++;//����ȥ������ʱ80s
  }
  else
  {
     display_mode=3;
     count=0;
  }
 }
 else
 {
  if(read_sw1==0)
  {
   if(count<5)
   {
    count++;//����ȥ������ʱ80s
   }
   else
   {
     display_mode=1;
     count=0;
	
   }
  }
  else
  {
  if(read_sw2==0)
  {
   if(count<5)
   {
    count++;//����ȥ������ʱ80s
   }
   else
   {
     display_mode=2;
     count=0;
		
   }
  }
  }
 }

  if(beep_flag==1)
		{
		     bflag=~GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_2);
			   GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2,bflag);
			   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,~bflag);
		}
		if(beep_flag==0)
		{
		   	GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0x0);
			  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
		}

}



void TIMER4A_Handler(void)
{
	unsigned long ulstatus; 
    ulstatus = TimerIntStatus(TIMER4_BASE, TIMER_CAPA_MATCH);
    TimerIntClear(TIMER4_BASE, ulstatus);
    //TimerIntClear(TIMER0_BASE, TIMER_CAPA_MATCH);
	//ulstatus = TimerIntStatus(TIMER4_BASE, TIMER_CAPA_EVENT);
    //TimerIntClear(TIMER4_BASE, ulstatus);
    timer4A_cnt++;
	  
	  
	//TimerDisable(TIMER4_BASE,TIMER_A);
	  //TimerEnable(TIMER4_BASE,TIMER_A);
	
}

void display_seg(void)
{
	  if(display_mode ==2)
		{
			pnt = 0x11;
			digit[0] = Current2_out / 1000 %10; 	     // ��ʾADC����ֵǧλ��
			digit[1] = Current2_out / 100 % 10; 	 // ��ʾADC����ֵ��λ��
			digit[2] = Current2_out / 10 % 10; 	 // ��ʾADC����ֵʮλ��
			digit[3] = Current2_out % 10;           // ��ʾADC����ֵ��λ��				

			digit[4] = Current1_out / 1000 % 10; 	     // ��ʾADC����ֵǧλ��
			digit[5] = Current1_out / 100 % 10; 	 // ��ʾADC����ֵ��λ��
			digit[6] = Current1_out / 10 % 10; 	 // ��ʾADC����ֵʮλ��
			digit[7] = Current1_out % 10;           // ��ʾADC����ֵ��λ��								
			led[0]=0; 
		  led[1]=1;
		  led[2]=0;
		}
		if(display_mode ==1)
		{
			pnt = 0x11;
			digit[0] = DAC6571_code / 1000 %10; 	  // ?????
			digit[1] = DAC6571_code / 100 % 10;   // ?????
			digit[2] = DAC6571_code / 10 % 10;    // ?????
			digit[3] = DAC6571_code % 10;         // ?????
			
			digit[4] = (Voltage_out /1000) %10;
			digit[5] = (Voltage_out / 100) % 10; // ��ʾ��ѹֵ��λ��
			digit[6] = (Voltage_out / 10) % 10;  // ��ʾ��ѹֵʮ��λ��
			digit[7] = Voltage_out % 10;         // ��ʾ��ѹֵ�ٷ�λ��  
			led[0]=1; 
		  led[1]=0;
		  led[2]=0;
		}
		if(display_mode ==3)
		{
			pnt = 0x04;
			digit[0] = f / 100000 %10; 	  // ?????
			digit[1] = f / 10000 % 10;   // ?????
			digit[2] = f / 1000 % 10;    // ?????
			digit[3] = f /100 % 10;         // ?????
			
		  digit[4] = '_';
			digit[5] = '_'; 
			digit[6] = 'F';
			digit[7] = '_';
			led[0]=0; 
		  led[1]=0;
		  led[2]=1;
		}
}


void UART0_Handler(void)
{
    int32_t uart0_int_status;

    uart0_int_status = UARTIntStatus(UART0_BASE, true); // ?????
    UARTIntClear(UART0_BASE, uart0_int_status);         // ?????

    switch (uart0_int_status)
    {
    case UART_INT_RT:                      // Receive Timeout Interrupt
    case UART_INT_RX:                      // Receive Interrupt
        while (UARTCharsAvail(UART0_BASE)) // ????? FIFO ????
        {
            uint8_t uart_receive_char = UARTCharGetNonBlocking(UART0_BASE); // ??????

            /**
             * ????????????,
             * ?? UART ??? Rx FIFO ?????? cmd_Rx_buf,
             * ???????????.
             */
            if (cmd_Rx_buf.WriteEnable)
            {
                // ??????????,?????
                if (cmd_Rx_buf.size >= cmd_Rx_buf.max_size)
                {
                    cmd_Rx_buf.data[cmd_Rx_buf.max_size - 1] = '\0';
                    cmd_Rx_buf.WriteEnable = false;
                }
                // tab ('\t'), white-space control codes ('\f','\v','\n','\r')
                // ? space (' ')????????,????.
                else if (isspace(uart_receive_char))
                {
                    cmd_Rx_buf.data[cmd_Rx_buf.size] = '\0'; // ????????????
                    // ???????????,???????,???????
                    if (cmd_Rx_buf.size)
                    {
                        cmd_Rx_buf.WriteEnable = false; // ????,????
                    }
                }
                else
                {
                    cmd_Rx_buf.data[cmd_Rx_buf.size++] = uart_receive_char;
                }
            }
        }
        break;
    case UART_INT_TX: // Transmit Interrupt
        // ?????????????(???)?? Tx FIFO
        while (!cmd_Tx_buf.WriteEnable && cmd_Tx_buf.size && UARTSpaceAvail(UART0_BASE))
        {
            if (UARTCharPutNonBlocking(UART0_BASE, cmd_Tx_buf.data[cmd_Tx_buf.next_trans_index]))
            {
                // ???????,??????????,????
                if (++cmd_Tx_buf.next_trans_index >= cmd_Tx_buf.size)
                {
                    cmd_Tx_buf.size = cmd_Tx_buf.next_trans_index = 0;
                    cmd_Tx_buf.WriteEnable = true;
                }
            }
        }
        break;
    default:
        break;
    }
}

/**
 * @brief ?? UART ???????
 * 
 * ??????????? cmd_Rx_buf ???????????????,
 * ??,???????,?????????????? cmd_Tx_buf ???;
 * ??,??????????????????? UART ??? Tx FIFO.
 */
void UART_clock(void)
{
    // ??????(????????????????),???????? cmd_Tx_buf ????,?????????????
    if (!cmd_Rx_buf.WriteEnable && cmd_Tx_buf.WriteEnable)
    {
        // 1. ?????????
        if (!strcmp("Voltage", (const char *)cmd_Rx_buf.data))
        {
					
					  display_mode=1;
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, " = %d.%d%d%d V",
                    Voltage_out/1000,Voltage_out/100%10,Voltage_out/10%10,Voltage_out%10);
						
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
        // 2. ???(??)??????
        else if (!strncmp("Current1", (const char *)cmd_Rx_buf.data, 6))
        {
            display_mode=2;
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, " = %d.%d%d%d A",
                    Current1_out/1000,Current1_out/100%10,Current1_out/10%10,Current1_out%10);
						
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
        // 3. ???(??)??????
        else if (!strncmp("Current2", (const char *)cmd_Rx_buf.data, 6))
        {
					  display_mode=2;
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, " = %d.%d%d%d A",
                    Current2_out/1000,Current2_out/100%10,Current2_out/10%10,Current2_out%10);
						
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
				else if (!strncmp("Frequency", (const char *)cmd_Rx_buf.data, 6))
        {
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, " = %d.%d kHz",
                    f/1000,(f/100)%10);
						display_mode=3;
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
        else if (!strncmp("Special Mode", (const char *)cmd_Rx_buf.data, 6))
        {
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, "Running in the special mode.");
						display_mode=2;
						special_mode=1;
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
				 else if (!strncmp("Normal Mode", (const char *)cmd_Rx_buf.data, 6))
        {
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};

            sprintf(src_str, "Running in the normal mode.");
						display_mode=2;
						special_mode=0;
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
				else if (!strncmp("Power", (const char *)cmd_Rx_buf.data, 6))
        {
            char src_str[CMD_RX_BUF_MAX_SIZE] = {'\0'};
            power = Voltage_out*(Current2_out+Current2_out);
            sprintf(src_str, " = %d.%d%d W",
                    power/1000000,(power/1000000)%10,(power/100000)%10);
						display_mode=2;
						special_mode=0;
            strcpy((char *)cmd_Tx_buf.data, (const char *)src_str);
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????

        }
        else
        {
            strcpy((char *)cmd_Tx_buf.data, "Error Command!");
            cmd_Tx_buf.size = strlen((const char *)cmd_Tx_buf.data);
            cmd_Tx_buf.next_trans_index = 0;
            cmd_Tx_buf.WriteEnable = false; // ????????????,????
        }

        // ?????????????,?????????
        cmd_Rx_buf.size = 0;           // ?????????
        cmd_Rx_buf.WriteEnable = true; // ????,???????????
    }

    
    while (!cmd_Tx_buf.WriteEnable && cmd_Tx_buf.size && UARTSpaceAvail(UART0_BASE))
    {
        if (UARTCharPutNonBlocking(UART0_BASE, cmd_Tx_buf.data[cmd_Tx_buf.next_trans_index]))
        {
            // ???????,??????????
            if (++cmd_Tx_buf.next_trans_index >= cmd_Tx_buf.size)
            {
                cmd_Tx_buf.size = cmd_Tx_buf.next_trans_index = 0; // ?????????
                cmd_Tx_buf.WriteEnable = true;                     // ???????????
            }
        }
    }
}






