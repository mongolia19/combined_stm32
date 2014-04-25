/**
  ******************************************************************************
  * @file USART/HalfDuplex/main.c 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define TxBufferSize2   (countof(TxBuffer2) - 1)
//#define GPIO_Remap_CAN             GPIO_Remap2_CAN1
/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define TRUE 1
#define FALSE 0
/* Private variables ---------------------------------------------------------*/
uint8_t databuffer[8]={0};
uint8_t sendbufferall[111]={0};
uint8_t sendbuffer[8]={0};
uint8_t sendbuffer0[16]={0};
uint8_t sendbuffer1[16]={0};
uint8_t sendbuffer2[16]={0};
uint8_t sendbuffer3[16]={0};
uint8_t sendbuffer4[16]={0};
uint8_t sendbuffer5[16]={0};
//uint8_t sendbuffer7[31]={0};
volatile uint16_t ADC_ConvertedValue[4];
volatile float U1,U2,U3,U4;
float x0,y0;
int x_dir;
int y_dir;
int hitpos;
uint8_t lastlightpos=0;
uint8_t LocationSendFlag;
uint8_t shouldturnflag=0;
uint8_t ref,numofbyte,n,m,t,s,ct,cnt,k,a,code_count;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;  
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void CAN_Configuration(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);
void TIM4_Configuration(void);
void USART_Config(USART_TypeDef* USARTx);
void Delay(__IO uint32_t nCount);
void UART_OUT(uint16_t *Data,uint16_t Len);
void Decode();
unsigned char ReceiveByte(void);
u32 GetTIM_Period(u8 n);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[6] = {0};
uint8_t TxBuffer2[] = "USART Half Duplex: USARTz -> USARTy using HalfDuplex mode";
uint8_t RxBuffer1[13]={0};
uint8_t RxBuffer2[TxBufferSize1];
uint8_t TxCounter1 = 0, RxCounter1 = 0,data=0,rec_f=0,n,m;
uint8_t TxCounter2 = 0, RxCounter2 = 0;

extern CanTxMsg TxMessage;
uint8_t y_bias;
int distance_valid_flag;
int send_data_flag;
void ADC1_Init(void);
void ADC1_GPIO_Config(void);
void ADC1_Mode_Config(void);
uint8_t getmax(float u1,float u2,float u3,float u4);
TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
TIM_TimeBaseInitTypeDef  TIM4_TimeBaseStructure;
TIM_OCInitTypeDef  TIM3_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
USART_InitTypeDef USART_InitStructure;
USART_InitTypeDef USART_InitStruct;
USART_ClockInitTypeDef USART_ClockInitStruct; 

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  RCC_Configuration();
  GPIO_Configuration();
  CAN_Configuration();	
  
  TIM4_Configuration();
  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  //CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
  USART_Config(USART1);
  USART_Cmd(USART1, ENABLE);
  
  USART_Config(USART3);
  USART_Cmd(USART3, ENABLE);
  //GPIO_ResetBits(GPIOB, GPIO_Pin_8|GPIO_Pin_9);
  //GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7);	
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  NVIC_Configuration();
  //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  
  while (1)
  { 
      
    if(rec_f==1)
      {
	rec_f=0;
       /* TxMessage.ExtId=0x1210;
        TxMessage.RTR=CAN_RTR_DATA;
        TxMessage.IDE=CAN_ID_STD;
        TxMessage.DLC=5;
        TxMessage.Data[0]=TxBuffer1[1];
        TxMessage.Data[1]=TxBuffer1[2];
        TxMessage.Data[2]=TxBuffer1[3];
        TxMessage.Data[3]=TxBuffer1[4];
        TxMessage.Data[4]=TxBuffer1[5];*/
        //CAN_Transmit(CAN1, &TxMessage);
        //*******USART_ITConfig(USARTy, USART_IT_RXNE, ENABLE); 防止接收太快，导致发送的数据不准确
        //UART_OUT(&TxBuffer1[0],6); //用rs485把 初始距离数据传出去
        TxMessage.ExtId=0x111214;
        TxMessage.RTR=CAN_RTR_DATA;
        TxMessage.IDE=CAN_ID_EXT;
        TxMessage.DLC=8;
        
        for(n=0,m=0;n<6;n++,m++)
        {
          switch( TxBuffer1[n])
          {
              case 0x30:TxMessage.Data[m]=0x0;
	      break;
               case 0x31:TxMessage.Data[m]=0x01;
	      break;
               case 0x32:TxMessage.Data[m]=0x02;
	      break;
               case 0x33:TxMessage.Data[m]=0x03;
	      break;
               case 0x34:TxMessage.Data[m]=0x04;
	      break;
               case 0x35:TxMessage.Data[m]=0x05;
	      break;
               case 0x36:TxMessage.Data[m]=0x06;
	      break;
               case 0x37:TxMessage.Data[m]=0x07;
	      break;
               case 0x38:TxMessage.Data[m]=0x08;
	      break;
               case 0x39:TxMessage.Data[m]=0x09;
	      break;
          }
        }
        
        CAN_Transmit(CAN1, &TxMessage); //这里是将距离值的1000被传给DSP 通过CAN 比如距离是18.555 传给DSP 是18555
      }
  }
}
/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
       
void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3|RCC_APB1Periph_USART2|RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2,ENABLE);
  
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   /* Configure USART1 Tx as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
   GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure USART2 Tx as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  /* Configure  PWM as alternate function  */  
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
     /* Configure  Pb0/1 pc0/4 INPUT PINs   */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;//|GPIO_Pin_6|GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4;//|GPIO_Pin_6|GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Configure USARTz(usart3) Tx as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
  /* Configure  TIM4 INPUT PIN   */
 /* GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
*/
}

void USART_Config(USART_TypeDef* USARTx){
   USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);	//配置串口参数函数
  
}
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
  
    /* Enable the USART1 Interrupt */
 NVIC_InitStructure.NVIC_IRQChannel =  USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USART2 Interrupt */
 NVIC_InitStructure.NVIC_IRQChannel =  USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
     NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
   /*NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
  
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
      NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
u32 GetTIM_Period(u8 n)////input 10 times bigger
{
  return (20000/n-1);
}
void TIM4_Configuration(void)
{
  
  TIM4_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
  TIM4_TimeBaseStructure.TIM_Period = 9999;      //  CK_CNT的计数频率=72M/(7199+1)=10K；10K/(9999+1)=1HZ ，也就是1S溢出一次；    
  TIM4_TimeBaseStructure.TIM_Prescaler = 1439;	      			     
  TIM4_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM4_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM4,&TIM4_TimeBaseStructure);

  TIM_Cmd(TIM4, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  
  
//  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(TIM4, &TIM_ICInitStructure);
//  TIM_Cmd(TIM4, ENABLE);
//TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

}  

void TIM3_Configuration(void)
{
TIM_DeInit(TIM3);

  /* Time Base configuration */
  TIM3_TimeBaseStructure.TIM_Prescaler =281;////yuanlai 281 719
  TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM3_TimeBaseStructure.TIM_Period =60000;/////60000    10000 			              //确定频率			     
  TIM3_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM3_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

  TIM_TimeBaseInit(TIM3,&TIM3_TimeBaseStructure);
  TIM_Cmd(TIM3,ENABLE);
  //TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);
  

}


void TIM2_Configuration(void)
{
  TIM_DeInit(TIM2);

  /* Time Base configuration */
  TIM3_TimeBaseStructure.TIM_Prescaler =359;//17
  TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM3_TimeBaseStructure.TIM_Period =39999;				              //确定频率			     
  TIM3_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM3_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

  TIM_TimeBaseInit(TIM2,&TIM3_TimeBaseStructure);
  TIM_Cmd(TIM2,ENABLE);///removed
  TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
  

}

void CAN_Configuration(void)
{
  
  /* CAN1 Periph clock enable */
  
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */ 
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 16;//2 4=1m 8=500k
  CAN_Init(CAN1, &CAN_InitStructure);
  CAN_Init(CAN2, &CAN_InitStructure);
 /* CAN filter init */
 
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInit(&CAN_FilterInitStructure);
  CAN_FilterInitStructure.CAN_FilterNumber=14;
  CAN_FilterInit(&CAN_FilterInitStructure);
 
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
   /* Configure CAN1 pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure CAN1 pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure CAN pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure CAN pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
}
uint8_t getmax(float u1,float u2,float u3,float u4)
{
  if(u1>=u2&u1>=u3&u1>=u4)
  {
    return 1;
  }
  else
  {
    if(u2>=u1&u2>=u3&u2>=u4)
    {
      return 2;
    }
    else
    {
      if(u3>=u1&u3>=u2&u3>=u4)
      {
        return 3;
      }
      else
      {
        return 4;
      }
    }
  }
}
void ADC1_Init()
{
  ADC1_GPIO_Config();
  ADC1_Mode_Config();
}

void ADC1_GPIO_Config()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ////Enable DMA Clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
////////////////PB0-PB1 PC0 PC4  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
}

void ADC1_Mode_Config()
{
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr=ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  
    /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立模式  CR1
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//扫描模式使能
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//连续转换模式使能
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//启动转换的外部事件--无  CR2
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//转换后的数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 4;                       //转换的通道数为1
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  //ADC12_IN8
  //ADC12_IN9
  //ADC12_IN10
  //ADC12_IN14
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_41Cycles5);	  //通道11采样时间
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_41Cycles5);	  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_41Cycles5);	  //通道11采样时间
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_41Cycles5);	  
  
  /* Enable ADC1 DMA */															  
  ADC_DMACmd(ADC1, ENABLE);//允许ADC1进行DMA传送
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);//使能ADC1

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);//允许ADC1复位校准寄存器 
  /* Check the end of ADC1 reset calibration register */
 // while(ADC_GetResetCalibrationStatus(ADC1));		//检测校准寄存器是否复位完成 

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);//启动ADC1 校准
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1)); //检测校准是否完成 
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//软件触发启动ADC1转换
}
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
void UART_OUT(uint16_t *Data,uint16_t Len)
{ 
  
uint8_t i;
	for(i=0; i<Len; i++)
	{ 
          Delay(10000); 
		USART_SendData(USART2,Data[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);   //判断是否发送完成。 
	}

}
/*************** 接收字符*****************/
unsigned char ReceiveByte(void)
{
     unsigned char i,temp1,temp2 ;
     temp1=temp2=0 ;
     for(i=0 ;i< 8 ;i++)
     {
           temp1=temp1<< 1 ;
           //GPIO_ResetBits(GPIOA, GPIO_Pin_10);//SCK = 0 ;
           //GPIO_SetBits(GPIOA, GPIO_Pin_10);//SCK = 1 ;
           //GPIO_ResetBits(GPIOA, GPIO_Pin_10);//SCK = 0 ;
           if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)==1) 
           {
             temp1++ ;
           }
     }
     for(i=0 ;i< 8 ;i++)
     {
           temp2=temp2<< 1 ;
           //GPIO_ResetBits(GPIOA, GPIO_Pin_10);//SCK = 0 ;
          // GPIO_SetBits(GPIOA, GPIO_Pin_10);//SCK = 1 ;
           //GPIO_ResetBits(GPIOA, GPIO_Pin_10);//SCK = 0 ;
           if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)==1) 
           {
             temp2++ ;
           }
     }
     return ((0xf0&temp1)+(0x0f&temp2)) ;
}
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
