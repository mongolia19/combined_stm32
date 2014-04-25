#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/



extern CanTxMsg TxMessage;
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/




/* Exported functions ------------------------------------------------------- */



 
void SPI_Config(void);
void CAN_Config(void);
void RTC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Config(void);

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/