/**
  @page FSMC_SRAM_DataMemory FSMC_SRAM_DataMemory
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file    FSMC/SRAM_DataMemory/readme.txt 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Description of the FSMC SRAM_DataMemory Example.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Example Description 

This example shows how to use the external SRAM mounted on STM3210E-EVAL board as
program data memory and internal SRAM for Stack.

@par Directory contents 

  - FSMC/SRAM_DataMemory/stm32f10x_conf.h      Library Configuration file
  - FSMC/SRAM_DataMemory/stm32f10x_it.c        Interrupt handlers
  - FSMC/SRAM_DataMemory/stm32f10x_it.h        Header for stm32f10x_it.c  
  - FSMC/SRAM_DataMemory/main.c                Main program

@par Hardware and Software environment 

  - This example runs only on STM32F10x High-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) evaluation board and can be easily tailored to any other 
    supported device and development board.
    
@par How to use it ? 

In order to make the program work, you must do the following:
- Create a project and setup all project configuration:
<ul>
 <li> RVMDK 
    - in Project->Options->Target window, select 'RAM1'and enter 0x68000000 as
      start address with size of 0x100000 (IRAM1 must be un-checked)
    - uncomment "#define DATA_IN_ExtSRAM " in the "system_stm32f10x.c" file               

 <li> EWARM5 
    - use "stm32f10x_flash_extsram.icf" as linker file
    - uncomment "#define DATA_IN_ExtSRAM " in the "system_stm32f10x.c" file

 <li> RIDE 
    - In the Application options -> script menu, set "Use Default Script File" 
      to "No" and use "stm32f10x_flash_extsram.ld" as Script File.
    - uncomment "#define DATA_IN_ExtSRAM " in the "system_stm32f10x.c" file 
</ul>

- Add the required Library files:
  - stm32f10x_rcc.c
  - system_stm32f10x.c (under Libraries\CMSIS\Core\CM3)
  - core_cm3.c (under Libraries\CMSIS\Core\CM3)
   
- Edit stm32f10x.h file to select the device you are working on (#define 
  STM32F10X_HD, in this case).
  
@b Tip: You can tailor the provided project template to run this example, for 
        more details please refer to "stm32f10x_stdperiph_lib_um.chm" user 
        manual; select "Peripheral Examples" then follow the instructions 
        provided in "How to proceed" section.   
- Link all compiled files and load your image into target memory
- Run the example

@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
    
 * <h3><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h3>
 */
