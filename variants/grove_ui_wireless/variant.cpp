/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

/*
 * Pins descriptions
 * TCC0 IOSET4
 * TCC1 IOSET2
 * TCC2 IOSET1
 * TCC3 IOSET1
 * TCC4 IOSET1
 */

const PinDescription g_APinDescription[]=
{
  // 0..9 - Digital & AD pins
  { PORTB, 8, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel2, TC4_CH0, TC4_CH0, EXTERNAL_INT_8 },
  { PORTB, 9, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel1, TC4_CH1, TC4_CH1, EXTERNAL_INT_9 },
  { PORTC, 2, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },
  { PORTC, 3, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },
  { PORTB, 4, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },
  { PORTB, 5, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
  { PORTB, 6, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },
  { PORTB, 7, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },
  { PORTC, 30, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel12, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTC, 31, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },

  //10
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  
  //-----------------------
  //11..12 DAC - Analog pins
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   // DAC/VOUT[0]
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   // DAC/VOUT[1]

  // 13 (LED)
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC2_CH1, NOT_ON_TIMER, EXTERNAL_INT_15 },
  
  // 14..16 - USB
  // --------------------
  { PORTA, 27, PIO_COM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // USB/DM
  { PORTA, 27, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // USB/DP

  //17..20  IRQ
  { PORTC, 4, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4},//USER BUTTON 1
  { PORTD, 0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },//USER BUTTON 2
  { PORTD, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },//USER BUTTON 3
  { PORTD, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },

  //-----------------------
  //21..25  EXTERNAL_IRQ
  { PORTC, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },
  { PORTC, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
  { PORTC, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },
  { PORTC, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },
  { PORTC, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  
  //26..29  UARTS
  { PORTC, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  // UART1_TX
  { PORTC, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  // UART1_RX  
  { PORTB, 27, PIO_SERCOM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM_F), No_ADC_Channel, TCC1_CH3, NOT_ON_TIMER, EXTERNAL_INT_13 },  //UART2_RX
  { PORTB, 26, PIO_SERCOM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM_F), No_ADC_Channel, TCC1_CH2, NOT_ON_TIMER, EXTERNAL_INT_12 },  //UART2_TX

  //-----------------------
  //30..33   I2C
  { PORTA, 17, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC1_CH0, EXTERNAL_INT_1 },
  { PORTA, 16, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC1_CH1, EXTERNAL_INT_0 },
  { PORTA, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_13 }, 
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_12 },          // DEN1


   //-----------------------
  //34..37   SPI3 LCD_SPI
  //PB8 PB19 PB20 PB21 (MISO PAD2, MOSI PAD3 SCK:PAD1 SS:PAD0)
  { PORTB, 18, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },
  { PORTB, 19, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },
  { PORTB, 20, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },
  { PORTB, 21, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },

  //38..41   SPI1 ESP_ESPI
  //PC24 PB24 PB25 PC25 (MISO:PAD2, MOSI:PAD0, SCK:PAD1, SS:PAD3)
  { PORTC, 24, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },
  { PORTB, 24, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },
  { PORTB, 25, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTC, 25, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },
 
  //42..45   SPI2  SD_SPI
  //PC18 PC16 PC17 PC19 (MISO:PAD2, MOSI:PAD0, SCK:PAD1, SS:PAD3)
  { PORTC, 18, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },
  { PORTC, 16, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },
  { PORTC, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },
  { PORTC, 19, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },

  //46..49   SPI
  //PB00 PB02 PB03 PB01 (MISO:PAD2, MOSI:PAD0, SCK:PAD1, SS:PAD3)
  { PORTB, 0, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), No_ADC_Channel, TC7_CH0, TC7_CH0, EXTERNAL_INT_0 },
  { PORTB, 2, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), No_ADC_Channel, TC6_CH0, TC6_CH0, EXTERNAL_INT_2 },
  { PORTB, 3, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), No_ADC_Channel, TC6_CH1, TC6_CH1, EXTERNAL_INT_3 },
  { PORTB, 1, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), No_ADC_Channel, TC7_CH1, TC7_CH1, EXTERNAL_INT_1 },

  // ----------------------
  //50..55 QSPI (SCK, CS, IO0, IO1, IO2, IO3)
  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  { PORTA, 8, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },
  { PORTA, 9, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },
  { PORTA, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTA, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },

  // ---------------------
  //56..63 I2S(FS0 FS1 MCK0 MCK1 SCK0 SCK1 SDI SDO)
  { PORTA, 20, PIO_TIMER_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH0, NOT_ON_TIMER, EXTERNAL_INT_4 },
  { PORTA, 23, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC4_CH1, TC4_CH1, EXTERNAL_INT_7 },
  { PORTB, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, 
  { PORTB, 29, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB, 16, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC6_CH0, TC6_CH0, EXTERNAL_INT_0 },    
  { PORTB, 28, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  
  { PORTA, 22, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_6 },     
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_5 },
  
  //---------------------
  //Extra Digital Pins
  //64..67
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH0, TC3_CH0, EXTERNAL_INT_14 },
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC2_CH1, NOT_ON_TIMER, EXTERNAL_INT_15 },
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH0, TC3_CH0, EXTERNAL_INT_2 },
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH1, TC3_CH1, EXTERNAL_INT_3 },
  
  //68..72
  { PORTB, 12, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH0, NOT_ON_TIMER, EXTERNAL_INT_12 },     
  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH1, NOT_ON_TIMER, EXTERNAL_INT_13 }, 
  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, NOT_ON_TIMER, EXTERNAL_INT_14 },    
  { PORTB, 15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH1, NOT_ON_TIMER, EXTERNAL_INT_15 }, 
  { PORTB, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
  
  //73..81
  { PORTC, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTC, 5, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
  { PORTC, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },
  { PORTC, 7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
  { PORTC, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTC, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
  { PORTC, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC0_CH4, EXTERNAL_INT_4 },
  { PORTC, 26, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC0_CH4, EXTERNAL_INT_10 },
  { PORTC, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC0_CH4, EXTERNAL_INT_12 },

  //82..87
  { PORTD, 8, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },    
  { PORTD, 9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },    
  { PORTD, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },  
  { PORTD, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },
  { PORTD, 20, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH0, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTD, 21, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH1, NOT_ON_TIMER, EXTERNAL_INT_11 },

  //88..91
  //4 WIRE LCD TOUCH
  { PORTC, 10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTC, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  { PORTC, 12, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_12 },
  { PORTC, 13, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_13 },
 
  //92 
  // MIC
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },  // DAC/VREFP
  //93..95
  //GYRDSCOPE_ADC
  { PORTA, 4, PIO_ANALOG, PIN_ATTR_PWM_E, ADC_Channel14, TC0_CH0, TC0_CH0, EXTERNAL_INT_4 }, 
  { PORTA, 6, PIO_ANALOG, PIN_ATTR_PWM_E, ADC_Channel16, TC1_CH0, TC1_CH0, EXTERNAL_INT_6 }, 
  { PORTA, 7, PIO_ANALOG, PIN_ATTR_PWM_E, ADC_Channel17, TC1_CH1, TC1_CH1, EXTERNAL_INT_7 },
  
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5, TC6, TC7 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;
SERCOM sercom6( SERCOM6 ) ;
SERCOM sercom7( SERCOM7 ) ;

Uart Serial1( &SERCOM_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial2( &SERCOM_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

void SERCOM1_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM1_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM1_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM1_3_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM2_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_3_Handler()
{
  Serial2.IrqHandler();
}