/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#define ARDUINO_MAIN
#include "Arduino.h"

#define LORAWAN_GROVE_POWER 38

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
  init();

  initVariant();

  delay(1);
#if defined(USBCON)
  USBDevice.init();
  USBDevice.attach();
#endif

  // Enable all the grove ports when program starts,
  // if you don't need to use them, write LOW to D38 for 
  // saving power.
  pinMode(LORAWAN_GROVE_POWER, OUTPUT);
  digitalWrite(LORAWAN_GROVE_POWER, HIGH);

  // Enable all grove ports if program for Wio GPS Board
  // D12 is power control pin for Wio GPS Board
#ifdef WIO_GPS_BOARD
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
#endif

  setup();

  for (;;)
  {
    loop();
    if (serialEventRun) serialEventRun();
  }

  return 0;
}
