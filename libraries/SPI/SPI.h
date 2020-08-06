/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#if defined(__SAMD51__)
  // SAMD51 has configurable MAX_SPI, else use peripheral clock default.
  // Update: changing MAX_SPI via compiler flags is DEPRECATED, because
  // this affects ALL SPI peripherals including some that should NOT be
  // changed (e.g. anything using SD card). Use the setClockSource()
  // function instead. This is left here for compatibility with interim code.
  #if !defined(MAX_SPI)
    #define MAX_SPI 24000000
  #endif
  #define SPI_MIN_CLOCK_DIVIDER 1
#else
  // The datasheet specifies a typical SPI SCK period (tSCK) of 42 ns,
  // see "Table 36-48. SPI Timing Characteristics and Requirements",
  // which translates into a maximum SPI clock of 23.8 MHz.
  // Conservatively, the divider is set for a 12 MHz maximum SPI clock.
  #if !defined(MAX_SPI)
    #define MAX_SPI 12000000
  #endif
  #define SPI_MIN_CLOCK_DIVIDER (uint8_t)(1 + ((F_CPU - 1) / MAX_SPI))
#endif

class SPISettings {
  public:
  SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }

  // Default speed set to 4MHz, SPI mode set to MODE 0 and Bit order set to MSB first.
  SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }

  private:
  void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  }

  void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
#if defined(__SAMD51__)
    this->clockFreq = clock; // Clipping handled in SERCOM.cpp
#else
    this->clockFreq = (clock >= (MAX_SPI * 2 / SPI_MIN_CLOCK_DIVIDER) ? MAX_SPI * 2 / SPI_MIN_CLOCK_DIVIDER : clock);
#endif

    this->bitOrder = (bitOrder == MSBFIRST ? MSB_FIRST : LSB_FIRST);

    switch (dataMode)
    {
      case SPI_MODE0:
        this->dataMode = SERCOM_SPI_MODE_0; break;
      case SPI_MODE1:
        this->dataMode = SERCOM_SPI_MODE_1; break;
      case SPI_MODE2:
        this->dataMode = SERCOM_SPI_MODE_2; break;
      case SPI_MODE3:
        this->dataMode = SERCOM_SPI_MODE_3; break;
      default:
        this->dataMode = SERCOM_SPI_MODE_0; break;
    }
  }

  uint32_t clockFreq;
  SercomSpiClockMode dataMode;
  SercomDataOrder bitOrder;

  friend class SPIClass;
};

class SPIClass {
  public:
  SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad, SercomRXPad);

  byte transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  void transfer(void *buf, size_t count);
  //void transfer(void* txbuf, void* rxbuf, size_t count); //non dma
  void transfer(void* txbuf, void* rxbuf, uint32_t count, bool block); //dma poll for completion
  void waitForTransfer(void); //dma poll for completion
  void transfer(void* txbuf, void* rxbuf, uint32_t count, void (*functionToCallWhenComplete)(void) ); //dma asynchronous

  // Transaction Functions
  void usingInterrupt(int interruptNumber);
  void notUsingInterrupt(int interruptNumber);
  void beginTransaction(SPISettings settings);
  void endTransaction(void);

  // SPI Configuration methods
  void attachInterrupt();
  void detachInterrupt();

  void begin();
  void end();

  void setBitOrder(BitOrder order);
  void setDataMode(uint8_t uc_mode);
  void setClockDivider(uint8_t uc_div);

  // SERCOM lookup functions are available on both SAMD51 and 21.
  volatile uint32_t *getDataRegister(void);
  int getDMAC_ID_TX(void);
  int getDMAC_ID_RX(void);
  uint8_t getSercomIndex(void) { return _p_sercom->getSercomIndex(); };
#if defined(__SAMD51__)
  // SERCOM clock source override is available only on SAMD51.
  void setClockSource(SercomClockSource clk);
#else
  // On SAMD21, this compiles to nothing, so user code doesn't need to
  // check and conditionally compile lines for different architectures.
  void setClockSource(SercomClockSource clk) { };
#endif // end __SAMD51__

  private:
  void init();
  void config(SPISettings settings);

  SERCOM *_p_sercom;
  uint8_t _uc_pinMiso;
  uint8_t _uc_pinMosi;
  uint8_t _uc_pinSCK;

  SercomSpiTXPad _padTx;
  SercomRXPad _padRx;

  bool initialized;
  uint8_t interruptMode;
  char interruptSave;
  uint32_t interruptMask;

  //***********************************************************
  // constants, objects, and functions used for dma transfers

  #define DMA_MAX_TRANSFER_SIZE		65535			// maximum bytes a dma can transfer per transaction

  Adafruit_ZeroDMA readChannel;
  Adafruit_ZeroDMA writeChannel;
  DmacDescriptor  *readDescriptor  = NULL;
  DmacDescriptor  *writeDescriptor = NULL;

  volatile bool    dma_write_done 	 	= false;		// true when read dma callback completes
  volatile bool    dma_read_done  	 	= false;		// true when write dma callback completes
  uint32_t 		   dma_bytes_remaining 	= 0;			// number of bytes remaining for future dma transactions
  volatile bool    dma_complete  	 	= false;		// all transactions completed and no bytes remaining
  void* 		   txbuf_last			= NULL;			// pointer to buffer last used
  void* 		   rxbuf_last			= NULL;			// pointer to buffer last used

  static void      dmaCallback_read(Adafruit_ZeroDMA *dma);
  static void      dmaCallback_write(Adafruit_ZeroDMA *dma);
  static void      checkDmaComplete(uint8_t channel);
  void (*userDmaCallback)(void) = NULL; //function pointer to users dma callback function

};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClass SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
  extern SPIClass SPI2;
#endif
#if SPI_INTERFACES_COUNT > 3
  extern SPIClass SPI3;
#endif
#if SPI_INTERFACES_COUNT > 4
  extern SPIClass SPI4;
#endif
#if SPI_INTERFACES_COUNT > 5
  extern SPIClass SPI5;
#endif

// For compatibility with sketches designed for AVR @ 16 MHz
// New programs should use SPI.beginTransaction to set the SPI clock
#define SPI_CLOCK_DIV2   (MAX_SPI * 2 / 8000000)
#define SPI_CLOCK_DIV4   (MAX_SPI * 2 / 4000000)
#define SPI_CLOCK_DIV8   (MAX_SPI * 2 / 2000000)
#define SPI_CLOCK_DIV16  (MAX_SPI * 2 / 1000000)
#define SPI_CLOCK_DIV32  (MAX_SPI * 2 / 500000)
#define SPI_CLOCK_DIV64  (MAX_SPI * 2 / 250000)
#define SPI_CLOCK_DIV128 (MAX_SPI * 2 / 125000)

#endif
