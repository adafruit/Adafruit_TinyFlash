// Barebones library for Winbond W25Q80BV serial flash memory.  Written
// with the limited space of the ATtiny85 in mind, but works on regular
// Arduinos too.  Provides basic functions needed for audio playback
// system only: chip and sector erase, block write, sequential byte read.
// Other than possibly adding support for other Winbond flash in the
// future, the plan is to NOT bloat this out with all bells and whistles;
// functions like block reads or buffered writes can be implemented in
// client code where RAM can be better managed in the context of the
// overall application (1 flash page = 1/2 the ATtiny85's RAM).
// Written by Limor Fried and Phillip Burgess for Adafruit Industries.
// MIT license.

#include "Adafruit_TinyFlash.h"

#define CMD_PAGEPROG 0x02
#define CMD_READDATA 0x03
#define CMD_WRITEDISABLE 0x04
#define CMD_READSTAT1 0x05
#define CMD_WRITEENABLE 0x06
#define CMD_SECTORERASE 0x20
#define CMD_CHIPERASE 0x60
#define CMD_ID 0x90

#define STAT_BUSY 0x01
#define STAT_WRTEN 0x02

#define CHIP_BYTES_W25Q80 1L * 1024L * 1024L
#define CHIP_BYTES_W25Q16 2L * 1024L * 1024L
#define CHIP_BYTES_W25Q32 4L * 1024L * 1024L
#define CHIP_BYTES_W25Q64 8L * 1024L * 1024L
#define CHIP_BYTES_W25Q128 16L * 1024L * 1024L

uint32_t __capacity;

#ifdef __AVR_ATtiny85__

#define CHIP_SELECT PORTB &= ~cs_mask;
#define CHIP_DESELECT PORTB |= cs_mask;
#define SPIBIT                                                                 \
  USICR = ((1 << USIWM0) | (1 << USITC));                                      \
  USICR = ((1 << USIWM0) | (1 << USITC) | (1 << USICLK));
static uint8_t spi_xfer(uint8_t n) {
  USIDR = n;
  SPIBIT
  SPIBIT
  SPIBIT
  SPIBIT
  SPIBIT
  SPIBIT
  SPIBIT
  SPIBIT
  return USIDR;
}

#else

#include <SPIMemory.h>
#define CHIP_SELECT *cs_port &= ~cs_mask;
#define CHIP_DESELECT *cs_port |= cs_mask;
#define spi_xfer(n) SPI.transfer(n)

#endif

// Constructor
/**
 * @brief Construct a new Adafruit_TinyFlash::Adafruit_TinyFlash object
 *
 * @param cs Chip Select pin
 */
Adafruit_TinyFlash::Adafruit_TinyFlash(uint8_t cs) {
#ifndef __AVR_ATtiny85__
  cs_port = portOutputRegister(digitalPinToPort(cs));
  cs_pin = cs;
#endif
  cs_mask = digitalPinToBitMask(cs);
}

//
/**
 * @brief Select chip and issue command (don't deselect; data may follow)
 *
 * @param c The SPI flash command to issue
 */
void Adafruit_TinyFlash::cmd(uint8_t c) { CHIP_SELECT(void) spi_xfer(c); }

//
/**
 * @brief  Initialize SPI pins, validate chip is present
 *
 * @return uint32_t The chip capacity in bytes
 */
SPIFlash flash;
uint32_t Adafruit_TinyFlash::begin(void) {
  uint32_t JedecID;
  flash.begin();
#ifdef __AVR_ATtiny85__
  PORTB &= ~(_BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2) | cs_mask);
  DDRB &= ~_BV(PORTB0); // DI (NOT MISO)
  DDRB |= _BV(PORTB1)   // DO (NOT MOSI)
          | _BV(PORTB2) // SCK
          | cs_mask;    // CS
#else
  pinMode(cs_pin, OUTPUT);
  SPI.begin();
  // Resistor-based 5V->3.3V logic conversion is a little sloppy, so:
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 500 KHz
#endif

  JedecID = flash.getJEDECID();
  

  switch (JedecID)
  {
  case 0xEF4014:
    __capacity = CHIP_BYTES_W25Q80;
    break;
  case 0xEF4015:
    __capacity = CHIP_BYTES_W25Q16;
    break;
  case 0xEF4016:
    __capacity = CHIP_BYTES_W25Q32;
    break;
  case 0xEF4017:
    __capacity = CHIP_BYTES_W25Q64;
    break;
  case 0xEF4018:
    __capacity = CHIP_BYTES_W25Q128;
    break;
  default:
    __capacity = 0L;
    break;
  }
  return __capacity;
}

//
/**
 * @brief Poll status register until busy flag is clear or timeout occurs
 *
 * @param timeout The maximum time to wait for ready
 * @return boolean true:busy flag cleared false: timeout occoured before the
 * busy flag cleared
 */
boolean Adafruit_TinyFlash::waitForReady(uint32_t timeout) {
  uint8_t status;
  uint32_t startTime = millis();

  do {
    cmd(CMD_READSTAT1);
    status = spi_xfer(0);
    CHIP_DESELECT
    if ((millis() - startTime) > timeout)
      return false;
  } while (status & STAT_BUSY);

  return true;
}

//
/**
 * @brief  Set up a read operation (no data is returned yet)
 *
 * @param addr The address to setup a read from
 * @return boolean
 */
boolean Adafruit_TinyFlash::beginRead(uint32_t addr) {

  if ((addr >= __capacity) || !waitForReady())
    return false;

  cmd(CMD_READDATA);
  (void)spi_xfer(addr >> 16);
  (void)spi_xfer(addr >> 8);
  (void)spi_xfer(addr);
  // Chip is held in selected state until endRead()

  return true;
}

//
/**
 * @brief  Read next byte (call N times following beginRead())
 *
 * @return uint8_t The byte that was read
 */
uint8_t Adafruit_TinyFlash::readNextByte(void) { return spi_xfer(0); }

//
/**
 * @brief Stop read operation
 *
 */
void Adafruit_TinyFlash::endRead(void){CHIP_DESELECT}

//
/**
 * @brief  Erase the whole chip.  Boom, gone.  Use with caution.
 *
 * @return boolean true: sucess false: failure
 */
boolean Adafruit_TinyFlash::eraseChip(void) {

  if (!waitForReady() || !writeEnable())
    return false;

  // Might want to have this clear the block protect bits

  cmd(CMD_CHIPERASE);
  CHIP_DESELECT

  if (!waitForReady(10000L))
    return false; // Datasheet says 6S max

  writeDisable();

  return true;
}

//
/**
 * @brief Erase one 4K sector
 *
 * @param addr An address within the sector to be erased
 * @return boolean true: success false: failure
 */
boolean Adafruit_TinyFlash::eraseSector(uint32_t addr) {

  if (!waitForReady() || !writeEnable())
    return false;

  cmd(CMD_SECTORERASE);
  (void)spi_xfer(addr >> 16); // Chip rounds this down to
  (void)spi_xfer(addr >> 8);  // prior 4K sector boundary;
  (void)spi_xfer(0);          // lowest bits are ignored.
  CHIP_DESELECT

  if (!waitForReady(1000L))
    return false; // Datasheet says 400ms max

  writeDisable();

  return true;
}

//
/**
 * @brief Private function used by write and erase operations
 *
 * @return boolean true: success false: failure
 */
boolean Adafruit_TinyFlash::writeEnable(void) {
  uint8_t status;

  cmd(CMD_WRITEENABLE);
  CHIP_DESELECT

  // Verify write-enable status
  cmd(CMD_READSTAT1);
  status = spi_xfer(0);
  CHIP_DESELECT
  return (status & STAT_WRTEN) ? true : false;
}

//
/**
 * @brief Companion to above function, used after write and erase operations
 *
 */
void Adafruit_TinyFlash::writeDisable(void) {
  cmd(CMD_WRITEDISABLE);
  CHIP_DESELECT
}

/**
 * @brief Writes one page: 256 bytes, starting at 256 byte boundary.  There are
no other options.  This is the ONLY write method provided by the library;
other capabilities (if needed) may be implemented in client code.
 *
 * @param addr The address to start writing from. If the address is not a page
bountry, the preceding boundry address will be used
 * @param data Pointer to the data to be written
 * @return boolean true: success false: failure
 */
boolean Adafruit_TinyFlash::writePage(uint32_t addr, uint8_t *data) {
  if ((addr >= __capacity) || !waitForReady() || !writeEnable())
    return false;

  cmd(CMD_PAGEPROG);
  (void)spi_xfer(addr >> 16);
  (void)spi_xfer(addr >> 8);
  (void)spi_xfer(0); // If len=256, page boundary only (datasheet 7.2.21)
  for (int i = 0; i < 256; i++) {
    (void)spi_xfer(data[i]);
  }
  CHIP_DESELECT // Write occurs after the CS line is de-asserted

      delay(3); // Max page program time according to datasheet

  if (!waitForReady())
    return false;

  writeDisable();

  return true;
}
