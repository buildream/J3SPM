/*
        ADS1256.h - Arduino Library for communication with Texas Instrument
   ADS1256 ADC
        Written by Adien Akhmad, August 2015
*/

/*
	J3SPMDAQ.CPP - Arduino Mega Library for ADS1256 and DAC8554 
*/


#include "J3SPMDAQ.h"
#include "Arduino.h"
#include "SPI.h"

J3SPMDAQ::J3SPMDAQ(float clockspdMhz, float vref, bool useResetPin) {

  //////////////
  //ADC setting
  // Set DRDY as input
  DDR_DRDY &= ~(1 << PINDEX_DRDY);
  // Set CS as output
  DDR_CS |= (1 << PINDEX_CS);

  if (useResetPin) {
    // set RESETPIN as output
    DDR_RESET |= (1 << PINDEX_RESET);
    // pull RESETPIN high
    PORT_RESET |= (1 << PINDEX_RESET);
  }

  // Voltage Reference
  _VREF = vref;

  // Default conversion factor
  _conversionFactor = 1.0;

  ///////////////
  //DAC setting
   
  _SCLK = 52;
  _SYNC = 53;
  _D =  51;
  _ENA = 7;
  _A1 = 6;
  _A0 = 5;
  _LDAC = 8;
  _initialized = false; 
  changeChannelBits(0,0,0,1,1,0,1,1); 
  _headerByte = B00010110;


  SPI.begin();
  SPI.beginTransaction(SPISettings(clockspdMhz * 1000000 / 4, MSBFIRST, SPI_MODE1));

}

void J3SPMDAQ::writeRegister(unsigned char reg, unsigned char wdata) {
  digitalWrite(_SYNC,HIGH);
  CSON();
  SPI.transfer(WREG | reg);
  SPI.transfer(0);
  SPI.transfer(wdata);
  __builtin_avr_delay_cycles(8);  // t11 delay (4*tCLKIN) after WREG command,
                                  // 16Mhz avr clock is approximately twice
                                  // faster that 7.68 Mhz ADS1256 master clock
  CSOFF();
}

unsigned char J3SPMDAQ::readRegister(unsigned char reg) {
  unsigned char readValue;

  digitalWrite(_SYNC,HIGH);
  CSON();
  SPI.transfer(RREG | reg);
  SPI.transfer(0);
  __builtin_avr_delay_cycles(200);  // t6 delay (50*tCLKIN), 16Mhz avr clock is
                                    // approximately twice faster that 7.68 Mhz
                                    // ADS1256 master clock
  readValue = SPI.transfer(0);
  __builtin_avr_delay_cycles(8);  // t11 delay
  CSOFF();

  return readValue;
}

void J3SPMDAQ::sendCommand(unsigned char reg) {
  digitalWrite(_SYNC,HIGH);
  CSON();
  waitDRDY();
  SPI.transfer(reg);
  __builtin_avr_delay_cycles(8);  // t11
  CSOFF();
}

void J3SPMDAQ::setConversionFactor(float val) { _conversionFactor = val; }

void J3SPMDAQ::readTest() {
  unsigned char _highByte, _midByte, _lowByte;
  digitalWrite(_SYNC,HIGH);
  CSON();
  SPI.transfer(RDATA);
  __builtin_avr_delay_cycles(200);  // t6 delay

  _highByte = SPI.transfer(WAKEUP);
  _midByte = SPI.transfer(WAKEUP);
  _lowByte = SPI.transfer(WAKEUP);

  CSOFF();
}

float J3SPMDAQ::readChannel() {
  digitalWrite(_SYNC,HIGH);
  CSON();
  SPI.transfer(RDATA);
  __builtin_avr_delay_cycles(200);  // t6 delay
  float adsCode = read_float32();
  CSOFF();
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) *
         _conversionFactor;
 }

// Call this ONLY after RDATA command
  unsigned long J3SPMDAQ::read_uint24() {
  unsigned char _highByte, _midByte, _lowByte;
  unsigned long value;

  _highByte = SPI.transfer(WAKEUP);
  _midByte = SPI.transfer(WAKEUP);
  _lowByte = SPI.transfer(WAKEUP);

  // Combine all 3-bytes to 24-bit data using byte shifting.
  value = ((long)_highByte << 16) + ((long)_midByte << 8) + ((long)_lowByte);
  return value;
}

// Call this ONLY after RDATA command
long J3SPMDAQ::read_int32() {
  long value = read_uint24();

  if (value & 0x00800000) {
    value |= 0xff000000;
  }

  return value;
}

// Call this ONLY after RDATA command
float J3SPMDAQ::read_float32() {
  long value = read_int32();
  return (float)value;
}

// Channel switching for single ended mode. Negative input channel are
// automatically set to AINCOM
void J3SPMDAQ::setChannel(byte channel) { setChannel(channel, -1); }

// Channel Switching for differential mode. Use -1 to set input channel to
// AINCOM
void J3SPMDAQ::setChannel(byte AIN_P, byte AIN_N) {
  unsigned char MUX_CHANNEL;
  unsigned char MUXP;
  unsigned char MUXN;

  switch (AIN_P) {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }

  switch (AIN_N) {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }

  MUX_CHANNEL = MUXP | MUXN;
  digitalWrite(_SYNC,HIGH);
  CSON();
  writeRegister(MUX, MUX_CHANNEL);
  sendCommand(SYNC);
  sendCommand(WAKEUP);
  CSOFF();
}

void J3SPMDAQ::begin(unsigned char drate, unsigned char gain, bool buffenable) {
  _pga = 1 << gain;
  sendCommand(SDATAC);  // send out SDATAC command to stop continous reading mode.
  writeRegister(DRATE, drate);  // write data rate register
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADCON, byte2send);
  if (buffenable) {
    uint8_t status = readRegister(STATUS);
    bitSet(status, 1);
    writeRegister(STATUS, status);
  }
  sendCommand(SELFCAL);  // perform self calibration
  waitDRDY();
  ;  // wait ADS1256 to settle after self calibration
}

void J3SPMDAQ::CSON() {
 PORT_CS &= ~(1 << PINDEX_CS);
}

void J3SPMDAQ::CSOFF() {
PORT_CS |= (1 << PINDEX_CS);
}

void J3SPMDAQ::waitDRDY() {
  while (PIN_DRDY & (1 << PINDEX_DRDY))
    ;
}

////////////////
//DAC functions
void J3SPMDAQ::setPins(uint8_t SCLK, uint8_t CS, uint8_t D, uint8_t ENA, uint8_t A1, uint8_t A0, uint8_t LDAC)
{
  _SCLK = SCLK;
  _SYNC = CS;
  _D = D;
  _ENA = ENA;
  _A1 = A1;
  _A0 = A0;
  _LDAC = LDAC;
  _initialized = false;
}

void J3SPMDAQ::setDAC(uint8_t A)
{
  bitWrite(_headerByte,6,bitRead(A,0));
  bitWrite(_headerByte,7,bitRead(A,1));
}

void J3SPMDAQ::initializePins()
{
    pinMode(_SYNC,OUTPUT);
    if (_ENA != -1) pinMode(_ENA,OUTPUT);
    if (_A0 != -1) pinMode(_A0,OUTPUT);
    if (_A1 != -1) pinMode(_A1,OUTPUT);
    if (_LDAC != -1) pinMode(_LDAC,OUTPUT);
    digitalWrite(_SYNC,HIGH);
    if (_ENA != -1) digitalWrite(_ENA,LOW);
    if (_A0 != -1) digitalWrite(_A0,LOW);
    if (_A1 != -1) digitalWrite(_A1,LOW);
    if (_LDAC != -1) digitalWrite(_LDAC,LOW);
    _initialized = true;
}

void J3SPMDAQ::sendHeader(int channel)
{
  if(channel == 0) {
      bitWrite(_headerByte,5,0);
      bitWrite(_headerByte,2,_ChannelABit1);
      bitWrite(_headerByte,1,_ChannelABit2);
    } else if (channel == 1) {
      bitWrite(_headerByte,5,0);
      bitWrite(_headerByte,2,_ChannelBBit1);
      bitWrite(_headerByte,1,_ChannelBBit2);
    } else if (channel == 2) {
      bitWrite(_headerByte,5,0);
      bitWrite(_headerByte,2,_ChannelCBit1);
      bitWrite(_headerByte,1,_ChannelCBit2);
    } else if (channel == 3) {
      bitWrite(_headerByte,5,0);
      bitWrite(_headerByte,2,_ChannelDBit1);
      bitWrite(_headerByte,1,_ChannelDBit2);
    } else if (channel == -1) {
      bitWrite(_headerByte,2,1);
      bitWrite(_headerByte,5,1);
    }

    SPI.transfer(_headerByte);
}

void J3SPMDAQ::sendData(unsigned int value)
{
    SPI.transfer((value >> 8));
    SPI.transfer(value);
}

void J3SPMDAQ::updateChannel(int channel, unsigned int value)
{
  if(!_initialized) initializePins();
  CSOFF();
  digitalWrite(_SYNC,LOW);
  sendHeader(channel);
  sendData(value);
  digitalWrite(_SYNC,HIGH);
}

void J3SPMDAQ::updateAllChannels(unsigned int value)
{
  updateChannel(-1,value);
}

void J3SPMDAQ::changeChannelBits(uint8_t ChannelABit1, uint8_t ChannelABit2, uint8_t ChannelBBit1, uint8_t ChannelBBit2, uint8_t ChannelCBit1, uint8_t ChannelCBit2, uint8_t ChannelDBit1, uint8_t ChannelDBit2)
{
    _ChannelABit1 = ChannelABit1; 
    _ChannelABit2 = ChannelABit2; 
    _ChannelBBit1 = ChannelBBit1; 
    _ChannelBBit2 = ChannelBBit2; 
    _ChannelCBit1 = ChannelCBit1; 
    _ChannelCBit2 = ChannelCBit2; 
    _ChannelDBit1 = ChannelDBit1; 
    _ChannelDBit2 = ChannelDBit2; 
}

