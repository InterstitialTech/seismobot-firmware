#include "max11210.h"
#include <libopencm3/sam/d/port.h>
#include <libopencm3/sam/d/spi.h>

#define PIN_CS 5
int SSS = 5;
// Constructor
void max_begin(void) {
  // The firmware (spark_wiring.cpp) does not allow monitoring of MISO using digitalRead().
  // The MISO if therefore also connected to a digital input of the Spark Core.
  //pinMode(D0, INPUT);
    
  // The Spark Core system clock is 72 MHz, while the MAX11210 SPI runs at 5 MHz maximum.
  // Setting the SPI clock divider to 16x results in a SCLK of 4.5 MHz.
  //SPI.begin();
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV8);
  //SPI.setDataMode(SPI_MODE0);

  // Set default configuration of the ADC in register CTRL1
  //setLineFreq(MAX11210_50HZ);
  //setInputRange(MAX11210_UNIPOLAR);
  //setClockSource(MAX11210_INTCLK);
  //setEnableRefBuf(false);
  //setEnableSigBuf(false);
  //setFormat(MAX11210_2SCOMPL)
  //setConvMode(MAX11210_CONT);
  _writeReg8(CTRL1, 0b10100100);
  _sCycle = false;

  // Set default configuration of the ADC in register CTRL2
  //pinModeGpio(0, OUTPUT);
  //pinModeGpio(1, OUTPUT);
  //pinModeGpio(2, OUTPUT);
  //pinModeGpio(3, OUTPUT);
  //digitalWriteGpio(0, HIGH);
  //digitalWriteGpio(1, LOW);
  //digitalWriteGpio(2, HIGH);
  //digitalWriteGpio(3, LOW);
  _writeReg8(CTRL2, 0b00001111);
  
  // Set default configuration of the ADC in register CTRL3
  //setGain(MAX11210_GAIN1);
  //setDisableSysGain(true);
  //setDisableSysOffset(true);
  //setDisableSelfCalGain(true);
  //setDisableSelfCalOffset(true);
  _writeReg8(CTRL3, 0b00011000);
  selfCal();

  // Initiate conversions
  //_rate = MAX11210_RATE30;
  //setRate(_rate);
}
void max_end(void) {
  _sendCmd(IMPD);
  //SPI.end();
}
unsigned char spitransfer(unsigned char data){
        SPI_DATA(SERCOM0_BASE) = data;
        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){} 
        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
        unsigned char resp = SPI_DATA(SERCOM0_BASE);
        return resp;
}

unsigned char _readReg8(unsigned char addr) {
  unsigned char mesg = ((addr & 0x0F) << 1) | START | MODE | READ;
  unsigned char resp;
        PORT_OUTCLR(PORTA) = (1 << PIN_CS);
        spitransfer(mesg);
        resp = spitransfer(0x0);
        PORT_OUTSET(PORTA) = (1 << PIN_CS);
  return resp;    
}

long _readReg24(unsigned char addr) {
  unsigned char mesg = ((addr & 0x0F) << 1) | START | MODE | READ;
  long resp;
        PORT_OUTCLR(PORTA) = (1 << PIN_CS);
        spitransfer(mesg);
        resp = spitransfer(0x0);
        resp = resp << 8;
        resp |= spitransfer(0x0);
        resp = resp << 8;
        resp |= spitransfer(0x0);


//        SPI_DATA(SERCOM0_BASE) = mesg;
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){} 
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
//        data = SPI_DATA(SERCOM0_BASE);
//        SPI_DATA(SERCOM0_BASE) = 0x00;
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){}
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
//        data = SPI_DATA(SERCOM0_BASE);
//        data = data << 8;
//        SPI_DATA(SERCOM0_BASE) = 0x00;
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){}
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
//        data = SPI_DATA(SERCOM0_BASE);
//        data = data << 8;
//        SPI_DATA(SERCOM0_BASE) = 0x00;
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){}
//        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
//        data = SPI_DATA(SERCOM0_BASE);
         
        PORT_OUTSET(PORTA) = (1 << PIN_CS);
  return resp;
}

long _readReg24s(unsigned char addr) {
  unsigned char mesg = ((addr & 0x0F) << 1) | START | MODE | READ;
  
        PORT_OUTCLR(PORTA) = (1 << PIN_CS);
        spitransfer(mesg);
  //digitalWrite(SSS, LOW);
  //SPI.transfer(mesg);
  long resp = spitransfer(0x00);
  resp = resp << 8;
  resp |= spitransfer(0x00);
  resp = resp << 8;
  resp |= spitransfer(0x00);
        PORT_OUTSET(PORTA) = (1 << PIN_CS);
        PORT_OUTCLR(PORTA) = (1 << PIN_CS);
  //digitalWrite(SSS, HIGH);
  //delay(1);
  //digitalWrite(SSS, LOW);
  return resp;
}

void _writeReg8(unsigned char addr, unsigned char data) {
  unsigned char mesg = ((addr & 0x0F) << 1) | START | MODE;
  PORT_OUTCLR(PORTA) = (1 << PIN_CS);
  spitransfer(mesg);
  spitransfer(data);
  PORT_OUTSET(PORTA) = (1 << PIN_CS);
}

void _writeReg24(unsigned char addr, long data) {
  unsigned char mesg = ((addr & 0x0F) << 1) | START | MODE;
  PORT_OUTCLR(PORTA) = (1 << PIN_CS);
  spitransfer(mesg);
  spitransfer((data >> 16) & 0xFF);
  spitransfer((data >> 8) & 0xFF);
  spitransfer(data & 0xFF);
  PORT_OUTSET(PORTA) = (1 << PIN_CS);
}

void _sendCmd(unsigned char data) {
  unsigned char mesg = (data & 0x3F) | START;
  PORT_OUTCLR(PORTA) = (1 << PIN_CS);
  spitransfer(mesg);
  PORT_OUTSET(PORTA) = (1 << PIN_CS);
}

long max_read(void) {

  unsigned char addr = DATA;
  long resp = _readReg24s(addr);
  return resp;
}
void selfCal(void) {
  _sendCmd(CAL0);
  for (long i=0; i<200000; i++) {
      __asm__("nop");
  }
      //delay(220); // A self-calibration requires 200 ms to complete
}

void sysOffsetCal(void) {
  _sendCmd(CAL1);
  for (long i=0; i<200000; i++) {
      __asm__("nop");
  }
  //delay(120); // A zero-scale calibration requires 100 ms to complete
}

void sysGainCal(void) {
  _sendCmd(CAL0 | CAL1);
  for (long i=0; i<200000; i++) {
      __asm__("nop");
  }
  //delay(120); // A full-scale calibration requires 100 ms to complete
}

void setRate(unsigned char rate) {
  _rate = rate;
  _sendCmd(rate & 0x07);
}

bool getSysGainOverRange(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  return ((stat & SYSOR) > 0x00);
}

unsigned char getRate(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  unsigned char rate = ((stat & 0x70) >> 4);
  return rate;
}

bool getOverRange(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  return ((stat & OR) > 0x00);
}

bool getUnderRange(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  return ((stat & UR) > 0x00);
}

bool getMeasStat(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  return ((stat & MSTAT) > 0x00);
}

bool getReady(void) {
  unsigned char addr = STAT1;
  unsigned char stat = _readReg8(addr);
  return ((stat & RDY) > 0x00);
}

void setLineFreq(unsigned char value) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (value == MAX11210_50HZ) {
    data = (stat | LINEF);
    _writeReg8(addr, data);
  }  
  else if (value == MAX11210_60HZ) {
    data = (stat & ~LINEF);
    _writeReg8(addr, data);
  }
}

void setInputRange(unsigned char value) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (value == MAX11210_UNIPOLAR) {
    data = (stat | U);
    _writeReg8(addr, data);
  }  
  else if (value == MAX11210_BIPOLAR) {
    data = (stat & ~U);
    _writeReg8(addr, data);
  }
}

void setClockSource(unsigned char value) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (value == MAX11210_EXTCLK) {
    data = (stat | EXTCLK);
    _writeReg8(addr, data);
  }  
  else if (value == MAX11210_INTCLK) {
    data = (stat & ~EXTCLK);
    _writeReg8(addr, data);
  }  
}

void setEnableRefBuf(bool state) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | REFBUF);
  else
    data = (stat & ~REFBUF);
  _writeReg8(addr, data);
}

void setEnableSigBuf(bool state) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | SIGBUF);
  else
    data = (stat & ~SIGBUF);
  _writeReg8(addr, data);
}

void setFormat(unsigned char value) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (value == MAX11210_OFFSET) {
    data = (stat | FORMAT);
    _writeReg8(addr, data);
  }  
  else if (value == MAX11210_2SCOMPL) {
    data = (stat & ~FORMAT);
    _writeReg8(addr, data);
  }
}

void setConvMode(unsigned char value) {
  unsigned char addr = CTRL1;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (value == MAX11210_SINGLE) {
    _sCycle = true;
    data = (stat | SCYCLE);
    _writeReg8(addr, data);
  }  
  else if (value == MAX11210_CONT) {
    _sCycle = false;
    data = (stat & ~SCYCLE);
    _writeReg8(addr, data);
  }
}

unsigned char getLineFreq(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  if ((stat & LINEF) > 0x00)
    return MAX11210_60HZ;
  else
    return MAX11210_50HZ;
}

unsigned char getInputRange(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  if ((stat & U) > 0x00)
    return MAX11210_UNIPOLAR;
  else
    return MAX11210_BIPOLAR;
}

unsigned char getClockSource(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  if ((stat & EXTCLK) > 0x00)
    return MAX11210_EXTCLK;
  else
    return MAX11210_INTCLK;
}

bool getEnableRefBuf(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  return ((stat & REFBUF) > 0x00);
}

bool getEnableSigBuf(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  return ((stat & SIGBUF) > 0x00);
}

unsigned char getFormat(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  if ((stat & FORMAT) > 0x00)
    return MAX11210_OFFSET;
  else
    return MAX11210_2SCOMPL;
}

unsigned char getConvMode(void) {
  unsigned char addr = CTRL1;
  unsigned char stat = _readReg8(addr);
  if ((stat & SCYCLE) > 0x00)
    return MAX11210_SINGLE;
  else
    return MAX11210_CONT;
}
/*
void pinModeGpio(int pin, unsigned char mode) {
  if (pin < 0 || pin > 3)
    return;
  unsigned char addr = CTRL2;
  unsigned char stat = _readReg8(addr);
  unsigned char data = 0x00;
  if (mode == OUTPUT)
    data = stat | (0x10 << pin);      
  else if (mode == INPUT)
    data = stat ^ (~(0x10 << pin));
  data &= 0xF0;
  _writeReg8(addr, data);
}

void digitalWriteGpio(int pin, bool value) {
  if (pin < 0 || pin > 3)
    return;
  unsigned char addr = CTRL2;
  unsigned char stat = _readReg8(addr);
  unsigned char data = 0x00;
  if (value)
    data = stat | (0x01 << pin);  
  else
    data = stat ^ (~(0x01 << pin));
  data &= 0x0F;  
  _writeReg8(addr, data);
}

bool digitalReadGpio(int pin) {
  if (pin < 0 || pin > 3)
    return false;    
  unsigned char addr = CTRL2;
  unsigned char stat = _readReg8(addr) & (0x01 << pin);
  return (stat > 0x00);
}
*/
void setGain(unsigned char gain) {
  if (gain > 0x04)
    gain = 0x04;
  unsigned char addr = CTRL3;
  unsigned char data = (gain & 0x07) << 5;
  unsigned char stat = _readReg8(addr);
  data |= (stat & 0x1F);
  _writeReg8(addr, data);
}

void setDisableSysGain(bool state) {
  unsigned char addr = CTRL3;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | NOSYSG);
  else
    data = (stat & ~NOSYSG);
  _writeReg8(addr, data);
}

void setDisableSysOffset(bool state) {
  unsigned char addr = CTRL3;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | NOSYSO);
  else
    data = (stat & ~NOSYSO);
  _writeReg8(addr, data);    
}

void setDisableSelfCalGain(bool state) {
  unsigned char addr = CTRL3;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | NOSCG);
  else
    data = (stat & ~NOSCG);
  _writeReg8(addr, data);    
}

void setDisableSelfCalOffset(bool state) {
  unsigned char addr = CTRL3;
  unsigned char data = 0x00;
  unsigned char stat = _readReg8(addr);
  if (state)
    data = (stat | NOSCO);
  else
    data = (stat & ~NOSCO);
  _writeReg8(addr, data);
}

unsigned char getGain(void) {
  unsigned char addr = CTRL3;
  unsigned char stat = _readReg8(addr);
  return ((stat & 0xE0) >> 5);
}
  
bool getDisableSysGain(void) {
  unsigned char addr = CTRL3;
  unsigned char stat = _readReg8(addr);
  return ((stat & NOSYSG) > 0x00);
}
  
bool getDisableSysOffset(void) {
  unsigned char addr = CTRL3;
  unsigned char stat = _readReg8(addr);
  return ((stat & NOSYSO) > 0x00);
}
  
bool getDisableSelfCalGain(void) {
  unsigned char addr = CTRL3;
  unsigned char stat = _readReg8(addr);
  return ((stat & NOSCG) > 0x00);   
}
  
bool getDisableSelfCalOffset(void) {
  unsigned char addr = CTRL3;
  unsigned char stat = _readReg8(addr);
  return ((stat & NOSCO) > 0x00);
}

void setSysGainCal(long value) {
  unsigned char addr = SGC;
  _writeReg24(addr, value);
}    

long getSysGainCal(void) {
  unsigned char addr = SGC;
  return _readReg24(addr);
}

void setSysOffsetCal(long value) {
  unsigned char addr = SOC;
  _writeReg24(addr, value);    
}

long getSysOffsetCal(void) {
  unsigned char addr = SOC;
  return _readReg24(addr);
}
  
void setSelfCalGain(long value) {
  unsigned char addr = SCGC;
  _writeReg24(addr, value);    
}

long getSelfCalGain(void) {
  unsigned char addr = SCGC;
  return _readReg24(addr);
}
  
void setSelfCalOffset(long value) {
  unsigned char addr = SCOC;
  _writeReg24(addr, value);    
}

long getSelfCalOffset(void) {
  unsigned char addr = SCOC;
  return _readReg24(addr);
}
