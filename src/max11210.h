#ifndef _MAXIM_MAX11210
  #define _MAXIM_MAX11210

  //#include "application.h"
  
  // MAX11210 8-bit Command unsigned char
  // ------------------------------------------------------------------------
  // name     B7         B6         B5     B4     B3     B2     B1     B0 
  // ------------------------------------------------------------------------
  // COMMAND  START = 1  MODE = 0   CAL1   CAL0   IMPD   RATE2  RATE1  RATE0
  // COMMAND  START = 1  MODE = 1   0      RS3    RS2    RS1    RS0    R/!W
  #define START 0b10000000
  #define MODE  0b01000000
  #define CAL1  0b00100000
  #define CAL0  0b00010000
  #define IMPD  0b00001000
  #define READ  0b00000001
  
  const unsigned char MAX11210_RATE1   = 0x00;
  const unsigned char MAX11210_RATE2_5 = 0x01;
  const unsigned char MAX11210_RATE5   = 0x02;
  const unsigned char MAX11210_RATE10  = 0x03;
  const unsigned char MAX11210_RATE15  = 0x04;
  const unsigned char MAX11210_RATE30  = 0x05;
  const unsigned char MAX11210_RATE60  = 0x06;
  const unsigned char MAX11210_RATE120 = 0x07;
  
  // MAX11210 Status & control registers
  // ------------------------------------------------------------------------
  //     name     B7     B6     B5     B4     B3     B2     B1     B0
  // ------------------------------------------------------------------------
  // 0x0 STAT1    SYSOR  RATE2  RATE1  RATE0  OR     UR     MSTAT  RDY
  // 0x1 CTRL1    LINEF  U/!B   EXTCLK REFBUF SIGBUF FORMAT SCYCLE RESERVED
  // 0x2 CTRL2    DIR4   DIR3   DIR2   DIR1   DIO4   DIO3   DIO2   DIO1
  // 0x3 CTRL3    DGAIN2 DGAIN1 DGAIN0 NOSYSG NOSYSO NOSCG  NOSCO  RESERVED
  //
  // 0x4 DATA     D[23:16], D[15:8], D[7:0]
  // 0x5 SOC      B[23:16], B[15:8], B[7:0]
  // 0x6 SGC      B[23:16], B[15:8], B[7:0]
  // 0x7 SCOC     B[23:16], B[15:8], B[7:0]
  // 0x8 SCGC     B[23:16], B[15:8], B[7:0]
  // ------------------------------------------------------------------------
  #define STAT1  0x00
  #define CTRL1  0x01
  #define CTRL2  0x02
  #define CTRL3  0x03
  #define DATA   0x04
  #define SOC    0x05
  #define SGC    0x06
  #define SCOC   0x07
  #define SCGC   0x08
  
  // STAT1
  #define SYSOR  0b10000000
  #define OR     0b00001000
  #define UR     0b00000100
  #define MSTAT  0b00000010
  #define RDY    0b00000001

  // CTRL1
  #define LINEF  0b10000000
  #define U      0b01000000
  #define EXTCLK 0b00100000
  #define REFBUF 0b00010000
  #define SIGBUF 0b00001000
  #define FORMAT 0b00000100
  #define SCYCLE 0b00000010
  
  const unsigned char MAX11210_50HZ     = 0x01;
  const unsigned char MAX11210_60HZ     = 0x00;
  const unsigned char MAX11210_UNIPOLAR = 0x01;
  const unsigned char MAX11210_BIPOLAR  = 0x00;
  const unsigned char MAX11210_EXTCLK   = 0x01;
  const unsigned char MAX11210_INTCLK   = 0x00;
  const unsigned char MAX11210_OFFSET   = 0x01;
  const unsigned char MAX11210_2SCOMPL  = 0x00;
  const unsigned char MAX11210_SINGLE   = 0x01;
  const unsigned char MAX11210_CONT     = 0x00;

  // CTRL3
  const unsigned char MAX11210_GAIN1  = 0x00;
  const unsigned char MAX11210_GAIN2  = 0x01;
  const unsigned char MAX11210_GAIN4  = 0x02;
  const unsigned char MAX11210_GAIN8  = 0x03;
  const unsigned char MAX11210_GAIN16 = 0x04;
  
  #define NOSYSG 0b00010000
  #define NOSYSO 0b00001000
  #define NOSCG  0b00000100
  #define NOSCO  0b00000010

//  namespace Maxim {
//    class Max11210 {
//      private:
unsigned char spitransfer(unsigned char data);
unsigned char _rate;
bool _sCycle;
unsigned char _readReg8(unsigned char addr);
long _readReg24(unsigned char addr);
long _readReg24s(unsigned char addr);
void _writeReg8(unsigned char addr, unsigned char data);
void _writeReg24(unsigned char addr, long data);
void _sendCmd(unsigned char data);
            
//      public:
//        Max11210();
void max_begin(void);
void max_end(void);
long max_read(void);

// CMD MODE 0
void selfCal(void);
void sysOffsetCal(void);
void sysGainCal(void);
void setRate(unsigned char rate);
// rate = {MAX11210_RATE1, MAX11210_RATE2_5, MAX11210_RATE5, MAX11210_RATE10, MAX11210_RATE15, MAX11210_RATE30, MAX11210_RATE60, MAX11210_RATE120}

// STAT1
bool getSysGainOverRange(void);
unsigned char getRate(void);
bool getOverRange(void);
bool getUnderRange(void);
bool getMeasStat(void); 
bool getReady(void);

// CTRL1
void setLineFreq(unsigned char value);    // value = {MAX11210_50HZ, MAX11210_60HZ}
void setInputRange(unsigned char value);  // value = {MAX11210_UNIPOLAR, MAX11210_BIPOLAR}
void setClockSource(unsigned char value); // value = {MAX11210_EXTCLK, MAX11210_INTCLK}
void setEnableRefBuf(bool state);
void setEnableSigBuf(bool state);
void setFormat(unsigned char value);      // value = {MAX11210_OFFSET, MAX11210_2SCOMPL}
void setConvMode(unsigned char value);    // value = {MAX11210_SINGLE, MAX11210_CONT}

unsigned char getLineFreq(void);
unsigned char getInputRange(void);
unsigned char getClockSource(void);
bool getEnableRefBuf(void);
bool getEnableSigBuf(void);
unsigned char getFormat(void);
unsigned char getConvMode(void);

// CTRL2
void pinModeGpio(int pin, unsigned char mode);
void digitalWriteGpio(int pin, bool value);
bool digitalReadGpio(int pin);

// CTRL3
void setGain(unsigned char gain);
// gain = {MAX11210_GAIN1, MAX11210_GAIN2, MAX11210_GAIN4, MAX11210_GAIN8, MAX11210_GAIN16}
void setDisableSysGain(bool state);
void setDisableSysOffset(bool state);
void setDisableSelfCalGain(bool state);
void setDisableSelfCalOffset(bool state);

unsigned char getGain(void);
bool getDisableSysGain(void);
bool getDisableSysOffset(void);
bool getDisableSelfCalGain(void);
bool getDisableSelfCalOffset(void);

//
void setSysGainCal(long value);
void setSysOffsetCal(long value);
void setSelfCalGain(long value);
void setSelfCalOffset(long value);

long getSysGainCal(void);
long getSysOffsetCal(void);
long getSelfCalGain(void);        
long getSelfCalOffset(void);
//    };
//  };
  
#endif
