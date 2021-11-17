#include <libopencm3/sam/memorymap.h>
#include <libopencm3/sam/d/port.h>
#include <libopencm3/sam/d/usart.h>
#include <libopencm3/sam/d/pm.h>
#include <libopencm3/sam/d/gclk.h>
#include <libopencm3/sam/d/sysctrl.h>
#include <libopencm3/sam/d/spi.h>
//#include "max11210.h"
#include "max11210.c"

#define PORT_LED PORTA
#define PIN_LED 2
#define PIN_CS 5

static void sysctrl_setup(void);
static void usart_setup(void);
static void pm_setup(void);
static void gpio_setup(void);
static void spi_setup(void);
void usart_tx8(unsigned char mesg);
void usart_tx32(unsigned long mesg);
void mydel(unsigned int ms);
void myprintln(unsigned long mesg);

int main(void) {

    long i;
    int a = 0;
    unsigned char data = 0;
    long datas = 0;
    sysctrl_setup();
    gpio_setup();
    pm_setup();
    usart_setup();
    spi_setup();
    max_begin();
    mydel(200);
    long selfcalgain = getSelfCalGain();
    long selfcaloffset = getSelfCalOffset();
    usart_tx32(selfcalgain);
    setLineFreq(MAX11210_50HZ);
    mydel(10);
    setConvMode(MAX11210_CONT);
    mydel(10);
    setLineFreq(MAX11210_50HZ);
    mydel(10);
    setConvMode(MAX11210_CONT);
    mydel(10);
    setInputRange(MAX11210_BIPOLAR);
    mydel(10);
    setFormat(MAX11210_OFFSET);
    mydel(10);
    setRate(0x05);
    mydel(10);
    usart_tx8(getInputRange());
    usart_tx8(getFormat());
    usart_tx8(getConvMode());
    usart_tx8(getRate());

    while (1) {
        PORT_OUTTGL(PORT_LED) = (1 << PIN_LED);
        PORT_OUTCLR(PORTA) = (1 << PIN_CS);
        while(gpio_get(PORTA, (1<<4))==(1<<4));
        unsigned long tmp = max_read();
        //char temp[8];
        //sprintf((char*)temp,"%u",tmp);
        //for 
        myprintln(tmp);
        //usart_tx32(tmp);
        mydel(1);
    }
/*
 
  digitalWrite(SSS, LOW);
  while(digitalRead(12)==HIGH){}
  long tmp = adcMax11210.read();
  Serial.println(tmp);
  if(digitalRead(8)==HIGH){
      digitalWrite(8, LOW);
    }else{
      digitalWrite(8, HIGH);
    }
 */
    return 0;

}

void usart_tx8(unsigned char mesg){
        while((USART_INTFLAG(SERCOM1_BASE)&USART_INTFLAG_DRE)!= USART_INTFLAG_DRE){}
        USART_DATA(SERCOM1_BASE) = mesg&0xff;
}

void myprintln(unsigned long mesg){
    unsigned long num = mesg;
    unsigned long power = 1;
    while(num>power){
        power=power*10;
    }
    power=power/10;
    while(num != 0){
        int digit = num/power;
        //send digit
        if(digit!=0){
            num=num-digit*power;
        }
        if(power!=1){
            power=power/10;
        }
        usart_tx8((digit+48));
    }
    //send \n
    usart_tx8(10);
}

void mydel(unsigned int ms){
    unsigned int n = 0;
    for(n = ms;n>0;n--){
        for(int i=0;i<1530;i++){
            __asm__("nop");
        }
    }
}

void usart_tx32(unsigned long mesg){
        if((USART_INTFLAG(SERCOM1_BASE)&USART_INTFLAG_DRE) == USART_INTFLAG_DRE){
            USART_DATA(SERCOM1_BASE) = mesg&0xff;
        }
        mydel(1);
        if((USART_INTFLAG(SERCOM1_BASE)&USART_INTFLAG_DRE) == USART_INTFLAG_DRE){
            USART_DATA(SERCOM1_BASE) = (mesg>>8)&0xff;
        }
        mydel(1);
        if((USART_INTFLAG(SERCOM1_BASE)&USART_INTFLAG_DRE) == USART_INTFLAG_DRE){
            USART_DATA(SERCOM1_BASE) = (mesg>>16)&0xff;
        }
        mydel(1);
        if((USART_INTFLAG(SERCOM1_BASE)&USART_INTFLAG_DRE) == USART_INTFLAG_DRE){
            USART_DATA(SERCOM1_BASE) = (mesg>>24)&0xff;
        }

}

static void sysctrl_setup(void){
    //removing prescaler for 8mhz osc
   SYSCTRL_OSC8M = 0x87070082;
   //SYSCTRL_OSC8M |= 0x0030;
}

static void pm_setup(void){


    //enable sercom1 sercom0 adc clocks
    PM_APBCMASK = PM_APBCMASK_ADC|PM_APBCMASK_SERCOM1|PM_APBCMASK_SERCOM0;


   //PM_APBCMASK(PM_BASE) |= 0b1100; 
   //enable generic clock 0 for the sercom1 clock module
   GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN|GCLK_SERCOM1_CORE*GCLK_CLKCTRL_ID;
   //enable generic clock 0 for the sercom0 clock module
   GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN|GCLK_SERCOM0_CORE*GCLK_CLKCTRL_ID;
   

}

static void usart_setup(void){
    //disable usart
    //USART_CTRLA(SERCOM1_BASE) &= !USART_CTRLA_ENABLE;
    //internal clock
    USART_CTRLA(SERCOM1_BASE) |= USART_CTRLA_MODE|USART_CTRLA_DORD;
    //asynchronous comms
    //USART_CTRLA(SERCOM1_BASE) &= !USART_CTRLA_CMODE;
    //rx pad [3]
    USART_CTRLA(SERCOM1_BASE) |= USART_CTRLA_RXPO*0x3;
    //tx pad [2]
    USART_CTRLA(SERCOM1_BASE) |= USART_CTRLA_TXPO*0x1;
    //chsize = 8
    //USART_CTRLB(SERCOM1_BASE) &= !USART_CTRLB_CHSIZE;
    //MSB data order
    //USART_CTRLA(SERCOM1_BASE) &= !USART_CTRLA_DORD;
    //set baud for 115200
    USART_BAUD(SERCOM1_BASE) = 50436;
    //enable usart
    USART_CTRLB(SERCOM1_BASE) |= USART_CTRLB_RXEN|USART_CTRLB_TXEN;
    
    USART_CTRLA(SERCOM1_BASE) |= USART_CTRLA_ENABLE;
    //while(USART_SYNCBUSY(SERCOM1_BASE)!=0){};

}

static void spi_setup(void){
    static uint32_t ctrla =   SPI_CTRLA_MODE*0x03 | 
                              SPI_CTRLA_CPOL*0 | 
                              SPI_CTRLA_CPHA*0 | 
                              SPI_CTRLA_FORM*0 |
                              SPI_CTRLA_DIPO*0x2 |
                              SPI_CTRLA_DOPO*0 |
                              SPI_CTRLA_DORD*0;

    SPI_CTRLA(SERCOM0_BASE) = ctrla;
    SPI_CTRLB(SERCOM0_BASE) = 0x00;
    SPI_BAUD(SERCOM0_BASE) = 100; //100khz
    SPI_CTRLB(SERCOM0_BASE) = SPI_CTRLB_RXEN;
    SPI_CTRLA(SERCOM0_BASE) |= SPI_CTRLA_ENABLE;
}

static void gpio_setup(void) {
    gpio_set_af(PORTA,PORT_PMUX_FUN_C,(GPIO4|GPIO14|GPIO15|GPIO24|GPIO25));
    PORT_DIRSET(PORTA) = GPIO2|GPIO5|GPIO14|GPIO15|GPIO24;
    PORT_DIRCLR(PORTA) = GPIO4|GPIO25;

    //PORT_PINCFG(PORT_LED, PIN_LED) = 0;

}
/*
unsigned char spitransfer(unsigned char data){
        SPI_DATA(SERCOM0_BASE) = data;
        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_TXC) != SPI_INTFLAG_TXC){} 
        while((SPI_INTFLAG(SERCOM0_BASE)&SPI_INTFLAG_RXC) != SPI_INTFLAG_RXC){} 
        unsigned char resp = SPI_DATA(SERCOM0_BASE);
        return resp;
}
*/
