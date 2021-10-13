#include <libopencm3/sam/memorymap.h>
#include <libopencm3/sam/d/port.h>

#define PORT_LED PORTA
#define PIN_LED 2

static void gpio_setup(void) {

    PORT_DIR(PORT_LED) = (1 << PIN_LED);
    PORT_PINCFG(PORT_LED, PIN_LED) = 0;

}

int main(void) {

    int i;

    gpio_setup();

    while (1) {

        PORT_OUTTGL(PORT_LED) = (1 << PIN_LED);

        for (i=0; i<100000; i++) {
            __asm__("nop");
        }

    }

    return 0;

}

