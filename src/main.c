/*
 * main.c
 *
 *  Created on: Oct 6, 2017
 *      Author: Robert.Chapman
 */

#include <msp430.h>
#include <stdint.h>
#include <string.h>

typedef struct Joystick {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t crc;
} Joystick;

/* SPI Config */
#define JOY_SPI_CLK_PIN         BIT0
#define JOY_SPI_MOSI_PIN        BIT1
#define JOY_SPI_MISO_PIN        BIT2
#define JOY_SPI_BAUD_CNTRL      UCB1BRW
#define JOY_SPI_CNTLR           UCB1CTLW0
#define JOY_SPI_INT_ENABLE      UCB1IE
#define JOY_SPI_PORT_SEL_1      P5SEL1
#define JOY_SPI_PORT_SEL_0      P5SEL0
#define JOY_SPI_CS_PIN          BIT4
#define JOY_SPI_CS_PORT_SEL     P4SEL1
#define JOY_SPI_CS_PORT_DIR     P4DIR
#define JOY_SPI_CS_PORT_OUT     P4OUT

#define FN(x)                   do { x } while (__LINE__ == -1)
#define JOY_SPI_TX(x)           FN( UCB1IFG &= ~UCRXIFG; UCB1TXBUF= (x); )
#define JOY_SPI_WAIT_DONE()     FN( while(!(UCB1IFG & UCRXIFG)); )
#define JOY_SPI_RX()            UCB1RXBUF

static void initSpi(void);
static void initClock(void);
static void stopWatchdog(void);
static void collectJoystickCoords(void);

volatile uint32_t count = 0;
volatile Joystick joystick = {0};
volatile uint8_t rxBuffer[1024] = {0};

int main(void) {

    stopWatchdog();
    initClock();
    initSpi();
    collectJoystickCoords();
}

static void initClock(void) {

    /* For XT1 */
    PJSEL0 |= BIT4 | BIT5;
    /* Disable the GPIO power-on default high-impedance mode */
    PM5CTL0 &= ~LOCKLPM5;
    /* Unlock CS registers */
    CSCTL0_H = CSKEY_H;
    /* Set DCO to 1MHz */
    CSCTL1 = DCOFSEL_1;
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    /* set all dividers */
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL4 &= ~LFXTOFF;
    do {
        /* Clear XT1 fault flag */
        CSCTL5 &= ~LFXTOFFG;
        SFRIFG1 &= ~OFIFG;
    /* Test oscillator fault flag */
    } while (SFRIFG1 & OFIFG);
    /* Lock CS registers */
    CSCTL0_H = 0;
}

static void initSpi(void) {

    /* USCI_B1 SCLK, MOSI, and MISO pin */
    JOY_SPI_PORT_SEL_1 &= ~(JOY_SPI_CLK_PIN | JOY_SPI_MOSI_PIN | JOY_SPI_MISO_PIN);
    JOY_SPI_PORT_SEL_0 |= (JOY_SPI_CLK_PIN | JOY_SPI_MOSI_PIN | JOY_SPI_MISO_PIN);
    /* Put state machine in reset */
    JOY_SPI_CNTLR = UCSWRST;
    /* 3-pin, 8-bit SPI master, clock polarity high, MSB */
    JOY_SPI_CNTLR |= UCMST | UCSYNC | UCMSB;
    /* MLX90333 specific. The inactive clock state is low. Data is changed on the first
       UCLK edge and captured on the following edge */
    JOY_SPI_CNTLR |= UCCKPL_0 | UCCKPH_0;
    /* ACLK clock source */
    JOY_SPI_CNTLR |= UCSSEL__SMCLK;
    JOY_SPI_BAUD_CNTRL = 58;
    /* Initialize USCI state machine */
    JOY_SPI_CNTLR &= ~UCSWRST;
    /* Chip select */
    JOY_SPI_CS_PORT_SEL &= ~JOY_SPI_CS_PIN;
    JOY_SPI_CS_PORT_DIR |= JOY_SPI_CS_PIN;
    JOY_SPI_CS_PORT_OUT |= JOY_SPI_CS_PIN;
    /* Enable all interrupts */
    __enable_interrupt();
}

static void stopWatchdog(void) {

    /* Hold timer so chip doesn't reset */
    WDTCTL = WDTPW | WDTHOLD;
}

static void collectJoystickCoords(void) {

    /* Let the hall effect settle just in case */
    __delay_cycles(10000);
    for (int i = 0; i < 3; i++) {
        UCB1IE |= UCTXIE;
        __delay_cycles(2000);
    }
    /* For debugging */
    __no_operation();
}

#pragma vector=EUSCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void) {

    switch(__even_in_range(UCB1IV, USCI_SPI_UCTXIFG)) {
        case USCI_NONE: break;
        case USCI_SPI_UCTXIFG:
            JOY_SPI_CS_PORT_OUT &= ~JOY_SPI_CS_PIN;
            for (int i = 0; i < 8; i++) {
                JOY_SPI_TX(0xFF);
                JOY_SPI_WAIT_DONE();;
                rxBuffer[count++] = (uint8_t)JOY_SPI_RX();
                __delay_cycles(400);
            }
            JOY_SPI_CS_PORT_OUT |= JOY_SPI_CS_PIN;
            __delay_cycles(2000);
            UCB1IE &= ~UCTXIE;
            break;
        default: break;
    }
}
