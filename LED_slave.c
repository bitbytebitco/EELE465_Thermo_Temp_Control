#include <msp430.h> 

int Rx_Command;
volatile int timer_action_select;           // Determine selected pattern to display
volatile int counter = 0;


void initI2C_slave(){
    UCB0CTLW0 |= UCSWRST;                   // SW RESET enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;         // Put into I2C mode
    UCB0I2COA0 = 0x0058 | UCOAEN;           // Set slave address + enable

    // Setup ports
    P1SEL1 &= ~BIT3;            // P1.3 SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 SDA
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O
    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF

    UCB0IE |= UCTXIE0 | UCRXIE0 | UCSTPIE;          // Enable I2C B0 Tx/Rx/Stop IRQs

    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF
}

void setLEDn(int n) {

    switch(n) {
        case 7:
            P1OUT |= BIT7;
            break;
        case 6:
            P1OUT |= BIT6;
            break;
        case 5:
            P2OUT |= BIT6;
            break;
        case 4:
            P2OUT |= BIT7;
            break;
        case 3:
            P1OUT |= BIT5;
            break;
        case 2:
            P1OUT |= BIT4;
            break;
        case 1:
            P1OUT |= BIT0;
            break;
        case 0:
            P1OUT |= BIT1;
            break;
    }
}

void ResetLED() {
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT6 | BIT7);
}

void PressA() {                     // Increasing pattern for A/heating mode
    setLEDn(counter);
}

void PressB() {                     // Decreasing pattern for B/cooling mode
    setLEDn(7-counter);
}

void PressC_heating() {             // Step increasing pattern for C/heating mode
    if(counter % 2 == 0) {
        setLEDn(counter + 1);
    } else {
        setLEDn(counter - 1);
    }
}

void PressC_cooling() {             // Step decreasing pattern for C/cooling mode
    if(counter % 2 == 0) {
        setLEDn(7-counter-1);
    } else {
        setLEDn(7-counter+1);
    }
}

void PressD() {                     // Alternating pattern for D/off mode
    ResetLED();
    if(counter % 2 == 0) {
        setLEDn(0);
        setLEDn(2);
        setLEDn(4);
        setLEDn(6);
    } else {
        ResetLED();
        setLEDn(1);
        setLEDn(3);
        setLEDn(5);
        setLEDn(7);
    }
}

void initTimerB0compare(){
    // Setup TB0
    TB0CTL |= TBCLR;            // Clear TB0
    TB0CTL |= TBSSEL__ACLK;     // Select ACLK
    TB0CTL |= MC__UP;           // UP mode

    TB0CCR0 = 10666;            // Set CCR0 value (period = 333ms)
    TB0CCTL0 &= ~CCIFG;         // Clear CCR0 flag
    TB0CCTL0 |= CCIE;           // Local IRQ enable for CCR0
}

void init() {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Setup LED outputs
    P1DIR |= (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);

    P2DIR |= (BIT6 | BIT7);
    P2OUT &= ~(BIT6 | BIT7);

    P2DIR |= BIT0;              // P2.0 LED output
    P2OUT &= ~BIT0;             // P2.0 off

    timer_action_select= 0 ;

    initI2C_slave();            // Temporary for hard code testing
    initTimerB0compare();

    PM5CTL0 &= ~LOCKLPM5;       // Turn on digital I/O

    __enable_interrupt();
}

int main(void) {

    init();

    while(1){}

    return 0;
}


void executeCommand() {
    if(Rx_Command == 0x80 && timer_action_select != 1) {            // Pattern A
        P2OUT ^= BIT0;
        timer_action_select = 1;
        counter = 0;
        ResetLED();
    } else if(Rx_Command == 0x40 && timer_action_select != 2) {     // Pattern B
        P2OUT ^= BIT0;
        timer_action_select = 2;
        counter = 0;
        ResetLED();
    } else if(Rx_Command == 0x17 && timer_action_select != 3) {     // Pattern C heating
        P2OUT ^= BIT0;
        timer_action_select = 3;
        counter = 0;
        ResetLED();
    } else if(Rx_Command == 0x11 && timer_action_select != 4) {     // Pattern C cooling
        P2OUT ^= BIT0;
        timer_action_select = 4;
        counter = 0;
        ResetLED();
    } else if(Rx_Command == 0x10 && timer_action_select != 5) {     // Pattern D
        P2OUT ^= BIT0;
        timer_action_select = 5;
        counter = 0;
        ResetLED();
    }
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    switch(UCB0IV){
        case 0x16:                                                  // Receiving

            Rx_Command = UCB0RXBUF;                                 // Retrieve byte from buffer

            executeCommand();                                       // Determine pattern using byte received

            UCB0IFG &= ~UCTXIFG0;                                   // Clear flag to allow I2C interrupt
            break;

    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){

    if(timer_action_select == 1) {              // Update pattern A upon interrupt
        PressA();
    } else if(timer_action_select == 2) {       // Update pattern B upon interrupt
        PressB();
    } else if(timer_action_select == 3) {       // Update pattern C heating upon interrupt
        PressC_heating();
    } else if(timer_action_select == 4) {       // Update pattern C cooling upon interrupt
        PressC_cooling();
    } else if(timer_action_select == 5) {
        PressD();                               // Update pattern D upon interrupt
    }

    if(counter == 8 && timer_action_select != 5) {      // Reset counter when it reaches 8 for all modes except pattern D
        counter = 0;
        ResetLED();
    } else {                                            // Increment counter if it is less than 8
        counter++;
    }

    TB0CCTL0 &= ~CCIFG;                                 // Clear TB0 flag
}
