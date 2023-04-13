#include <msp430.h> 

volatile int j = 0;
volatile char pkt[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char tpkt[sizeof(pkt)];

volatile int action_select = 0;
volatile int action_count = 0;
volatile int ms_thresh, ms_count, ms_flag;

void initI2C_slave(){
    UCB0CTLW0 |= UCSWRST;               // SW RESET enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;     // Put into I2C mode
    UCB0I2COA0 = 0x0068 | UCOAEN;       // Set slave address + enable

    UCB0CTLW1 |= 0xC0;             // Set UCCLTO = 11 (~34ms clock low timeout)

    // Setup ports
    P1SEL1 &= ~BIT3;            // P1.3 SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 SDA
    P1SEL0 |= BIT2;

    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF

    UCB0IE |= UCTXIE0 | UCRXIE0 | UCSTPIE | UCCLTOIE | UCBCNTIE | UCNACKIE;          // Enable I2C B0 Tx/Rx/Stop IRQs

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O
    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF
}

void initTimerB0compare(){
    // Setup TB0
    TB0CTL |= TBCLR;            // Clear TB0
    TB0CTL |= TBSSEL__SMCLK;    // Select SMCLK
    TB0CTL |= MC__UP;           // UP mode

    TB0CCR0 = 1045;             // Set CCR0 value (Period = 1045 for 1ms)
    TB0CCTL0 &= ~CCIFG;         // Clear CCR0 flag
}


void init(){
    WDTCTL = WDTPW | WDTHOLD;                   // Stop watchdog timer

    P1DIR |= (BIT4 | BIT5 | BIT6 | BIT7);       // Set P1.4 Output for DB4-DB7 (LCD)
    P1OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);      // Clear P1.4-7

    P2DIR |= (BIT7 | BIT6);      // Set P2.7 output for RS (LCD) - (RS = 0 instruction, RS = 1 data)
    P2OUT &= ~(BIT7 | BIT6);     // Clear P2.7

//    P2DIR |= BIT6;      // Set P2.6 output for E (LCD), data starts on falling edge
//    P2OUT &= ~BIT6;     // Clear P2.6

    P1DIR |= BIT1;      // P1.1 LED output
    P1OUT &= ~BIT1;     // Clear P1.1

    initI2C_slave();
    initTimerB0compare();

    __enable_interrupt();
}


void delay1000(){
    int i;
    for(i=0;i<=1000;i++){}
}

void configure(){
    P2OUT &= ~BIT7;                     // Clear RS
    P1OUT |= (BIT4 | BIT5);             // Set P1.4-5
    P1OUT &= ~(BIT6 | BIT7);            // P1.6-7
}

void configure2(){
    P2OUT &= ~BIT7;                     // Clear RS
    P1OUT |= BIT5;                      // Set P1.5
    P1OUT &= ~(BIT4 | BIT6 | BIT7);     // Clear P1.4, P1.6-7
}

void latch(){
    P2OUT |= BIT6;              // Set E bit (LCD)
    delay1000();
    P2OUT &= ~BIT6;             // Reset E bit (LCD)
}

void delay_ms(int ms){
    ms_flag = 0;
    ms_count = 0;
    ms_thresh = ms;
    TB0CCTL0 |= CCIE;           // Local IRQ enable for CCR0
    while(ms_flag != 1){}
    TB0CCTL0 &= ~CCIE;          // Disable CCR0
}

void clear_display(){
    sendByte(0b00000001, 0);
}

void setNibbleBit(int n) {
    switch(n){
        case 3:
            P1OUT |= BIT7;
            break;
        case 2:
            P1OUT |= BIT6;
            break;
        case 1:
            P1OUT |= BIT5;
            break;
        case 0:
            P1OUT |= BIT4;
            break;
    }
}

void clearNibbleBit(int n) {
    switch(n){
        case 3:
            P1OUT &= ~BIT7;
            break;
        case 2:
            P1OUT &= ~BIT6;
            break;
        case 1:
            P1OUT &= ~BIT5;
            break;
        case 0:
            P1OUT &= ~BIT4;
            break;
    }
}

void sendNibble(unsigned short byte, int rs) {
    if(rs == 1){
        P2OUT |= BIT7;      // Set RS
    } else {
        P2OUT &= ~BIT7;     // Set RS
    }

    int k,n;
    volatile int test;
    n = 1;

    for(k=0;k<=3;k++){
        test = byte & n;
        if(test>0){
            setNibbleBit(k);
        } else {
            clearNibbleBit(k);
        }
        n = n << 1;
    }
    latch();
    delay_ms(2);
}

void sendByte(int char_code, int rs){
    sendNibble(char_code >> 4, rs);
    sendNibble(char_code, rs);
}

void setCursorSecondRow(){
    sendByte(0b10101000, 0);
}

void LCDsetup() {

    int i;
    for(i=0;i<=2;i++){
        configure();
        latch();
        delay_ms(10);
    }

    configure2();               // Funcset interface 4 bit
    latch();
    delay_ms(10);

    sendByte(0b00101100, 0);    // Function set interface 4-bit & spec display lines and fonts

    sendByte(0b00001100, 0);    // Display on (cursor not shown)

    sendByte(0b00000001, 0);    // Clear display

    sendByte(0b00000110, 0);    // Entry mode set

}

int getCharCode(int in) {
    int ret;
    switch(in) {
        case 0x01: // 1
            ret =  0b00110001;
            break;
        case 0x02: // 2
            ret =  0b00110010;
            break;
        case 0x03: // 3
            ret =  0b00110011;
            break;
        case 0x04: // 4
            ret =  0b00110100;
            break;
        case 0x05: // 5
            ret =  0b00110101;
            break;
        case 0x06: // 6
            ret =  0b00110110;
            break;
        case 0x07: // 7
            ret =  0b00110111;
            break;
        case 0x08: // 8
            ret =  0b00111000;
            break;
        case 0x09: // 9
            ret =  0b00111001;
            break;
        case 0x17: // *
            ret =  0b00101010;
            break;
        case 0x00: // 0
            ret =  0b00110000;
            break;
        case 0x12: // .
            ret = 0b00101110;
            break;
        case 0x11: // #
            ret =  -1;
            break;
        case 0x80:
            ret = 0b01000001;
            break;
        case 0x40:
            ret = 0b01000010;
            break;
        case 0x20:
            ret = 0b01000011;
            break;
        case 0x10:
            ret = 0b01000100;
            break;
        default:
            ret = 0;
            break;
    }
    return ret;
}


void LCDstartDisplay() {
    int i;

    sendByte(0b01010010, 1);        // Display R
    sendByte(0b01100101, 1);        // Display e
    sendByte(0b01110011, 1);        // Display s
    sendByte(0b00111101, 1);        // Display =

    shiftCurFwd(4);

    sendByte(0b01000001, 1);        // Display A
    sendByte(0b00111010, 1);        // Display :

    shiftCurFwd(4);

    sendByte(0b11011111, 1);        // Display °
    sendByte(0b01000011, 1);        // Display C
    setCursorSecondRow();
    shiftCurFwd(1);
    sendByte(0b00111101, 1);        // Display =

    shiftCurFwd(3);

    sendByte(0b01110011, 1);        // Display s

    shiftCurFwd(2);

    sendByte(0b01010000, 1);        // Display P
    sendByte(0b00111010, 1);        // Display :

    shiftCurFwd(4);

    sendByte(0b11011111, 1);        // Display °
    sendByte(0b01000011, 1);        // Display C

}

renderPkt(char* bytes, int start, int stop) {
    int m;
    int charCount = 0;
//    UCB0IE &= ~(UCTXIE0 | UCRXIE0 | UCSTPIE);   // Disable
    for(m=start;m<=stop;m++){
        int code = getCharCode(bytes[m]);
        sendByte(code, 1);                      // Display character
        charCount++;
    }
//    UCB0IE |= (UCTXIE0 | UCRXIE0 | UCSTPIE);    // Enable
}

void shiftCurFwd(int f){
    int i;
    for(i = 0; i < f; i++) {
        sendByte(0b00010100, 0);
    }
}

void shiftCurBkwd(){
    sendByte(0b00010000, 0);
}

void returnHome(){
    sendByte(0b00000010, 0);
}

int main(void)
{
    int charCount = 0;
    int i;

init();
    delay_ms(20);
    LCDsetup();

    clear_display();

    LCDstartDisplay();

    action_select = 1;

    while(1){

        if(action_select == 1){                 // Perform action based on received data from master (display temperature, reset screen, toggle LED)

            for(i = 0;i<(sizeof(pkt));i++){
                tpkt[i] = pkt[i];
            }

            int code = getCharCode(tpkt[0]);
            if(code == -1){                      // Clear LCD and reset default display if '#' received
                code = 0;
                clear_display();
                LCDstartDisplay();
                charCount = 0;
            } else if(code != 0){

//                P1OUT |= BIT1;              // Turn on LED if any other character pressed
//                char p = pkt[9];
//                if((pkt[9] != 0x80) && (pkt[9] != 0x40) && (pkt[9] != 0x20) && (pkt[9] != 0x10) && (pkt[9] != 0x00)) {
//                    P1SEL0 &=  ~(BIT2);
//                    P1SEL0 &=  ~(BIT3);
//                    UCB0CTLW0 |= UCSWRST;               // SW RESET enabled
//                    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF
//                    P1SEL0 |=  (BIT3|BIT2); // Re-connect pins to I2C
//                    UCB0IE &= ~(UCTXIE0 | UCRXIE0 | UCSTPIE);   // Disable
//                    UCB0IE |= (UCTXIE0 | UCRXIE0 | UCSTPIE);    // Enable
//                }

                  // renderPkt(tpkt, 0, 10); testing

                shiftCurFwd(4);
                sendByte(getCharCode(tpkt[10]), 1);    // Display n value
                shiftCurFwd(5);
                renderPkt(tpkt, 0, 1);              // Display ambient temperature
                sendByte(getCharCode(0x12), 1);     // .
                sendByte(getCharCode(tpkt[2]), 1);  // tenths place

                setCursorSecondRow();
                sendByte(getCharCode(tpkt[9]), 1);    // Display M
                shiftCurFwd(1);
                renderPkt(tpkt, 6, 8);                     // Display time
                shiftCurFwd(5);

                renderPkt(tpkt, 3, 4);              // Display plant temperature
                sendByte(getCharCode(0x12), 1);     // .
                sendByte(getCharCode(tpkt[5]), 1);  // tenths place
                returnHome();

                action_select = 0;          // End action

            }

        }
    }

    return 0;
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){

    switch(UCB0IV){
        case 0x06: // start condition

            break;
        case 0x08:  // stop condition
            j = 0;
            action_select = 1;       // Display temperature or perform action for #/*
            break;
        case 0x16:                           // Receiving
//            if(j == sizeof(pkt)-1){
//               action_select = 1;       // Display temperature or perform action for #/*
//            }
            pkt[j] = UCB0RXBUF;      // Retrieve byte from buffer
            j++;

            break;
        case 0x18:
            break;
        case 0x1C: // clock low timeout
//            UCB0CTLW0 |= UCTXNACK;           // Generate START condition
            //initI2C_slave();
            break;
    }

    //UCB0IFG &= ~UCTXIFG0;                   // Clear flag to allow I2C interrupt
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    ms_count++;
    if(ms_count == ms_thresh){
        ms_flag = 1;
        ms_count = 0;
    }

    TB0CCTL0 &= ~CCIFG;
}
