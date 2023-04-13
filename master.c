#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int j = 0;
int r = 0;

volatile int i, n, sampleCount;
volatile int sampling_active = 0;
volatile float avg;
volatile float unrounded_avg = 0;
volatile float lm92_avg = 0;
volatile float lm92_temps[9];
volatile float ADC[9];
int lm92_packet[] = { 0x00, 0x00 };
int mode, mode_b;


int mode_count = 0;
int rtc_sec = 0;
int last_rtc_sec = 0;

unsigned int lm92_pointer_set = 0;
int lm92_temp = 0b0000000000000; // 12-bit + sign bit (13 bits)
float lm92_celcius;

unsigned int ADC_value;

volatile int sensor_ms_count = 0;
volatile int ms_count;

char led_packet[] = { 0x80 };
int send_led_packet_flag = 0;
volatile int send_lcd_packet_flag = 0;
volatile int query_rtc_flag = 0;
volatile int query_lm92_flag = 0;
char lm92_write_packet[] = { 0x00 };
char rtc_packet[] = {0x0A};         // 8 byte packet for transmission

//char packet[] = {[P1], [P2], [P3], [A1], [A2], [A3], [T], [T], [T], [M], [N]};
char packet[] = {0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A};         // 11 byte packet for transmission
char plant_op_mode = 0x00; // 0x80, 0x40, 0x20, 0x10, 0x00

volatile unsigned char col_holding, row_holding;
volatile unsigned char pressed_key;
int lock_state = 0;

void initI2C_master(){
     UCB1CTLW0 |= UCSWRST;          // SW RESET ON

     UCB1CTLW0 |= UCSSEL_3;         // SMCLK
     UCB1BRW = 10;                  // Prescalar to 10

     UCB1CTLW0 |= UCMODE_3;         // Put into I2C mode
     UCB1CTLW0 |= UCMST;            // Set as MASTER
     UCB1CTLW0 |= UCTR;             // Put into Tx mode

     UCB1CTLW1 |= 0xC0;             // Set UCCLTO = 11 (~34ms clock low timeout)

     UCB1CTLW1 |= UCASTP_2;         // Enable automatic stop bit
     UCB1TBCNT = sizeof(packet);    // Transfer byte count

     // Setup ports
     P4SEL1 &= ~BIT7;            // P4.7 SCL
     P4SEL0 |= BIT7;


     P4SEL1 &= ~BIT6;            // P4.6 SDA
     P4SEL0 |= BIT6;

     PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O
     UCB1CTLW0 &= ~UCSWRST;      // SW RESET OFF

}

void initRTC_master(){
     UCB0CTLW0 |= UCSWRST;          // SW RESET ON

     UCB0CTLW0 |= UCSSEL_3;         // SMCLK
     UCB0BRW = 10;                  // Prescalar to 10

     UCB0CTLW0 |= UCMODE_3;         // Put into I2C mode
     UCB0CTLW0 |= UCMST;            // Set as MASTER
     UCB0CTLW0 |= UCTR;             // Put into Tx mode
     UCB0I2CSA = 0x0068;            // DS3231 RTC Slave address = 0x68

     UCB0CTLW1 |= 0xC0;             // Set UCCLTO = 11 (~34ms clock low timeout)

     UCB0CTLW1 |= UCASTP_2;         // Enable automatic stop bit
     UCB0TBCNT = sizeof(packet);    // Transfer byte count

     // Setup ports
     P1SEL1 &= ~BIT3;            // P1.3 SCL
     P1SEL0 |= BIT3;

     P1SEL1 &= ~BIT2;            // P1.2 SDA
     P1SEL0 |= BIT2;

     UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF

     UCB0IE |= UCTXIE0 | UCRXIE0 | UCCLTOIE | UCBCNTIE | UCNACKIE;          // enable I2C B0 Tx, Rx, timeout IRQ


}

void initTimerB0compare(){      // Setup TB0
    TB0CTL |= TBCLR;            // Clear TB0
    TB0CTL |= TBSSEL__ACLK;     // Select SMCLK
    TB0CTL |= MC__UP;           // UP mode
}

void initTimerB1compare() {
    TB1CTL |= TBCLR;            // Clear TB1
    TB1CTL |= TBSSEL__SMCLK;    // Select SMCLK
    TB1CTL |= MC__UP;           // UP mode
    TB1CCR0 = 1049;           // Set CCR0 value (1 ms)
    TB1CCTL0 &= ~CCIFG;         // Clear TB1 flag
    TB1CCTL0 |= CCIE;           // Enable TB1 interrupt
}

void initTimerB2compare() {
    TB2CTL |= TBCLR;            // Clear TB1
    TB2CTL |= TBSSEL__SMCLK;    // Select SMCLK
    TB2CTL |= MC__UP;           // UP mode
    TB2CCR0 = 1049;           // Set CCR0 value (1 ms)
    TB2CCTL0 &= ~CCIFG;         // Clear TB1 flag
    TB2CCTL0 |= CCIE;           // Enable TB1 interrupt
}


void configureAdc() {

//    SYSCFG2 |= 0x0001 << ADCINCH_8;         // Configure ADC A8 pin (P5.0)
//    ADCCTL0 &= ~ADCENC;                     // Disable ADC
//    ADCCTL0 |= ADCSHT_2 | ADCON;            // ADC ON, 16 clocks
//    ADCCTL1 = ADCSHP;                       // Sampling timer
//    ADCCTL2 = ADCRES;                       // 10 bit resolution
//    ADCMCTL0 = ADCINCH_8 | ADCSREF_0;       // ADC A8 input select, Vref of AVCC
//    ADCIE = ADCIE0;                         // Enable ADC

    SYSCFG2 |= 0x0001 << ADCINCH_8;         // Configure ADC A8 pin (P5.0)
    ADCCTL0 &= ~ADCENC;                     // Disable ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;            // ADC ON, 16 clocks
    ADCCTL1 |= ADCSHP;                       // Sampling timer
    ADCCTL2 |= ADCRES;                       // 10 bit resolution
    ADCMCTL0 |= ADCINCH_8 | ADCSREF_0;       // ADC A8 input select, Vref of AVCC
    ADCIE |= ADCIE0;

}

void columnInput(){
    P3DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as input
    P3REN |= (BIT0 | BIT1 | BIT2 | BIT3);   // Enable pull up/down resistor
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Configure resistors as pull down

    P3DIR |= (BIT4 | BIT5 | BIT6 | BIT7);   // Init pins as outputs
    P3OUT |= (BIT4 | BIT5 | BIT6 | BIT7);   // Set as outputs

    P3IES &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // L-H edge sensitivity
    P3IE |= (BIT0 | BIT1 | BIT2 | BIT3);    // Enable IRQs

    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Clear the P3 interrupt flags
}

void rowInput(){
    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Initialize pins as input
    P3REN |= (BIT4 | BIT5 | BIT6 | BIT7);   // Enable pull up/down resistor
    P3OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Configure resistors as pull down

    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);   // Init pins as outputs
    P3OUT |= (BIT0 | BIT1 | BIT2 | BIT3);   // Set as outputs
}


void delay1000() {
    int i;
    for(i = 0; i <= 1000; i++){}
}

void ADC_start() {
    ADCCTL0 |= ADCENC | ADCSC;              // Start sampling and conversion
    while((ADCIFG & ADCIFG0) == 0);
    //__bis_SR_register(LPM3_bits | GIE);     // Enter LPM0, ADC_ISR force exits
}

void ADC_stop() {
    ADCCTL0 &= ~(ADCENC | ADCON);           // Stop sampling
}

void sampleSensor() {           // Shift stack of measured values right one, then record new temperature from sensor using ADC

    if(sampleCount >= 8) {
        ADC[8] = ADC[7];
    }
    if(sampleCount >= 7) {
        ADC[7] = ADC[6];
    }
    if(sampleCount >= 6) {
        ADC[6] = ADC[5];
    }
    if(sampleCount >= 5) {
        ADC[5] = ADC[4];
    }
    if(sampleCount >= 4) {
        ADC[4] = ADC[3];
    }
    if(sampleCount >= 3) {
        ADC[3] = ADC[2];
    }
    if(sampleCount >= 2) {
        ADC[2] = ADC[1];
    }
    if(sampleCount >= 1) {
        ADC[1] = ADC[0];
    }

    //delay1000();                    // Wait for ADC ref
    ADC_start();                    // Start sample from ADC
    //ADC_stop();                     // Stop sample from ADC

    ADC_value = ADCMEM0;

}


int getCharKey(int d) {
    switch(d) {                         // Set transmit key based on digit
        case 1:
            return 0x01;
            break;
        case 2:
            return 0x02;
            break;
        case 3:
            return 0x03;
            break;
        case 4:
            return 0x04;
            break;
        case 5:
            return 0x05;
            break;
        case 6:
            return 0x06;
            break;
        case 7:
            return 0x07;
            break;
        case 8:
            return 0x08;
            break;
        case 9:
            return 0x09;
            break;
        case 0:
            return 0x00;
            break;
        case -1:                 // Use key 0x10 for '.'
            return 0x10;
        default:
            return 0;
            break;
    }
}

void loadPacket() {

    int digit, j;

   // Place each digit of LM92 temperature into last four places of packet
   float num = lm92_avg;
   int t = ((int)num % 100)/10;
   int o = ((int)num % 10)/1;
   int th = ((int)(num * 10) % 10);

   for(i = 3; i < 6; i++) {
       if(i == 3) {
           digit = t;
       } else if(i == 4) {
           digit = o;
       } else if(i == 5) {
           digit = th;
       }
       packet[i] = getCharKey(digit);
   }

    // Round Celsius average
    avg = round(unrounded_avg*10)/10;

    t = ((int)avg % 100)/10;
    o = ((int)avg % 10)/1;
    th = ((int)(avg * 10) % 10);

    // Place each digit of Celsius temperature (including decimal) into first three places of packet
    for(i = 0; i < 3; i++) {
        if(i == 0) {
            digit = t;
        } else if(i == 1) {
            digit = o;
        } else if(i == 2) {
            digit = th;
        }
        packet[i] = getCharKey(digit);
    }

    th = ((int)mode_count % 1000)/100;
    t = ((int)mode_count % 100)/10;
    o = ((int)mode_count % 10)/1;

    // Time packet bytes
    for(i = 6; i < 9; i++) {
        if(i == 6) {
            digit = th;
        } else if(i == 7) {
            digit = t;
        } else if(i == 8) {
            digit = o;
        }
        packet[i] = getCharKey(digit);
    }

    packet[9] = plant_op_mode; // [M]
    packet[10] = n; // [N]

}

void getAverageLM92() {

    float lavg = 0;

    // Transmit temperature average if enough samples have been recorded for desired window size
    if(sampleCount > n - 1  && n > 0) {

        // Calculate moving average for window size n
        for(i = 0; i < n; i++) {
            lavg = lavg + lm92_temps[i];
        }
        lm92_avg = lavg / n;
    }
}

void getAverageLM19() {
    float shim = 4; // 5 deg celcius

    avg = 0;                             // Reset moving average

    //Convert DN (ADCMEM0) to voltage to temperature (Celsius)
    float voltage = (3.3*ADC_value)/1023;
//    float tcel = (1.8641 - voltage) / 0.01171;
//    ADC[0] = tcel;
    ADC[0] = -1481.96 + sqrt( 2.1962*pow(10, 6) + (1.8639 - (voltage)) / (3.88*pow(10, -6))  );

    // Transmit temperature average if enough samples have been recorded for desired window size
    if(sampleCount > n - 1  && n > 0) {

        // Calculate moving average for window size n
        for(i = 0; i < n; i++) {
            avg = avg + ADC[i];
        }
        unrounded_avg = (avg / n) + shim;
    }
}

void queryRTC(){

    mode_b = 1;
    UCB0CTLW0 |= UCTR;
    UCB0TBCNT = 1;
    UCB0CTLW0 |= UCTXSTT;           // Generate START condition
    while((UCB0IFG & UCSTPIFG)==0); // wait for STOP
    UCB0IFG &= ~UCSTPIFG;  // clear the stop flag

    mode_b = 2;
    UCB0CTLW0 &= ~UCTR;
    UCB0TBCNT = sizeof(rtc_packet);
    UCB0CTLW0 |= UCTXSTT;           // Generate START condition
    while((UCB0IFG & UCSTPIFG)==0); // wait for STOP
    UCB0IFG &= ~UCSTPIFG;  // clear the stop flag


}

void send_led_packet(){
    led_packet[0] = plant_op_mode;

    UCB1I2CSA = 0x0058;                 // Set slave address
    mode = 4;
    UCB1CTLW0 |= UCTR;
    UCB1TBCNT = 1;
    UCB1CTLW0 |= UCTXSTT;           // Generate START condition
    int timeout_cnt = 0;
    int tcnt = 0;
    int timeout = 0;
    while((UCB1IFG & UCSTPIFG)==0){
        // wait for STOP
        timeout_cnt++;
        if(timeout_cnt == 2500){
            tcnt++;
            if(tcnt == 20){
                UCB1IFG |= UCSTPIFG;
                return;
            }

        }
    };
    UCB1IFG &= ~UCSTPIFG;  // clear the stop flag

}

void queryLM92(){

    UCB1I2CSA = 0x0048;                 // Set slave address
    if(lm92_pointer_set == 0){
        mode = 1;
        UCB1CTLW0 |= UCTR;
        UCB1TBCNT = 1;
        UCB1CTLW0 |= UCTXSTT;           // Generate START condition
        int timeout_cnt = 0;
        int tcnt = 0;
        int timeout = 0;
        while((UCB1IFG & UCSTPIFG)==0){
            // wait for STOP
            timeout_cnt++;
            if(timeout_cnt == 2500){
                tcnt++;
                if(tcnt == 10){
                    if(UCB1IFG & UCCLTOIFG)
                    UCB1IFG |= UCSTPIFG;
                    return;
                }

            }
        };
        UCB1IFG &= ~UCSTPIFG;  // clear the stop flag

        lm92_pointer_set = 1;
    }

    mode = 2;
    UCB1CTLW0 &= ~UCTR;
    UCB1TBCNT = 2;
    UCB1CTLW0 |= UCTXSTT;           // Generate START condition
//    int timeout_cnt = 0;
//    int tcnt = 0;
//    int timeout = 0;
//    while((UCB1IFG & UCSTPIFG)==0){
//        // wait for STOP
//        timeout_cnt++;
//        if(timeout_cnt == 2500){
//            tcnt++;
//            if(tcnt == 20){
//                UCB1IFG |= UCSTPIFG;
//                return;
//            }
//
//        }
//    };
    UCB1IFG &= ~UCSTPIFG;  // clear the stop flag
}

void parseLM92Temp(){
    int i, j;
    int data;
    lm92_temp = 0;

    int n = 8;
    for(i = 3; i<=7; i++){
        if((lm92_packet[1] & n)>0){
            lm92_temp |= i-2;
        }
        n = n << 1;
    }

    j = 1;
    n = 32;
    data = lm92_packet[0];
    for(i = 0; i<=7; i++){
       if((data & j)>0){
           lm92_temp |= n;
       }
       n = n << 1;
       j = j << 1;
    }

}

void loadLM92temp(){
    int l;
    lm92_celcius = lm92_temp * 0.0625;

    // rotate lm92_temps
    for(l = 0; l < 9; l++){
        lm92_temps[l+1] = lm92_temps[l];
    }
    lm92_temps[0] = lm92_celcius;
    getAverageLM92();
}


void sendLCDPacket(){
//    TB1CCTL0 &= ~CCIFG
    while(sampling_active == 1);
    if(sampling_active != 1){
        UCB1I2CSA = 0x0068;                 // Set slave address
        mode = 3;
        j = 0;
        UCB1CTLW0 |= UCTR;
        UCB1TBCNT = sizeof(packet);
        UCB1CTLW0 |= UCTXSTT;           // Generate START condition
        int timeout_cnt = 0;
        int tcnt = 0;
        int timeout = 0;
        while((UCB1IFG & UCSTPIFG)==0){
            // wait for STOP
            timeout_cnt++;
            if(timeout_cnt == 2500){
                tcnt++;
                if(tcnt == 20){
                    UCB1IFG |= UCSTPIFG;
                    return;
                }

            }
        };
        UCB1IFG &= ~UCSTPIFG;  // clear the stop flag
    }
}

void setHot(){
    P5OUT &= ~BIT2;  // unset cool
    P5OUT |= BIT1;  // set hot
}
void setCool(){
    P5OUT |= BIT2;  // set cool
    P5OUT &= ~BIT1;  // unset hot
}
void setOff(){
    P5OUT &= ~BIT2;  // unset cool
    P5OUT &= ~BIT1;  // unset hot
}
void updateTempControls(){
    if(plant_op_mode == 0x80){
        setHot();
    } else if(plant_op_mode == 0x40){
        setCool();
    } else if(plant_op_mode == 0x20){
        // match ambient
    } else if(plant_op_mode == 0x10){
        setOff();
    } else {
        setOff();
    }
}

int main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    P6DIR |= (BIT2 | BIT3 | BIT4 | BIT5 | BIT6);     // Set P6.6 as OUTPUT
    P6OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6);    // Clear P6.6

    P5DIR |= (BIT2 | BIT1); // hot/cold outputs
    P5OUT |= ~(BIT2 | BIT1);

    initI2C_master();                   // Intialize master for I2C transmission
    initRTC_master();
    initTimerB0compare();               // Initialize Timer B0 for getting keypad input
    initTimerB1compare();               // Initialize Timer B1 for sampling temperature sensor  (1ms * 333);
    initTimerB2compare();
    configureAdc();                     // Initialize ADC for receiving input signal from temperature sensor

    columnInput();                      // Configure keypad columns to start as inputs

    PM5CTL0 &= ~LOCKLPM5;               // Turn on Digital I/O


    UCB1IE |= UCTXIE0 | UCRXIE0 | UCCLTOIE | UCBCNTIE | UCNACKIE;                  // Enable I2C TX interrupt

    __enable_interrupt();

    int i,k,l;
    while(1){

        // LED Bar
        if(send_led_packet_flag == 1){
            send_led_packet();
            send_led_packet_flag = 0;
        }

        // RTC
        if(query_rtc_flag == 1){
            P6OUT ^= BIT3;
            queryRTC(); // query RTC
            query_rtc_flag = 0;
        }

        // LM92
        if(query_lm92_flag == 1){
            P6OUT ^= BIT2;
            queryLM92(); // query LM92 (plant temp)
            parseLM92Temp();

            loadLM92temp();

            query_lm92_flag = 0;
        }

        // Output control
        updateTempControls();

        // 16x2 LCD output
        if(send_lcd_packet_flag == 1){
            loadPacket();
            sendLCDPacket();
            send_lcd_packet_flag = 0;
        }


        for(i=0;i<100;i++){
            for(k=0;k<50;k++){
                delay1000();
            }
        }
    }

    return 0;
}



int is_unlocked(char key){
    if(lock_state == 0){
        return 1;
    } else {
        if(lock_state == 1){
            if(key == 0x80){
                P6OUT |= BIT6;
                lock_state = 2;
            }
        } else if(lock_state == 2){
            if(key == 0x40){
                P6OUT |= BIT5;
                lock_state = 3;
            }
        } else if(lock_state == 3){
            if(key == 0x20){
                P6OUT |= BIT4;
                //displayUnlockPattern();
                lock_state = 0;
            }
        }
        return 0;
    }
}


void keyPressedAction(char pressed_key) {
    if(is_unlocked(pressed_key) == 1){
        if(pressed_key == 0x11 || pressed_key == 0x17) {        // If #/*, transmit to LCD slave
            packet[0] = pressed_key;                            // Place #/* first in packet
            for(i = 1; i < sizeof(packet); i++) {
                packet[i] = 0x0A;                               // Fill the rest of packet with dummy values
            }
        } else {
            switch(pressed_key) {           // If not */#, set n value to key pressed (0-9)
                case 0x87:
                    n = 1;
                    break;
                case 0x83:
                    n = 2;
                    break;
                case 0x81:
                    n = 3;
                    break;
                case 0x80:
                    plant_op_mode = pressed_key;
                    break;
                case 0x47:
                    n = 4;
                    break;
                case 0x43:
                    n = 5;
                    break;
                case 0x41:
                    n = 6;
                    break;
                case 0x40:
                    plant_op_mode = pressed_key;
                    break;
                case 0x27:
                    n = 7;
                    break;
                case 0x23:
                    n = 8;
                    break;
                case 0x21:
                    n = 9;
                    break;
                case 0x20:
                    plant_op_mode = pressed_key;
                    break;
                case 0x10:
                    plant_op_mode = pressed_key;
                    break;
                case 0x13:
                    n = 0;
                    for(i = 0; i < 7; i++) {
                        packet[i] = 0x0A;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    columnInput();                              // Reset keypad columns to be inputs
    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);      // Clear the P3 interrupt flags

}

#pragma vector=PORT3_VECTOR
__interrupt void ISR_PORT3(void){           // Enable timer for debouncing
    TB0CCTL0 |= CCIE;                       // Local IRQ enable for CCR0
    TB0CCR0 = 400;                          // Set CCR0 value (period) // old value = 2384
    TB0CCTL0 &= ~CCIFG;                     // Clear CCR0 flag to begin count
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){

    TB0CCTL0 &= ~CCIE;      // Disable TimerB0
    UCB1IE &= ~UCTXIE0;     // Disable I2C B0 TX interrupt

    col_holding = P3IN;

    rowInput();

    row_holding = P3IN;
    pressed_key = col_holding + row_holding;

    P6OUT |= BIT4; // green led on

    if(pressed_key == 0x80 || pressed_key == 0x40 || pressed_key == 0x20 || pressed_key == 0x10){
         send_led_packet_flag = 1;
         mode_count = 0;
     }

    keyPressedAction(pressed_key);

    UCB1IE |= UCTXIE0;      // Enable I2C B0 TX interrupt
    UCB1CTLW0 |= UCTXSTT;           // Generate START condition
}


#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void) {
    sensor_ms_count++;
    if((sensor_ms_count == 250)||(sensor_ms_count == 750)){
        if(sensor_ms_count == 250){
            query_rtc_flag = 1;
        }
        query_lm92_flag = 1;

    }
    if((sensor_ms_count == 500)||(sensor_ms_count == 1000)){

        P6OUT ^= BIT6;    // toggle P6.6
        sampleCount++;
        sampling_active = 1;
        sampleSensor();                                     // Sample temperature sensor
        getAverageLM19();
        P6OUT ^= BIT6;    // toggle P6.6

        sampling_active = 0;
        if(sensor_ms_count == 1000){
            sensor_ms_count = 0;
        }
    }
    if(sensor_ms_count == 900){
        if(plant_op_mode != 0){
            send_lcd_packet_flag = 1;
        }
    }

    TB1CCTL0 &= ~CCIFG;                                     // Clear flag
}

#pragma vector = TIMER2_B0_VECTOR
__interrupt void ISR_TB2_CCR0(void){
    ms_count++;

    if(ms_count == 1090){
        P6OUT ^= BIT5;
        UCB1CTLW0 |= UCTXSTT;           // Generate START condition
        ms_count = 0;
    }

    TB0CCTL0 &= ~CCIFG;
}

#pragma vector = ADC_VECTOR
__interrupt void ISR_ADC(void) {
    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)) {
        case ADCIV_ADCIFG:
            ADCIFG &= ~ADCIFG0;                             // Clear ADC flag
            //__bic_SR_register_on_exit(LPM3_bits + GIE);     // Exit low power mode
            break;
        default:
            break;
    }
}

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_TX_ISR(void){                     // Fill TX buffer with packet values

    switch(UCB1IV){
        case 0x16:      // ID 16: RXIFG0
            if(mode == 2){
                if (j == (sizeof(lm92_packet)-1)){
                    lm92_packet[j] = UCB1RXBUF;
                    j = 0;
                } else {
                    lm92_packet[j] = UCB1RXBUF;
                    j++;
                }
            }
            break;
        case 0x18:      // ID 18: TXIFG0
            if(mode == 1){
                UCB1TXBUF = lm92_write_packet[0];
            } else if(mode == 4){
                UCB1TXBUF = led_packet[0];
            } else if(mode == 3){
                if (j == (sizeof(packet)-1)){
                    UCB1TXBUF = packet[j];
                    P6OUT &= ~BIT4; // green led on
                    j = 0;
                } else {
                    UCB1TXBUF = packet[j];
                    j++;
                }
            }
            break;
        case 0x04: // NACK
            break;
        case 0x1A: // byte count zero interrupt
            //j = 0;
            break;
        case 0x1C: // clock low timeout
            UCB0CTLW0 |= UCTXNACK;
//            UCB0CTLW0 = UCSWRST;                        // Put I2C into reset
//                UCB0IE &= ~(UCTXIE);                        // Disable TX interrupt
//                UCB0IE |= UCRXIE;                           // Enable RX interrupt
//            UCB0CTLW0 &= ~UCSWRST;                      // Take I2C out of reset
            break;
        default:
            break;
    }
}

/*
 * RTC I2C Interrupt
 */
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    switch(UCB0IV){
        case 0x16:      // ID 16: RXIFG0
            if(mode_b == 2){
                if (r == (sizeof(rtc_packet)-1)){
                    rtc_packet[r] = UCB0RXBUF;
                    r = 0;

                    rtc_sec = rtc_packet[0];
                    if(rtc_sec != last_rtc_sec){
                        mode_count++;
                        last_rtc_sec = rtc_sec;
                        if(mode_count == 300){
                            mode_count = 0;
                            plant_op_mode = 0x10;
                        }
                    }
                } else {
                    rtc_packet[r] = UCB0RXBUF;
                    r++;
                }
            }
            break;
        case 0x18:      // ID 18: TXIFG0
            if(mode_b == 1){
                UCB0TXBUF = 0x00;
            }
            break;
        default:
            break;
    }


}
