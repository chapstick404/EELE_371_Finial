//-------------------------------------------------------------------------------
// Zachary Elmer and Leah Baker, EELE 371, 11/21/2024
//  reverseCycleNumber = 128 because 513 is the steps per cycle and 513/4 = 128
//  forwardCycleNumber = 13 because it is roughly 1/10th of 128
//  TB0CCR0 = 4678 because the period is supposed to be minimized, the maximum RPM (with good torque) is 25,
//       and ((25 RPM) * (513 steps/revolution) / (60 s/m))^-1 is 0.004678 seconds per step or 4678 uS per step
//-------------------------------------------------------------------------------
#include <msp430.h>

/**
 * Leah Baker, 371, 11/18/2024 main.c
 * A4 set as P1.4
 * Precision is 0.00083V
 * Accuracy is 0.000415V
 */

#define SW1wINT {P4SEL0 &= ~BIT1; \
                P4SEL1 &= ~BIT1; \
                P4DIR &= ~BIT1; \
                P4REN |= BIT1; \
                P4OUT |= BIT1; \
                P4IES |= BIT1; } //Setting SW1 as an input with H-L sensitivity (P4.1)
                                 //Interrupts will still need to be enabled
#define SW1IntEn {P4IFG &= ~BIT1; \
                  P4IE |= BIT1;}

#define SW2wINT {P2SEL0 &= ~BIT3; \
                P2SEL1 &= ~BIT3; \
                P2DIR &= ~BIT3; \
                P2REN |= BIT3; \
                P2OUT |= BIT3; \
                P2IES |= BIT3; } //Setting SW2 as an input with H-L sensitivity (P4.1)
                                 //Interrupts will still need to be enabled
#define SW2IntEn {P2IFG &= ~BIT3; \
                  P2IE |= BIT3;}


#define LED1ON P1OUT |= BIT0;
#define LED1OFF P1OUT &= ~BIT0;
#define LED1Tog P1OUT ^= BIT0;

#define LED2ON P6OUT |= BIT6;
#define LED2OFF P6OUT &= ~BIT6;
#define LED2Tog P6OUT ^= BIT6;

#define LED2OUT {P6DIR |= BIT6; \
                LED2OFF} //Setting LED2 as an output (P6.6)


#define LED1OUT {P1DIR |= BIT0; \
                LED1OFF} //Setting LED1 as an output (P1.0)

#define VOLTAGECONVERSION 1230 //Conversion factor for voltage to measured ADC value


//Controls for the status logic
//GreenLED (LED2) below LOWVOLTAGE
//RedLED (LED1) above HIGHVOLTAGE
//No LED between LOWVOLTAGE and HIGHVOLTAGEs
#define LOWVOLTAGE 1
#define HIGHVOLTAGE 2

#define OFFSET 16 //Measured offset at 0v


//Knobs to control motor behavior
#define ForwardCycleNumber 13 //Clockwise
#define ReverseCycleNumber 128 //AntiClockwise


char Packet[] = {0x03, 0x00, 0x00, 0x12, 0x08, 0x03, 0x11, 0x24}; //Send Current time to the RTC as configuration
_Bool firstpacket = 1;
int Data_Cnt = 0;

// UART and message variables
char forward[] = "\n\r Motor reversed 1 rotation. \r\n\0";
char reverse[] = "\n\r Motor advanced 1 step \r\n\0";
int position = 1;
char *message; //Memory start of message to be sent

char Data_In;
char Seconds_Recived;
char Minutes_Recived;

int ADC_Value;
_Bool RTC_Recive_Flag = 0;


// Motor control variables
_Bool moveForward = 0;
_Bool moveReverse = 0;
int state = 0;
int cycle = 0;

void I_O_Init(void){
    /*
     * Initialize inputs and outputs
     */

    LED1OUT
    LED2OUT

    P1SEL1 |= BIT4;
    P1SEL0 |= BIT4; //P1.4 for A4 (Device specific Datasheet)

    //RTC Pin Setup
    P1SEL1 &= ~BIT3;
    P1SEL0 |= BIT3; //P1.3 = SCL

    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2; //P1.2 = SDA

    //UART Pin Setup
    P4SEL1 &= ~BIT3;
    P4SEL0 |= BIT3;
    P4SEL1 &= ~BIT2;
    P4SEL0 |= BIT2;
}

void ADC_Init(void){
    /*
     * Initialize the ADC
     */

    ADCCTL0 &= ~ADCSHT; //Clear ADCSHT
    ADCCTL0 |= ADCSHT_2; //Conversion cycles - 16
    ADCCTL0 |= ADCON;

    ADCCTL1 |= ADCSSEL_2; //ADC Clock source as SMCLK
    ADCCTL1 |= ADCSHP; //Sample signal source as sampling timer

    ADCCTL2 &= ~ADCRES; //Clear ADCRES
    ADCCTL2 |= ADCRES_2; //12-bit Resolution

    ADCMCTL0 |= ADCINCH_4; //ADC Input channel as A4
}

void I2C_Init(void){
    /*
     * Initialize the I2C controller
     * P1.3 as SCL
     * P1.2 as SDA
     */

    //I2C controller setup
    UCB0CTLW0 |= UCSWRST; //USCI_B0 in SW reset

    UCB0CTLW0 |= UCSSEL__SMCLK; //BRCLK=SMCLK=1Mhz
    UCB0BRW = 10; //Divide BRCLK by 10 SCL=100kHz

    UCB0CTLW0 |= UCMODE_3; //I2C mode
    UCB0CTLW0 |= UCMST; //Master mode
    UCB0I2CSA = 0x0068; //Slave address =0x68

    UCB0CTLW1 |= UCASTP_2;//Number of bytes to receive

    UCB0CTLW0 &= ~UCSWRST; //Out of SW reset
}

void UART_init(void){
    /*
     * Initialize the eUSCI_A1 as UART
     */
    //-- Put eUSCI_A1 into reset
    UCA1CTLW0 |= UCSWRST;

    //-- Configure eUSCI_A1
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 17;
    UCA1MCTLW |= 0x4A00;

    //-- Take eUSCI_A1 out of SW reset
    UCA1CTLW0 &= ~UCSWRST;
}

void Timer_init(void)
{
    //--Setup Timer
    TB0CTL |= TBCLR; //Clear timer and dividers
    TB0CTL |= TBSSEL__SMCLK;
    TB0CTL |= MC__UP; //Mode Up
    TB0CTL |= ID__1;
    TB0EX0 = TBIDEX__1;
    TB0CCR0 = 4678;

    //Timer ISQ
    TB0CCTL0 &= ~CCIFG; //Clear interrupt flag
    TB0CCTL0 |= CCIE;
}

void init(void){
    /* Init function
     *
     */

    I_O_Init();
    ADC_Init();
    I2C_Init();
    Timer_Init();

    //Interrupt Enables
    ADCIE |= ADCIE0; //Enable ADC Conv Compleate IRQ

    UCB0IE |= UCTXIE0;
    UCB0IE |= UCRXIE0;

    PM5CTL0 &= ~LOCKLPM5; // Turn on GPIO
    __enable_interrupt();
        return;
}

void RTC_Config(void){
    //Transmit configuration to RTC

    UCB0CTLW0 |= UCTR; //Transmit mode
    UCB0TBCNT = sizeof(Packet); //number of bytes in packet
    UCB0CTLW0 |= UCTXSTT; // Generate START condition

    while ((UCB0IFG & UCSTPIFG) == 0){}
    UCB0IFG &= ~UCSTPIFG;
}

void RTC_Recive(void){
    //Receive Data from RTC

    RTC_Recive_Flag = 0;

    //Transmit Register Address to RTC
    UCB0TBCNT = 0x01; //Limit to 1 byte transmit
    UCB0CTLW0 |= UCTR; //Tx mode
    UCB0CTLW0 |= UCTXSTT; //Start condition

    while ((UCB0IFG & UCSTPIFG) == 0){} //Waiting until the I2C controller completes
    UCB0IFG &= ~UCSTPIFG;

    UCB0CTLW0 &= ~UCTR; //Rx Mode
    UCB0TBCNT = 0x02; //Want only 2 Registers from RTC
    Data_Cnt = 0;
    UCB0CTLW0 |= UCTXSTT; //Start condition

    while ((UCB0IFG & UCSTPIFG) == 0){}  //Wait until I2C completes
    UCB0IFG &= ~UCSTPIFG;
}

void ADC_Measure(void){
    //Start the ADC measurement
    ADCCTL0 |= ADCENC | ADCSC; //Enable and start conversion
    while((ADCIFG & ADCIFG0) == 0){} //Wait for conversion to finish (??? TI says it works)

}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    init();

    RTC_Config();

    while(1){
        ADC_Measure();
        if(RTC_Recive_Flag){
            RTC_Recive();
        }

    }

    return 0;
}

//---ISR--
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void){
    //Gather ADC value and compare to voltage macros
    ADC_Value = ADCMEM0;
    if(ADC_Value - OFFSET < LOWVOLTAGE * VOLTAGECONVERSION){
        LED1OFF
        LED2ON
    }
    else if(ADC_Value - OFFSET > HIGHVOLTAGE * VOLTAGECONVERSION){
        RTC_Recive_Flag = 1;
        LED1ON
        LED2OFF
    }
    else{
        LED1OFF
        LED2OFF
    }
}

#pragma vector = EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void){
    if(message[position] == '\0'){ // '\0' is the ending char so exit and stop transmitting
        position = 1;
        UCA1IE &= ~UCTXCPTIE; //Turn off TX interrupt because we are done
    }
    else{
        UCA1TXBUF = message[position];
        position++;
    }
    UCA1IFG &= ~UCTXCPTIFG;
}
//--End UART ISR

//ISR
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    if(firstpacket){
        //Transmit Config Data to RTC
        if(Data_Cnt == (sizeof(Packet)-1)){
            UCB0TXBUF = Packet[Data_Cnt];
            Data_Cnt = 0;
            firstpacket = 0;
        }
        else{
            UCB0TXBUF = Packet[Data_Cnt];
            Data_Cnt++;
        }
        }

    else{
        switch(UCB0IV){
            case 0x16:
                Data_In = UCB0RXBUF;
                Data_Cnt++;
                break;
            case 0x18:
                UCB0TXBUF = 0x03;
                break;
            default:
                break;
        }
        if(Data_Cnt == 1){
            Seconds_Recived = Data_In;
        }
        if(Data_Cnt == 2){
            Minutes_Recived = Data_In;
        }
        }
}


