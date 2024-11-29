//-------------------------------------------------------------------------------
// Zachary Elmer and Leah Baker, EELE 371, 11/21/2024
//  Reverse_Cycle_Number = 128 because 513 is the steps per cycle and 513/4 = 128
//  Forward_Cycle_Number = 13 because it is roughly 1/10th of 128
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
#define Forward_Cycle_Number 13 //Clockwise
#define Reverse_Cycle_Number 128 //AntiClockwise


char RTC_Packet[] = {0x03, 0x00, 0x00, 0x12, 0x08, 0x03, 0x11, 0x24}; //Send Current time to the RTC as configuration

_Bool RTC_config = 0; //If true the I2C sends the RTC config packet over the I2C bus.
_Bool port_expander_config = 0; //If true the I2C sends the port expander config over the bus

int Data_Cnt = 0;

// UART and message variables
char Forward[] = "\n\r Motor reversed 1 rotation. \r\n\0";
char Reverse[] = "\n\r Motor advanced 1 step \r\n\0";
int Position = 1;
char *Message; //Memory start of message to be sent

char Data_In;
char Seconds_Recived;
char Minutes_Recived;

int ADC_Value;
_Bool RTC_Recive_Flag = 0;


// Motor control variables
_Bool Move_Forward = 0;
_Bool Move_Reverse = 0;
int State = 0;
int Cycle = 0;

void I_O_Init(void){
    /*
     * Initialize inputs and outputs
     */

    LED1OUT
    LED2OUT

    P3SEL0 &= ~(BIT0 | BIT1| BIT2 | BIT3);
    P3SEL1 &= ~(BIT0 | BIT1| BIT2 | BIT3); //P3.0 - P3.3 set as I/O
    P3DIR |= (BIT0 | BIT1| BIT2 | BIT3) ;
    P3OUT &= ~(BIT0 | BIT1| BIT2 | BIT3);SW1wINT

    P3OUT |= BIT0;

    SW1wINT
    SW2wINT

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

void UART_Init(void){
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

void Timer_Init(void)
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

void Init(void){
    /* Init function
     *
     */

    I_O_Init();
    ADC_Init();
    I2C_Init();
    UART_Init();
    Timer_Init();

    //Interrupt Enables
    ADCIE |= ADCIE0; //Enable ADC Conv Compleate IRQ

    UCB0IE |= UCTXIE0;
    UCB0IE |= UCRXIE0;

    SW1IntEn
    SW2IntEn

    PM5CTL0 &= ~LOCKLPM5; // Turn on GPIO
    __enable_interrupt();
        return;
}

void RTC_Config(void){
    //Transmit configuration to RTC

    UCB0CTLW0 |= UCTR; //Transmit mode
    UCB0I2CSA = 0x0068; //Slave address =0x68 (RTC address)
    UCB0TBCNT = sizeof(RTC_Packet); //number of bytes in packet
    RTC_config = 1;
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
    UCB0I2CSA = 0x0068; //Slave address =0x68 (RTC address)
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
    /*
     * Gather ADC value and compare to voltage macros
     */
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
//--End ADC ISR

#pragma vector = EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void){
    /*
     * Loads from memory defined by message pointer into buf until null char is hit.
     */
    if(Message[Position] == '\0'){ // '\0' is the ending char so exit and stop transmitting
        Position = 1;
        UCA1IE &= ~UCTXCPTIE; //Turn off TX interrupt because we are done
    }
    else{
        UCA1TXBUF = Message[Position];
        Position++;
    }
    UCA1IFG &= ~UCTXCPTIFG;
}
//--End UART ISR

//ISR
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    /*
     * If RTC_Config is true, transmits the RTC config array to the RTC
     * Else if the interrupt is 0x16 then receive data from the RTC
     * If the interrupt is 0x18 then transmit the register address to the RTC
     * Then load each response into individual variables
     */
    if(RTC_config){
        //Transmit Config Data to RTC
        if(Data_Cnt == (sizeof(RTC_Packet)-1)){
            UCB0TXBUF = RTC_Packet[Data_Cnt];
            Data_Cnt = 0;
            RTC_config = 0;
        }
        else{
            UCB0TXBUF = RTC_Packet[Data_Cnt];
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
//--End I2C ISR

#pragma vector = PORT4_VECTOR
__interrupt void ISR_Port4_S1(void){
    /*
     * On SW1 pressed set system state to Move_Forward, active motion
     * Clear the timer count to align timer with switch press
     * Then start transmit of forward message over UART
     */
    //SW1 ISR
    // TODO: prevent both interrupts from being active at the same time
    TB0EX0 = TBIDEX__5;         // make divider 5 so that 1/10th of the movement takes 1/2 the time of a full rotation
    Move_Forward = 1;
    State = 0;
    TB0R = 0; //Clear timer count


    //Sends Forward on press
    UCA1IE |= UCTXCPTIE;
    Message = Forward; //Set the message to be sent
    UCA1TXBUF = Message[0]; //Transmit the start of the message

    P4IFG &= ~BIT1;
}
//--End SW1 ISR

#pragma vector = PORT2_VECTOR
__interrupt void ISR_Port2_S2(void){
    /*
     * On SW2 pressed set system state to Move_Reverse, active motion
     * Clear the timer count to align timer with switch press
     * Then start transmit of reverse message
     */
    //SW2 ISR
    // TODO: prevent both interrupts from being active at the same time
    TB0EX0 = TBIDEX__1;         // make sure divider is 1
    Move_Reverse = 1;
    State = 0;
    TB0R = 0;

    //Sends reverse on press
    UCA1IE |= UCTXCPTIE;
    Message = Reverse; //Sets the message to be sent
    UCA1TXBUF = Message[0]; //Transmit the start of the message

    P2IFG &= ~BIT3;
}
//--End SW2 ISR

#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    /*
     * Timer with state machine controlling motion of the motor
     * Each state machine is enabled by Move_Forward or Move_Reverse
     */

    //FSM controlling the order of led lighting
    //Todo add default cases
    if(Move_Forward){
        P3OUT &= 0;
        State++;
        switch (State) {
        case 1:
            P3OUT |= BIT1;
            break;

        case 2:
            P3OUT |= BIT2;
            break;

        case 3:
            P3OUT |= BIT3;
            break;
        case 4:
            P3OUT |= BIT0;
            State = 0;
            if(Cycle == Forward_Cycle_Number -1){ //We have reached the maximum number of cycles, time to end
                Move_Forward = 0;
                Cycle = 0;
            }
            else{
                Cycle++;
            }
            break;
        }
    }

    if(Move_Reverse){
        P3OUT &= 0;
        State++;
        switch (State) {
        case 1:
            P3OUT |= BIT3;
            break;

        case 2:
            P3OUT |= BIT2;
            break;

        case 3:
            P3OUT |= BIT1;
            break;
        case 4:
            P3OUT |= BIT0;
            State = 0;
            if(Cycle == Reverse_Cycle_Number -1){ //We have reached the maxium number of cylces, time to end
                Move_Reverse = 0;
                Cycle = 0;
            }
            else{
                Cycle++;
            }
            break;
        }
    }
    TB0CCTL0 &= ~CCIFG;
}
//End Timer ISR
