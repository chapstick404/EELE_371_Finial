#include <msp430.h> 
//cats for da cat throne!
//merp
//nya

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	return 0;
}
