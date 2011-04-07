
#ifndef _CONFIG_H
#define _CONFIG_H

//*** <<< Use Configuration Wizard in Context Menu >>> *** 


// Configure the interrupt priority  at a level from 0 to 15 (0 is highest priority)
// for each interrupot vector (VIC0 and VIC1)


//<h>Vector 0	(higher priority than vector 1)
//<i> IRQ with the same priority are NOT allowed

//<o> VIC0.5: system time <0-15>
#define PRIORITY_TIMER1 0   	// VIC0.5: system time

//<o> VIC0.6: servo pwm	<0-15>
#define PRIORITY_TIMER2 7   	// VIC0.6: servo pwm, must be disabled on UART redirection

//<o> VIC0.8 VIC 0.9: usb port <0-15>
#define PRIORITY_USB 	9		// VIC0.8 VIC 0.9: usb port

//<o> VIC0.15: adc conversion <0-15>
#define PRIORITY_ADC 	14     	// VIC0.15: adc conversion
//=============================================================================
//</h>
//<h>Vector 1	(lower priority than vector 0)

//<o> VIC1.0: uart0 to GPS/MK3MAG <0-15>
#define PRIORITY_UART0 	2    	// VIC1.0: uart to GPS/MK3MAG

//<o> VIC1.1: uart2 to FC <0-15>
#define PRIORITY_UART2 	1		// VIC1.1: uart to FC

//<o> VIC1.2: debug uart1 <0-15>
#define PRIORITY_UART1 	0		// VIC1.2: debug uart

//<o> VIC1.4: i2c to MK3MAG <0-15>
#define PRIORITY_I2C1 	4     	// VIC1.4: i2c to MK3MAG

//<o> VIC1.5: SPI0 <0-15>
#define PRIORITY_SPI0 	3		// VIC1.5: timing forced by FC, must be lower than UARTS for flashing FC thrue the NC

//<o> VIC0.5: VIC1.10 switch SD card <0-15>
#define PRIORITY_SDSWITCH 14	// VIC1.10 switch at sd port

#define PRIORITY_SW	15		//VIC1.11: Software Interrupt at absolute lowest priority

//</h>
//=============================================================================
//<h>Baudrates
//<o> UART0 	<9600=> 9600 Baud <19200=> 19,2 kBaud <38400=> 38,4 kBaud <57600=> 57,6 kBaud <115200=> 115,2 kBaud	
#define UART0_BAUD_RATE 57600		//Baud Rate for the serial interfaces

//<o> UART1		<9600=> 9600 Baud <19200=> 19,2 kBaud <38400=> 38,4 kBaud <57600=> 57,6 kBaud <115200=> 115,2 kBaud	
#define UART1_BAUD_RATE 57600		//Baud Rate for the serial interfaces

//<o> UART2 	<9600=> 9600 Baud <19200=> 19,2 kBaud <38400=> 38,4 kBaud <57600=> 57,6 kBaud <115200=> 115,2 kBaud	
#define UART2_BAUD_RATE 57600		//Baud Rate for the serial interfaces

//<o> I2C-Bus1 	<22000=> 20 kBit <50000=> 50 kBit <100000=> 100 kBit <200000=> 200 kBit <400000=> 400 kBit	
#define I2C1_CLOCK 50000			// Bit Rate for I2C

//</h>
//<<< end of configuration section >>> 

#endif // _CONFIG_H
