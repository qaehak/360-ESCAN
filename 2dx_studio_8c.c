/*  2DX3 Project 2 Deliverable Code

		Written by Mehak Shah, 400478491
		Last Modified: April 02, 2024

		Assigned LED: D1 (PN1)
		Optional Assigned LED: D2 (PN0)
	
		Assigned Bus Speed: 40 MHz
*/ 

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

uint16_t dev = 0x29;			// Address of the ToF sensor as an I2C follower peripheral
int status = 0;

volatile unsigned long FallingEdges = 0;    // Global variable visible in Watch window of debugger
																						// Increments at least once per button press
int home = 0;																// Tracks the number of motor steps		

//  >>>>>>>>>>>>>>>>> Port Initilization >>>>>>>>>>>>>>>>>>

//  Port J handles switch input
void PortJ_Init(void){
	//  Use PJ1 as switch
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					//  Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	//  Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										//  Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x02;     										//  Enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	Enable weak pull up resistor on PJ1
}

//  Port H handles steppar motor rotation
void PortH_Init(void){
	//  Port H pins (PH0-PH3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;						//  Activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};		//  Allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        										//  Configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     										//  Disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        										//  Enable digital I/O on Port H pins (PH0-PH3)
  GPIO_PORTH_AMSEL_R &= ~0x0F;     										//  Disable analog functionality on Port H	pins (PH0-PHS3)	
	return;
}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;         	// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;        // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};	  			// ready?
  GPIO_PORTB_AFSEL_R |= 0x0C;           					// 3) enable alt funct on PB2,3       0b00001100
  GPIO_PORTB_ODR_R |= 0x08;             					// 4) enable open drain on PB3 only
  GPIO_PORTB_DEN_R |= 0x0C;             			    // 5) enable digital I/O on PB2,3
                                                  // 6) configure PB2,3 as I2C
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
  I2C0_MCR_R = I2C_MCR_MFE;                      			// 9) master function enable
  I2C0_MTPR_R = 0b0000000000000101000000000111011;    // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;           // activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};  // allow time for clock to stabilize
  GPIO_PORTG_DIR_R &= 0x00;                          // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                       // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                          // enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                       // disable analog functionality on PN0
  return;
}

//  XSHUT     
//  This pin is an active-low shutdown input; 
//	the board pulls it up to VDD to enable the sensor by default. 
//	Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                   // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;            // PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                  // make PG0 input (HiZ)    
}

//  >>>>>>>>>>>>>>>>> Motor Function >>>>>>>>>>>>>>>>>>

//  Rotates motor clockwise
void spinCW(){													
	GPIO_PORTH_DATA_R = 0b0011;
	SysTick_Wait10ms(1);
	GPIO_PORTH_DATA_R = 0b0110;
	SysTick_Wait10ms(1);	
	GPIO_PORTH_DATA_R = 0b1100;
	SysTick_Wait10ms(1);			
	GPIO_PORTH_DATA_R = 0b1001;
	SysTick_Wait10ms(1);
}

//  Rotates motor counter-clockwise
void spinCCW(){																
	GPIO_PORTH_DATA_R = 0b1001;
	SysTick_Wait10ms(1);
	GPIO_PORTH_DATA_R = 0b1100;
	SysTick_Wait10ms(1);
	GPIO_PORTH_DATA_R = 0b0110;
	SysTick_Wait10ms(1);
	GPIO_PORTH_DATA_R = 0b0011;
	SysTick_Wait10ms(1);
}

//  >>>>>>>>>>>>>>>>> Interrupt Functions >>>>>>>>>>>>>>>>>>

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;             		// Initialize counter

		GPIO_PORTJ_IS_R = 0;   						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;  						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;  						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x02; 					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;  					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000; 					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000;				// (Step 4) Set interrupt priority to 5

		EnableInt();											// (Step 3) Enable Global Interrupt. lets go!
}

//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOJ_IRQHandler(void){
  FallingEdges = FallingEdges + 1;	  //  Increase the global counter variable; Observe in Debug Watch Window
	
	GPIO_PORTJ_ICR_R = 0x02; 					  //  Acknowledge flag by setting proper bit in ICR register
}

//  >>>>>>>>>>>>>>>>> Main Function >>>>>>>>>>>>>>>>>> 

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;

	// Initialize
	PortH_Init();						 //  Initialize Port H for motor control
	PortJ_Init();						 //  Initialize the onboard push button on PJ1
	PortJ_Interrupt_Init();	 //  Initalize and configure the Interrupt on Port J
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	int state = 0;					 //  Button state: Currently OFF
	int numLayers = 16;			 //  Number of scans. Default = 3 for small objects
	
	// Wait for device ToF booted
	while(sensorState == 0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
		
  // Initialize the sensor with the default setting 
  status = VL53L1X_SensorInit(dev);

	// Function has to be called to enable the ranging 
  status = VL53L1X_StartRanging(dev);
	
	while(1){								 //  Inside an infinite while loop,
		WaitForInt();					 //  Call WaitForInt() function to wait for interrupts
		state = state ^ 1;	 	 //  Toggle button state
		
		// Outer loop to repeat the section of code 3 times
		for (int repeat = 0; repeat < numLayers && state == 1; repeat++) {
			for(int i = 0; i < 512 && state == 1; i++){												
				
				//  Exit condition after scans are complete
				if (repeat == (numLayers) && i == 511){
					FallingEdges = FallingEdges + 1;
					state = state ^ 1;
					home = i;
					break;
				}	
				
				spinCW();
					
				//  Toggles LED D1 (PN1) every 11.25 degrees
				if (i % 16 == 0){
					FlashLED1(1);
						
					//  Wait until the ToF sensor's data is ready
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED2(1);
						VL53L1_WaitMs(dev, 5);
					}
					
				dataReady = 0;
				status = VL53L1X_ClearInterrupt(dev); //  Clear interrupt has to be called to enable next interrupt
					
				//  Read the distance value from the ToF sensor
				status = VL53L1X_GetDistance(dev, &Distance);
				sprintf(printf_buffer,"%d\r\n",Distance);
				UART_printf(printf_buffer);
				}
					
				//  Exit condition (when button is pressed midway)
				if (FallingEdges % 2 == 0){
					FlashLED2(1);
					home = i;                  // Resets variable
					state = state ^ 1;
				}
			}
			SysTick_Wait10ms(10);
			for (int i = 0; i < 512; i++){spinCCW();}
		}
		//SysTick_Wait10ms(10);
				
		//  Resets motor to home position (i = 0)
		for (int i = 0; i < home; i++){spinCCW();}
		home = 0;
	} 
}