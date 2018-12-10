/*
 * File:        TFT, keypad, DAC, LED test
 * Author:      Bruce Land
 * For use with Sean Carroll's Big Board
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

#include <xc.h>
#include <plib.h>
////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_3.h"
#include "SparkFun_LSM9DS1.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
////////////////////////////////////

////////////////////////////////////
// pullup/down macros for keypad
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
////////////////////////////////////

////////////////////////////////////
// some precise, fixed, short delays
// to use for extending pulse durations on the keypad
// if behavior is erratic
#define NOP asm("nop");
// 1/2 microsec
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// one microsec
#define wait40 wait20;wait20;
////////////////////////////////////

//find clock period 
#define SYS_FREQ 40000000
#define PERIOD 45
#define generate_period SYS_FREQ/PERIOD/64
volatile int pwm_on_time = (int)(generate_period/4);

//flag to initiate boot sequence
// volatile int boot = 1;

/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/



///////Gyro+Accel variables//////////
//gyroAxis
volatile float gx = 0;
volatile float gy = 0;
volatile float gz = 0;

//accelAxis
volatile float ax = 0;
volatile float ay = 0;
volatile float az = 0;
//Gyro weigt
volatile float wGyro = 2;


volatile float bgx = 0;
volatile float bgy = 0;
volatile float bgz = 0;
volatile float gyroRate[3] = {0,0,0};
volatile float lgyroRate[3] = {0,0,0};




//updated position vector
volatile float Rest[3] = {0,0,0};
volatile float Racc[3] = {0,0,0};
//updated position angles
volatile float Poss[3] = {0,0,0};
volatile float yaw = 0;

//first reading
volatile int firstReading = 1;
///////end gyro + accel variables/////////

///////Reciever REad variables//////////
//ADC reading
volatile int adc_11 = 0;
volatile int adc_5 = 0;

//reciever duty cycle
volatile int start = 0;
volatile int end = 0;
volatile int recDuty = 0;

//what edge 
volatile int fallingEdge = 0;
////////End reciever REad Variables/////

///////PWM+PID control variables///////
volatile int calibrateTime = 227; // 5second calibration time @ 22ms per count
volatile int calibrateCount = 0;
volatile int calibrated = 0;
volatile float goalPoss[3] = {0,0,0};
volatile float goalYaw = 0;

//xaxis tilt PID
volatile int xP = 100;
volatile int xI = 0;
volatile int xD = 0;
volatile float xPfactor = 0;
volatile float xIfactor = 0;
volatile float xDfactor = 0;
volatile float xFactor = 0;

//yaxis tilt PID
volatile int yP = 0;
volatile int yI = 0;
volatile int yD = 0;
volatile float yPfactor = 0;
volatile float yIfactor = 0;
volatile float yDfactor = 0;
volatile float yFactor = 0;

//yaw(x) PID
volatile int yawP = 10;
volatile int yawI = 0;
volatile int yawD = 0;
volatile float yawPfactor = 0;
volatile float yawIfactor = 0;
volatile float yawDfactor = 0;
volatile float yawFactor = 0;

//Current Error
volatile float xError = 0;
volatile float yError = 0;
volatile float yawError = 0;

//last Error
volatile float xLError = 0;
volatile float yLError = 0;
volatile float yawLError = 0;




///////End PWM+PID control variables////


// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_param;

//== Timer 2 interrupt handler ===========================================
//=============================
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{

	// adc_11 = ReadADC10(1);
 //    adc_5 = ReadADC10(0);
///////////////////////Gyro+Accel orientation////////////////////////
    
//////////////////End orientation////////////////////////////////

//////////////////PID Control////////////////////////////////////
	//wait for calibration
	if(!calibrated){
		if(calibrateCount < calibrateTime){
			calibrateCount++;
			goalPoss[0] += Poss[0];
			goalPoss[1] += Poss[1];
			goalPoss[2] += Poss[2];
			bgx += gyroRate[0];
			bgy += gyroRate[1];
			bgz += gyroRate[2];
		}
		else{
			calibrated = 1;
			goalPoss[0] /= calibrateTime;
			goalPoss[1] /= calibrateTime;
			goalPoss[2] /= calibrateTime;
			bgx /= calibrateTime;
			bgy /= calibrateTime;
			bgz /= calibrateTime;
			yaw = 0;
		}
	}
	//PID control
	else{
		//min 659
		//max 1200
		// // need to go to lab to get these values
		int leftServo = 929-(int)yawFactor-(int)xFactor;//center Pos +- have to check yawFactor and xtilt factor
		int rightServo = 929-(int)yawFactor+(int)xFactor;//center Pos +- have to check yawFactor and xtilt factor
		//logic for what to do with reciever value
		int leftMotor = 659;
		int rightMotor = 659;
		if(recDuty > 720){
			leftMotor = recDuty-(int)yFactor;//reciever value - ytilt factor
			rightMotor = recDuty+(int)yFactor;//reciever value + ytilt factor
		}
		else{
			leftMotor = 659;//off
			rightMotor = 659;//off
		}

		///range values////
		if(leftServo>1200){
			leftServo = 1200;
		}
		if(leftServo<659){
			leftServo = 659;
		}

		if(rightServo>1200){
			rightServo = 1200;
		}
		if(rightServo<659){
			rightServo = 659;
		}

		if(rightMotor>1200){
			rightMotor = 1200;
		}
		if(rightServo<659){
			rightMotor = 659;
		}

		if(leftMotor>1200){
			leftMotor = 1200;
		}
		if(leftServo<659){
			leftMotor = 659;
		}

		//////////////////PWM output/////////////////////////////////////
		//leftMotor control
		SetDCOC3PWM(leftMotor);
		//rightMotor control
		SetDCOC2PWM(rightMotor);
		//leftServo control
		SetDCOC5PWM(leftServo);
		//rightServo control
		SetDCOC4PWM(rightServo);

	}



///clear interupt/////
	mT2ClearIntFlag();

} // end ISR TIMER2



// == Capture 1 ISR ====================================================
// get pwm duty cycle input from reciever
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void)
{	
	//read reciever duty cycle
	if(!fallingEdge){
		mIC1ReadCapture();
		fallingEdge = 1;
		TMR3 = 0;
	}
	else{
		INTDisableInterrupts();
		recDuty = mIC1ReadCapture();
		INTEnableInterrupts();

		fallingEdge = 0;
	}
    mIC1ClearIntFlag();
    
}

static PT_THREAD (protothread_param(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
    	PT_YIELD_TIME_msec(22);

	    int gyro[3];
	    int accel[3];

	    int outputG[3];
	    int outputA[3];

	    float accelA[3];
	   	float gyroA[3];

	   	INTDisableInterrupts();
	    get_gyro(&outputG);
		get_accel(&outputA);
		INTEnableInterrupts();
	   

	    int i;

	    for(i=0; i<3; i++){
	    	//2s complement conversion
		    int negativeG = (outputG[i] & (1 << 15)) != 0;

			if (negativeG)
		  	gyro[i] = outputG[i] | ~((1 << 16) - 1);
			else
		  	gyro[i] = outputG[i];

		  	int negativeA = (outputA[i] & (1 << 15)) != 0;

		  	if (negativeA)
		  	accel[i] = outputA[i] | ~((1 << 16) - 1);
			else
		  	accel[i] = outputA[i];

		  	accelA[i] = (accel[i]*(0.122))/1000*9.80665;
		  	gyroRate[i] = gyro[i] * 0.00875; //245dps
		}
		
		//get accel contribution
		float norm = sqrtf(powf(accelA[0],2) + powf(accelA[1],2) + powf(accelA[2],2));
	    Racc[0] = accelA[0]/norm;
	    Racc[1] = accelA[1]/norm;
	    Racc[2] = accelA[2]/norm;
	    ax = Racc[0];
	    ay = Racc[1];
	    az = Racc[2];
	    gyroRate[0] *= -1;

	    if(firstReading){
	    	Rest[0] = Racc[0];
	    	Rest[1] = Racc[1];
	    	Rest[2] = Racc[2];
	    	firstReading = 0;
	    }
	    if(calibrated){
	    	gyroRate[0] -= bgx;
	    	gyroRate[1] -= bgy;
	    	gyroRate[2] -= bgz;
	    }

	    if((Rest[2]<0.1) && (Rest[2]> -0.1)){
	    	gx = Rest[0];
	    	gy = Rest[1];
	    	gz = Rest[2];
	    }
	    else{
		    //get gyro contribution
		    float AxzL = atan2f(Rest[0],Rest[2]);
		    float Axz = AxzL + (gyroRate[1]+lgyroRate[1])*0.5*0.025; // 22ms period

		    float AyzL = atan2f(Rest[1],Rest[2]);
		   	float Ayz = AyzL + (gyroRate[0]+lgyroRate[0])*0.5*0.025; // 22ms period

		    gx = sin(Axz)/(sqrtf(1 + powf(cos(Axz),2)*powf(tan(Ayz),2)));
		    gy = sin(Ayz)/(sqrtf(1 + powf(cos(Ayz),2)*powf(tan(Axz),2)));
		    if(Rest[2]>0)
		    	gz = sqrtf(1-powf(gx,2)-powf(gy,2));
		    else
		    	gz = -1*sqrtf(1-powf(gx,2)-powf(gy,2));
		}

		yaw += gyroRate[2]*0.025; // 22ms period
		if(yaw >= 360)yaw-=360;
		if(yaw<0)yaw+=360;

		//used for averaging rotation
		lgyroRate[0] = gyroRate[0];
		lgyroRate[1] = gyroRate[1];
		lgyroRate[2] = gyroRate[2];

		//update posistion
		Rest[0] = (Racc[0] + gx*wGyro)/(1+wGyro);
		Rest[1] = (Racc[1] + gy*wGyro)/(1+wGyro);
		Rest[2] = (Racc[2] + gz*wGyro)/(1+wGyro);

		float normTot = sqrtf(powf(Rest[0],2) + powf(Rest[1],2) + powf(Rest[2],2));
		Poss[0] = acosf(Rest[0]/normTot);
		Poss[1] = acosf(Rest[1]/normTot);
		Poss[2] = acosf(Rest[2]/normTot);

		if(calibrated){
			//attain error
			xError = goalPoss[0]-Poss[0];
			yError = goalPoss[1]-Poss[1];
			///need to do something with 360 here
			if(yaw > 180)yawError = 360 - yaw;
			else yawError = goalYaw - yaw;

			///x tilt control
			xPfactor = xError;
			xIfactor += xError;
			xDfactor = xError - xLError;
			xLError = xError;
			INTDisableInterrupts();
			xFactor = xP*xPfactor + xI*xIfactor + xD*xDfactor;
			INTEnableInterrupts();
			///y tilt control
			yPfactor = yError;
			yIfactor += yError;
			yDfactor = yError - yLError;
			yLError = yError;
			INTDisableInterrupts();
			yFactor = yP*yPfactor + yI*yIfactor + yD*yDfactor;
			INTEnableInterrupts();
			///yaw control
			yawPfactor = yawError;
			yawIfactor += yawError;
			yawDfactor = yawError - yawLError;
			yawLError = yawError;
			INTDisableInterrupts();
			yawFactor = yawP*yawPfactor + yawI*yawIfactor + yawD*yawDfactor;
			INTEnableInterrupts();
		}

    } // END WHILE(1)
	PT_END(pt);
} // thread 4


// === Main  ======================================================
void main(void) {
	//SYSTEMConfigPerformance(PBCLK);
  
	ANSELA = 0; ANSELB = 0; 
    
    //set up gyro/accel
    int master = i2c_master_setup();
    config_gyro_accel_default();
    config_mag_default();
    
	// === Config timer and output compare to make PWM ========
	// set up timer2 to generate the wave period -- SET this to 1 mSec!
	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, generate_period);
	// Need ISR to compute PID controller
	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
	mT2ClearIntFlag(); // and clear the interrupt flag


	//LeftMotor PWM
	// set up compare3 for PWM mode
	OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //
	// OC3 is PPS group 4, map to RPB9 (pin 18)
	PPSOutput(4, RPB10, OC3);
	OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,659,659);

	//rightMotor PWM
	// set up compare2 for PWM mode
	OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //
	// OC2 map to RPB11
	PPSOutput(2, RPB11, OC2);
	OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,659,659);

	//rightServo
	// set up compare4 for PWM mode
	OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //
	// OC4 map to RPA4
	PPSOutput(3, RPA4, OC4);
	OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,929,929);

	//leftServo
	// set up compare5 for PWM mode
	OpenOC5(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //
	// OC5 map to RPA2
	PPSOutput(3, RPA2, OC5);
	OpenOC5(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,929,929);



//////input capture to read PWM form reciever
	OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_64, 0xFFFF); //set prescal


	// === set up input capture ================================
	OpenCapture1( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
	// turn on the interrupt so that every capture can be recorded
	ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
	INTClearFlag(INT_IC1);
	// connect Rb13
	PPSInput(3, IC1, RPB13);

	mPORTASetPinsDigitalIn(BIT_4);
///////    
    

	// === config the uart, DMA, vref, timer5 ISR =============
	PT_setup();

	// === setup system wide interrupts  ====================
	INTEnableSystemMultiVectoredInt();


	// === config threads ==========
	// turns OFF UART support and debugger pin, unless defines are set
	PT_setup();

	// init the threads
	PT_INIT(&pt_param);
    

	// round-robin scheduler for threads
	while (1){
		PT_SCHEDULE(protothread_param(&pt_param));
			
	}
} // main

// === end  ======================================================

