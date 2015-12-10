
/*************************************
//// Shield Interfaces Description /////////////////

//// DIP /////////////////

Standard Mode:

1	2	3	4
X	0  	0	X	Raw Focusing Mode
X	1	0	X	Servo Alignment
X	0	1	X	Derivative Focusing Mode
X	1	1	X	Line Following Mode
Pin 1:	1 HBRIDGE enabled, RunningState;	0 HBRIDGE DISSABLED;
Pin 4:	0 SingleSpeed_SingleUart;	1 DualSpeed_NoUart

Race Mode:

Pin 1: HBRIDGE enabled
Pin 2: S-Mode enabled
Pin 3: Cross Detection enabled
Pin 4: Stopline Detection enabled


//// LEDs /////////////////		0 LED_ON	x LED_OFF

0xxx	Line lost
x0xx	Line temporarily lost
xx0x	Right edge following
xxx0	Left edge following
xx00	Double line following
0xx0	Cross Section detected
00xx	Stop Line detected
x00x	S-Mode active
0000	Low battery


//// Pots ///////////

Standard mode:
Pot_1	Max Speed percent
Pot_2	No Function

Race mode:
Pot_1	Max Speed percent
Pot_2	Friction Coefficient Correction - rotate clockwise to decrease friction coefficient


//// Push Buttons ///////////

Switch_1	Stop Line reset

**************************************/

#include "main.h"

#define CHANNEL_0 0
#define CHANNEL_1 1
#define PUSH_BUTTON_1 1
#define PUSH_BUTTON_2 2
//#define TESTING


#define BLUETOOTH_FLAG	-128

#define FLOAT_TO_CHAR(x)	(x >= 0.0) ? (signed char)(x + 0.5) : (signed char)(x - 0.5)		
#define LIMIT_255(x)		(x <= 254.5f && x >= 0.0f && !(x >= 127.5f && x <= 128.5f)) ? x : ((x > 255.0f) ? 255.0f : ((x >= 127.5f && x <= 128.5f) ? 127.0f : 0.0f))
#define LIMIT_ABS_127(x)	(x <= 126.9f && x >= -126.9f) ? x : ((x > 126.9f) ? 126.9f : -126.9f)

#define LINE_SCAN_IMAGE(x)	LINE_SCAN_IMAGE_EXAPND(x)	// Second level of indirection needed to expand x defined as macro
#define LINE_SCAN_IMAGE_EXAPND(x)	LineScanImage##x

// carState struct initialization - keeps the main parameters of the car state
static carState_s carState =
{   .motorState = FORCED_DISABLED, .UARTSpeedState = UNDEFINED, 
    .lineDetectionState = LINE_LOST, .lineScanState = NO_NEW_LINESCAN_IMAGE, 
	.detectedType = NONE, .edge = NO_EDGE, .crossSection = NO, .crossSectionPosition = NO_CROSS,
	.lineDistance = STRAIGHT_DISTANCE_DEFAULT, .carMode = NORMAL_MODE, .sMode = S_MODE_OFF};

uint16_t loop_time = 0, loop_begin = 0;
float batteryLevel = 0;
static float targetSpeed = 0;
float incline_speed;
float speedL = 0;
float speedR = 0;
static float servoValue = 0;
extern int32_t newExposure;
float max_speed_percent = 30;
uint8_t received_byte1 = 0; //received byte from bluetooth 0 to 255
uint8_t received_byte2 = 0;
uint8_t count = 0; //bluetooth count
static uint8_t stoplineJustDetected = 0;
float friction_correct = 0;
float i =0; // for testing pwm speed for torque control
uint8_t camera_output[128];
#ifdef S_MODE_ENABLE
uint8_t s_mode_enable = 1;
#else
uint8_t s_mode_enable = 0;
#endif

#ifdef CROSS_DETECTION_ENABLE
uint8_t cross_section_enable = 1;
#else
uint8_t cross_section_enable = 0;
#endif

#ifdef STOPLINE_ENABLE
uint8_t stopline_enable = 1;
#else
uint8_t stopline_enable = 0;
#endif

// Test timer ////////////////////
uint32_t test_begin = 0;
uint32_t test_time  = 0;
//////////////////////////////////


int main(void)
{
	TFC_Init(&carState);
	while(TFC_Ticker[5]<=10000){
	}
	
	#ifdef ACCELEROMETER_ENABLE
		Init_I2C();
		Init_Accelerometer();
	#endif

	//LCDinit();

	while (1)
	{
		loop_begin = TFC_Ticker[5];
		#ifdef TERMINAL_ENABLED
			TFC_Task();
		#endif
		
		#ifndef RACE_MODE_CONTROLS	
			evaluateUARTorSpeed(&carState);		// Checks DIP switch
		#endif
		
		evaluateMotorState(&carState);		// Checks DIP switch
		
		LEDfeedback(&carState);				// Gives feedback for car state by the 4 LEDs
		
		TERMINAL_PRINTF("%X,", getSpeed(CHANNEL_1));
		
		#ifndef RACE_MODE_CONTROLS
			switch ((TFC_GetDIP_Switch() >> 1) & 0x03)
			{
			default:
			case 0:
				rawFocussingMode(&carState);
				break;
	
			case 1:
				servoAlignment();
				break;
	
			case 2:
				derivativeFocussingMode(&carState);
				break;
	
			case 3:
				lineFollowingMode(&carState);
				break;
			}
		#else
			if(((TFC_GetDIP_Switch() >> 1) & 0x01) == 1)
				s_mode_enable = 1;
			else
				s_mode_enable = 0;
			if(((TFC_GetDIP_Switch() >> 2) & 0x01) == 1)
				cross_section_enable = 1;
			else
				cross_section_enable = 0;
			if(((TFC_GetDIP_Switch() >> 3) & 0x01) == 1)
			{
				stopline_enable = 1;
			}
			else
			{
				stopline_enable = 0;
			}
			
			friction_correct = ((0 + 1.0f)/2)*MAX_FRICTION_CORRECT;
			
			lineFollowingMode(&carState);
		#endif
		
		
		loop_time = TFC_Ticker[5] - loop_begin;
	}
	
	return 0;
}


void lineFollowingMode(carState_s* carState)
{
	static lineScanState_t steeringControlUpdate;
	static uint32_t totalIntensity = 0;
	
	if(TFC_GetPush_Button(PUSH_BUTTON_1) == 1)	// Stop line detection reset
	{
		stoplineJustDetected = 0;
		carState->lineDetectionState = LINE_LOST;
	}

	if (TFC_Ticker[6] >= 1000)	// Read battery every 100 ms
	{
		TFC_Ticker[6] = 0;
		batteryLevel = TFC_ReadBatteryVoltage();
	}
	
	#ifdef TELEMETRY_ENABLE
	if (TFC_Ticker[4] >= 50)	// Send telemetry every 5 ms
	{
		TFC_Ticker[4] = 0;
		telemetrySendData();
		telemetryReadData();
	}
	#endif
	
	if(TFC_Ticker[5]>=50){ 
		TFC_Ticker[5]=0;
		LCDwriteState(carState); 				//gives feedback for car state by the LCD display
	}
	
	#ifdef ACCELEROMETER_ENABLE
		if(TFC_Ticker[8]>=1000){
			TFC_Ticker[8]=0;
			//accel_z=getZAcc();
			if(getZAcc<-1){
				incline_speed=max_speed_percent+10;
			}
			else{
				incline_speed=max_speed_percent;
				}
		}
	#endif
	#ifndef ACCELEROMETER_ENABLE
		incline_speed=max_speed_percent;
	#endif
		
	if (carState->lineScanState == LINESCAN_IMAGE_READY)
	{
		steeringControlUpdate = LINESCAN_IMAGE_READY;	
	//	test_begin = TFC_Ticker[5];
		findLine(LineScanImage0, LineScanImage1, carState);
	//	test_time = TFC_Ticker[5] - test_begin;
		processRaceLine(carState);
	}
		
	if (TFC_Ticker[0] >= 100)	// Update reduced to 10 ms from 20 ms because servoValue used for speed control now
	{
		TFC_Ticker[0] = 0;
		servoValue = getDesiredServoValue(carState->raceLineCenter, 0, &steeringControlUpdate);
		
		if(servoValue>0){ //to offset not steering left enough
				servoValue+=servoValue*0.3f; //
				if(servoValue>STEERING_LIMIT_UPPER)
					{
					servoValue=STEERING_LIMIT_UPPER;
					}
			}
	
		
		TFC_SetServo(0, servoValue - SERVO_MOUNT_DIRECTION*STEERING_OFFSET);
		servoValue = servoValueAverage(servoValue);		// Low-pass of servoValue to be used for the radius mapping
	}

	if (carState->lineScanState == LINESCAN_IMAGE_READY) //Stopline detection and exposure control are performed after servo has been updated
	{
		carState->lineScanState = NO_NEW_LINESCAN_IMAGE;
		if(stopline_enable == 1)
		{
			findStop(carState);
		}
		totalIntensity = getTotalIntensity(LineScanImage0);
		TFC_SetLineScanExposureTime(calculateNewExposure(totalIntensity, TARGET_TOTAL_INTENSITY));
	}

	if (carState->lineDetectionState == LINE_FOUND || carState->lineDetectionState == LINE_TEMPORARILY_LOST)
	{
		targetSpeed = targetSpeedAverage(getDesiredSpeed(carState, ((500 + 1.0f)/2), getRadiusRoot(servoValue)));
		
		float activeDifferentialModifier[] =
			{ getActiveDifferentialModifier(servoValue, CHANNEL_0), getActiveDifferentialModifier(servoValue, CHANNEL_1) };
		
		// Speed measurement depending on the particular mode which is used
		#if (SPEED_DETECTION_MODE == 0)
			speedR = getSpeed(CHANNEL_0);
			speedL = getSpeed(CHANNEL_1);
		#elif (SPEED_DETECTION_MODE == 1)
			speedR = (getSpeed(CHANNEL_0) + getSpeed(CHANNEL_1))/2;
			speedL = speedR;
		#elif (SPEED_DETECTION_MODE == 2)
			speedR = getSpeed(CHANNEL_0);
			speedL = speedR;
		#elif (SPEED_DETECTION_MODE == 3)
			speedL = getSpeed(CHANNEL_1);
			speedR = speedL;
		#endif

		if (carState->UARTSpeedState == DUAL_SPEED_NO_UART)
		{
			TFC_SetMotorPWM(
					(incline_speed/100)*getDesiredMotorPWM(targetSpeed * activeDifferentialModifier[0], speedR, isANewmeasurementAvailable(CHANNEL_0), CHANNEL_0),
					(incline_speed/100)*getDesiredMotorPWM(targetSpeed * activeDifferentialModifier[1], speedL, isANewmeasurementAvailable(CHANNEL_1), CHANNEL_1));
		}
		else if (carState->UARTSpeedState == SINGLE_SPEED_SINGLE_UART)
		{
			// Missing setup - Normally this mode is not used
		}
	}
	else if (carState->lineDetectionState == LINE_LOST)
	{
		TFC_HBRIDGE_DISABLE;
		TFC_SetMotorPWM(0, 0);
	}
	else if (carState->lineDetectionState == STOPLINE_DETECTED)
	{
		if(stopline_enable == 1)
		{	
			static uint32_t stoplineDetectedMoment = 0;
			static uint32_t stopDelayTime = 0;
			
			if(stoplineJustDetected == 0)
			{
				stoplineJustDetected = 1;
				stoplineDetectedMoment = TFC_Ticker[5];
				
				float lookAhead = LOOK_AHEAD_DISTANCE;
				float lastSpeed = ((speedL+speedR)/2);
				
				if(lastSpeed < 10.0f)
				{
					lookAhead = lookAhead + 0.25f;
				}
				
				stopDelayTime = (uint32_t)(((lookAhead)/(lastSpeed*3.1416f/20.0f))*10000);
			}
			else if((TFC_Ticker[5] - stoplineDetectedMoment) >= stopDelayTime)
			{
				float speedMeasurement[] = { getSpeed(CHANNEL_0), getSpeed(CHANNEL_1) };
				
				if (speedMeasurement[0] > 2.0f || speedMeasurement[1] > 2.0f)
				{
					TFC_SetMotorPWM(-0.5f, -0.5f);
				}
				else
				{
					TFC_SetMotorPWM(0, 0);
				}
			}
		}	
	}
}

void rawFocussingMode(carState_s* carState)
{
	if (TFC_Ticker[0] >= 200 && carState->lineScanState == LINESCAN_IMAGE_READY)
	{
		TFC_Ticker[0] = 0;
		carState->lineScanState = NO_NEW_LINESCAN_IMAGE;

		TFC_SetServo(0, -SERVO_MOUNT_DIRECTION*STEERING_OFFSET);
		TFC_HBRIDGE_DISABLE;
		TFC_SetMotorPWM(0, 0);

		TFC_SetLineScanExposureTime(calculateNewExposure(getTotalIntensity(LineScanImage0), TARGET_TOTAL_INTENSITY));
		
		#ifdef TERMINAL_ENABLED
			for (uint8_t i = 0; i < 128; i++)
			{
				TERMINAL_PRINTF("%X,", LINE_SCAN_IMAGE(CAMERA_SERIAL_DATA)[i]);
			}
			TERMINAL_PRINTF("\r\n");
		#endif		
	}
}

void derivativeFocussingMode(carState_s* carState)
{
	if (TFC_Ticker[0] >= 200 && carState->lineScanState == LINESCAN_IMAGE_READY)
	{
		TFC_Ticker[0] = 0;
		carState->lineScanState = NO_NEW_LINESCAN_IMAGE;

		TFC_SetServo(0, -SERVO_MOUNT_DIRECTION*STEERING_OFFSET);
		TFC_HBRIDGE_DISABLE;
		TFC_SetMotorPWM(0, 0);

		TFC_SetLineScanExposureTime(calculateNewExposure(getTotalIntensity(LineScanImage0), TARGET_TOTAL_INTENSITY));
		int16_t temp[128];
		
		test_begin = TFC_Ticker[5];
		edgeDetection(LINE_SCAN_IMAGE(CAMERA_SERIAL_DATA), temp, 128);
		test_time = TFC_Ticker[5] - test_begin;
		
		#ifdef TERMINAL_ENABLED
			for (uint8_t i = 0; i < 128; i++)
			{
				TERMINAL_PRINTF("%X,", abs(temp[i]));
			}
			TERMINAL_PRINTF("\r\n");
		#endif	
	}
}


void TFC_Init(carState_s* carState)
{
	TFC_InitClock();
	TFC_InitSysTick();
	TFC_InitGPIO();
	TFC_InitServos();
	TFC_InitMotorPWM();
	TFC_InitADCs(carState);
	TFC_InitLineScanCamera();
	
	#ifdef CHANGE_BLUETOOTH_BAUD_RATE
		TFC_InitUARTs(SDA_SERIAL_BAUD, 38400);	// 38400 is default for the BL module in AT mode 2
		int dummy_timee = TFC_Ticker[0];
		while(TFC_Ticker[0] - dummy_timee < 30000)
		{}
		TFC_BluetoothModuleSetBaud(BLUETOOTH_BAUD);		// Need to manually power the AT pin on the module
	#endif
	
	TFC_InitUARTs(SDA_SERIAL_BAUD, 9600);
	
	#ifdef TERMINAL_ENABLED
		TFC_InitTerminal();
	#endif
		
	TFC_HBRIDGE_DISABLE;
	TFC_SetMotorPWM(0, 0);
	TFC_InitSpeedSensor();
	preloadProbabilityTables(); //Prevents probability tables for stop line evaluation from being created too late
	generateKernel();
	
	#ifdef RACE_MODE_CONTROLS
	PORTA_PCR2 &= 0xFFFFF8FF;
	PORTA_PCR2 |= 0x00000300;
	disable_irq(INT_UART0-16);
	carState->UARTSpeedState = DUAL_SPEED_NO_UART;
	#endif
	
	TFC_SetServo(0, 0);
}

void TFC_Task()
{
#if defined(TERMINAL_USE_SDA_SERIAL)
	TFC_UART_Process();
#endif
	TFC_ProcessTerminal();
}

void evaluateMotorState(carState_s* carState)
{
	if ((TFC_GetDIP_Switch() & 0x01) == 1)
	{
		TFC_HBRIDGE_ENABLE;
		carState->motorState = RUNNING;
	}
	else if ((TFC_GetDIP_Switch() & 0x01) == 0)
	{
		TFC_HBRIDGE_DISABLE;
		TFC_SetMotorPWM(0, 0);
	}
}

void evaluateUARTorSpeed(carState_s* carState)
{
	if (((((TFC_GetDIP_Switch() >> 3) & 0x01) == 0x00) && ((PORTA_PCR2 >> 8) & 0x00000007) != 0x00000002) || carState->UARTSpeedState == UNDEFINED) //Single UART single sensor
	{
		PORTA_PCR2 &= 0xFFFFF8FF;
		PORTA_PCR2 |= 0x00000200;
		enable_irq(INT_UART0-16);
		carState->UARTSpeedState = SINGLE_SPEED_SINGLE_UART;
	}

	else if (((((TFC_GetDIP_Switch() >> 3) & 0x01) == 0x01) && ((PORTA_PCR2 >> 8) & 0x00000007) != 0x00000003) || carState->UARTSpeedState ==UNDEFINED) //Dual sensor no uart
	{
		PORTA_PCR2 &= 0xFFFFF8FF;
		PORTA_PCR2 |= 0x00000300;
		disable_irq(INT_UART0-16);
		carState->UARTSpeedState = DUAL_SPEED_NO_UART;
	}
}

void LEDfeedback(carState_s* carState)
{
	if(batteryLevel > LOW_BATTERY)
	{
		if(carState->lineDetectionState == STOPLINE_DETECTED)	// Also set in lineFollowingMode()
		{
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
		#ifdef CROSS_DETECTION_ENABLE
		else if(carState->crossSection == YES)
		{
			GPIOB_PSOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
		#endif
		else if(carState->sMode == S_MODE_ON)
		{
			GPIOB_PCOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if(carState->detectedType == DOUBLE_EDGE)
		{
			GPIOB_PSOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->edge == LEFT_EDGE)
		{	
			GPIOB_PSOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->edge == RIGHT_EDGE)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->lineDetectionState == LINE_TEMPORARILY_LOST)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->lineDetectionState == LINE_LOST)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
	}
	else
	{
		GPIOB_PSOR |= (1<<8);
		GPIOB_PSOR |= (1<<9);
		GPIOB_PSOR |= (1<<10);
		GPIOB_PSOR |= (1<<11);		
	}
}

void servoAlignment()		// Helps setting the servo offset value by debugging
{
	if (TFC_Ticker[0] >= 200)
	{
		TFC_Ticker[0] = 0;
		float offset = 30000 * 0.15f;
		TFC_SetServo(0, offset);
	}
}

void telemetrySendData()
{
	uart_putchar(UART2_BASE_PTR, BLUETOOTH_FLAG);
	
	// 1 - Track Centre Detected by the Camera
	uart_putchar(UART2_BASE_PTR, (signed char)carState.lineCenter);
	// 2 - Calculated Target Speed
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*targetSpeed)));
	// 3 - Speed of the right wheel
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_0))));
	// 4 - Speed of the left wheel
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_1))));
	// 5 - PWM output to right motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*TFC_GetMotorPWM(CHANNEL_0)));
	// 6 - PWM output to left motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*TFC_GetMotorPWM(CHANNEL_1)));
	// 7 - Current of the right motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(60*TFC_ReadMotorCurrent(CHANNEL_0))));
	// 8 - Current of the left motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(60*TFC_ReadMotorCurrent(CHANNEL_1))));
	// 9 - Servo Value
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*servoValue));
	// 10 - Detected edge type - Left:100, Right:-100, Track: 0, None:50
	uart_putchar(UART2_BASE_PTR, (signed char)(carState.edge==NO_EDGE)?(50):((carState.edge==BOTH_EDGE)?(0):((carState.edge==LEFT_EDGE)?(100):(-100))));
	// 11 - Camera Exposure Time, 1 unit = 0.1 ms
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255((float)newExposure*0.025f)));
	// 12 - Loop time
	uart_putchar(UART2_BASE_PTR, (signed char)loop_time);
	// 13 - Line Distance
	uart_putchar(UART2_BASE_PTR, (signed char)carState.lineDistance);
	
}

void telemetryReadData(){
	
	if(count==0){
		//if something is on the UART, get it and put it in received_byte
		if(uart_getchar_present(UART2_BASE_PTR)){
			received_byte1 = uart_getchar(UART2_BASE_PTR);
			GPIOB_PSOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
			count++;
		}
	}
	else if(count>0){
		if(uart_getchar_present(UART2_BASE_PTR)){
			received_byte2 = uart_getchar(UART2_BASE_PTR);
			count=0;
		}
	}
	
	if(received_byte1!=0 && received_byte2!=0){
		switch(received_byte1){
		case 255: //SPEED
			BluetoothSetSpeed(received_byte2);
			break;
		case 251: //LED
			BluetoothSetLED(received_byte2);
			break;
		}
		//received_byte1=0;
		//received_byte2=0;
	}
	
}

void BluetoothSetSpeed(uint8_t i){
	max_speed_percent=i;
		if(max_speed_percent>100){
			max_speed_percent=100;
		}
}

void BluetoothSetLED(uint8_t i){	
		switch(i){
			case 1: 
				GPIOB_PSOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
				break;
			case 2:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PSOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
				break;
			case 3:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PSOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);			
				break;
			case 4: 
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PSOR |= (1<<11);
				break;
			default:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
			}
}

void processRaceLine(carState_s* carState)
{
	if(s_mode_enable)
	{	
		if (carState->lineCenter < S_LINE_THRESHOLD 
				&& carState->lineDistance > S_DISTANCE_THRESHOLD
				&& carState->crossSection == NO)
		{
			if(TFC_Ticker[7] > S_TIME_THRESHOLD)
			{
				carState->sMode = S_MODE_ON;
			}
		}
		else
		{
			carState->sMode = S_MODE_OFF;
			TFC_Ticker[7] = 0;
		}
	}
	
	static float previousRaceLine = 0;
	
	if (carState->sMode == S_MODE_ON)
	{
		previousRaceLine = (S_LOWPASS_COEFF*previousRaceLine + (1 - S_LOWPASS_COEFF)*((float)carState->lineCenter));
		carState->raceLineCenter = (uint8_t)(previousRaceLine + 0.5f);
	}
	else
	{
		carState->raceLineCenter = carState->lineCenter;
		previousRaceLine = (float)carState->lineCenter;
	}
}

float targetSpeedAverage(float targetSpeed)
{
	static float previousTargetSpeed = 0;
	previousTargetSpeed = 0.9f*previousTargetSpeed + 0.1*targetSpeed;
	return previousTargetSpeed;
}

float servoValueAverage(float servoValue)
{
	static float previousServoValue = 0;
	previousServoValue = SERVO_LOWPASS_COEFF*previousServoValue + (1-SERVO_LOWPASS_COEFF)*servoValue;
	return previousServoValue;
}
