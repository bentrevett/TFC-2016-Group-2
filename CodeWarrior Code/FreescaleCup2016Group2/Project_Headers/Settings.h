/*
 * Settings.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Miroslav Dobrev
 *      
 *  Keeps all often used settings.    
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

extern uint8_t cross_section_enable;
extern uint8_t stopline_enable;

//// main.c ///////////////////////////////////////////////////////////////////////////////////////
//#define RACE_MODE_CONTROLS				// Uncomment to enable Race Mode Controls
										// DIP1 Motors, DIP2 S-Mode, DIP3 Cross Detection, DIP4 Stopline Detection 
										// POT_1 Max Speed		POT_2 Friction Coeff Correction

//#define ACCELEROMETER_ENABLE
#define TELEMETRY_ENABLE

#define TARGET_TOTAL_INTENSITY 300000	// Sets the target intensity of the camera image
#define LOW_BATTERY				 7.5f	// LED low battery threshold, 7.5V = 6 x 1.25V per cell
#define LOOK_AHEAD_DISTANCE		0.40f	// Measured in (m) from the front end of the car (used only for stopline timing)
										// Reduce to make the car stopping quicker after stopline detected

#define TERMINAL_ENABLED	     		// Comment out to switch terminal off
#define CAMERA_SERIAL_DATA		0		// Main camera: 0, Second camera: 1
#define SPEED_DETECTION_MODE	0		// 0 - independent, 1 - averaged, 2 - right wheel, 3 - left wheel
//#define CHANGE_BLUETOOTH_BAUD_RATE	// Comment out to change Bluetooth module baud rate
										// BT module should be put to AT mode by applying 3.3V to KEY pin on startup

//#define STOPLINE_ENABLE					// Comment out to disable stopline detection
#define STOPLINE_START_TIME	 20000		// Min elapsed time from reset to activate stopline detection 10000 = 1s
///////////////////////////////////////////////////////////////////////////////////////////////////

//// SteeringControl.c ////////////////////////////////////////////////////////////////////////////
#define Kp 0.010f 						// PID Constants    Germany,Italy: 0.010f
#define Ki 0.040f						// 					Germany,Italy: 0.040f
#define Kd 0.00070f						// 					Germany,Italy: 0.00070f

#define INTEGRAL_LIMIT 		   1.0f		// Integral anti-windup
#define STEERING_LIMIT_UPPER   0.43f 	// Right Turn Limit
#define STEERING_LIMIT_LOWER   -0.3f	// Left Turn Limit
#define STEERING_OFFSET		   0.05f  // Servo center point offset, positive to turn left
#define SERVO_MOUNT_DIRECTION -1		// Changes the direction of the servo depending how the servo is mounted
#define SERVO_LOWPASS_COEFF	   0.800f	// Servo Lowpass Coeff - Affects only the speed control and active diff
///////////////////////////////////////////////////////////////////////////////////////////////////

//// TargetSpeedControl.c ////////////////////////////////////////////////////////////////////////////
#define MAX_STRAIGHT_SPEED  30.0f	// Maximum speed in straights
#define MAX_SPEED  21.0f 		// Maximum Possible Target Speed in Normal Mode
#define MIN_SPEED   9.0f 		// Minimum Target Speed, Set low value if Servo Mapping is used for Speed Control

#define K_LIN 	  0.008f		// Coeff for old linear algorithm	
#define ERROR_THRESHOLD		0	// Min track center error to start reducing the speed (pixels) - not important if servo mapping used

#define SPEED_MAP_ENABLE				// Uncomment to enable radius mapping for speed control
#define FRICTION_COEFF_ROOT		 1.1f	// Root of Friction coeffiecient with the track
#define MAX_FRICTION_CORRECT	-0.2f;	// Max value to substract from friction coeff with POT_2 in Race Mode
///////////////////////////////////////////////////////////////////////////////////////////////////

//// LineDetectionDouble.h ////////////////////////////////////////////////////////////////////////
#define DOUBLE_LINE_DETECTION		// Uncomment to switch to double line + single line following
#define CAMERA_TWO_ENABLE			// Uncomment to enable second camera
#define LINE_DISTANCE_ENABLE		// Uncomment to enable line distance detection by second camera

#define DERIVATIVE_THRESHOLD 350 				// Threshold for main camera
#define DERIVATIVE_THRESHOLD_STOP		110		// Stopline threshold
#define DISTANCE_DERIVATIVE_THRESHOLD	160		// Threshold for second camera

#define DISTANCE_MAX		120		// Max distance that can be detected in pixels
#define STRAIGHT_DISTANCE_DEFAULT	 120	// Default value when no edge is detected

#define TRACK_WIDTH_MEAN 	106		// Track Width in pixels seen by the camera
#define TRACK_WIDTH_MIN		(0.75*TRACK_WIDTH_MEAN)		// Affects only double line following
#define TRACK_WIDTH_MAX		(1.25*TRACK_WIDTH_MEAN)		//

#define CAMERA_CENTER_OFFSET	  -3	// Corrects non-perfect mounting of the camera (in pixels)
										// Positive if camera looks to the right of the center

#define MAX_LOST_LINE_DURATION	5000	// Time to stop after losing the line, 1s = 10000

	// Cross Section Detection
#define CROSS_DETECTION_ENABLE				// Uncomment to enable Cross Section Detection
#define CROSS_DERIVATIVE_THRESHOLD  4.5f	// Increase for less false positives
#define CROSS_MIN_SPEED				3.0f	// Minimum speed to detect cross section
#define CROSS_MINIMUM_DISTANCE		69		// Minimum distance to the inside of the cross section
#define CROSS_LINE_ERROR_REDUCTION	0.965f	// Decrease for faster straighten up of steering
#define CROSS_MID_TIME_THRESHOLD    50		// Min line temp lost time to enter mid cross section (10 = 1ms) 

	// Straight Mode
#define STRAIGHT_MODE_ENABLE				// Uncomment to enable Straight Mode
#define STRAIGHT_DISTANCE_THRESHOLD	 81		// 81 ~ 140cm (20cm before main camera seeing corner) Min distance to trigger Straight Mode
#define STRAIGHT_ERROR_THRESHOLD	  6		// Max lineCenter error to trigger Straight Mode (pixels)
#define STRAIGHT_CRITICAL_ERROR		  7		// Critical error to disable Straight Mode

	// S-Mode
//#define S_MODE_ENABLE						// Uncomment to enable S-mode
#define S_LINE_THRESHOLD		  10		// Max LineCentre to enter S-mode
#define S_DISTANCE_THRESHOLD	  70		// Min lineDistance to enter S-mode
#define S_TIME_THRESHOLD		4500		// Min time to start S-Mode 1s = 10000
#define S_LOWPASS_COEFF		   0.900f		// S-mode low-pass coeff, put to Zero to disable
///////////////////////////////////////////////////////////////////////////////////////////////////

//// ActiveDifferential.c /////////////////////////////////////////////////////////////////////////
#define K_ACTIVE 0.400f		// Active differenctial coeff (when servo mapping is not used)
							// Theoretical max value should be around 0.486f
#define ACTIVE_MAP_ENABLE			// Uncomment to enable servo mapping for active diff
#define K_ACTIVE_COMP		1.0f	// Correction of modifier calculated using steering radius mapping
///////////////////////////////////////////////////////////////////////////////////////////////////

//// AutoExposure.c ///////////////////////////////////////////////////////////////////////////////
#define Kp_EXPOSURE		0.01f		// Exposure correction coeff
#define MAX_EXPOSURE 	10000		// Maximum allowed exposure time, 1 ms = 1000
#define MIN_EXPOSURE  	 6000		// Minimum allowed exposure time
///////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* SETTINGS_H_ */

