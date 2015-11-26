/*
 * LineDetectionDouble.h
 *
 *  Created on: Oct 29, 2014
 *      Author: Miroslav Dobrev
 */

#ifndef LINEDETECTIONDOUBLE_H_
#define LINEDETECTIONDOUBLE_H_

#include "TFC/TFC.h"
#include "Probability.h"
#include "EdgeDetection.h"
#include "Settings.h"
#include "typedefs_structs\carState_s.h"


// Find Line ////////////////////////////////////////////////////////////////////
				//TRACK_WIDTH_MEAN moved to Settings.h
#define TRACK_WIDTH_SD 5
#define TRACK_CENTER_DIFFERENCE_SD 10
#define TRACK_CENTER_DIFFERENCE_MEAN 0

#define MIN_CERTAINTY 0.2f

#define MAX_NUMBER_OF_TRANSITIONS 64
#define MAX_NUMBER_OF_LINES 32
#define MAX_NUMBER_OF_TRACKS 100
#define MAX_NUMBER_OF_DOUBLE_LINES 32
#define MAX_NUMBER_OF_STOP_LINES 32

#define DOUBLE_LINE_POSITION_DIFFERENCE_SD 10
#define DOUBLE_LINE_POSITION_DIFFERENCE_MEAN 0
/////////////////////////////////////////////////////////////////////////////////////

// Find StopLine ////////////////////////////////////////////////////////////////////
#define MIN_STOPLINE_CERTAINTY 0.4f

#define STOPLINE_SIDE_WIDTH_SD 10
#define STOPLINE_SIDE_WIDTH_MEAN 18			// Replaced by STOPLINE_SIDE_PERCENTAGE
#define STOPLINE_SIDE_PERCENTAGE 0.1981f	// Percent of track width
#define STOPLINE_GAP_WIDTH_SD 4
#define STOPLINE_GAP_WIDTH_MEAN 12			// Replaced by STOPLINE_GAP_PERCENTAGE
#define STOPLINE_GAP_PERCENTAGE	0.1321f		// Percent of track width
/////////////////////////////////////////////////////////////////////////////////////

typedef enum {blackToWhite, whiteToBlack} transitionDirection_t;
typedef enum {LEFT, RIGHT} doubleLineType_t;

struct detectedTransition_s{
	uint8_t position;
	transitionDirection_t direction;
	uint16_t derivative;
};

struct detectedTransitions_s{
	struct detectedTransition_s transition[MAX_NUMBER_OF_TRANSITIONS];
	uint8_t numberOfTransitions;
};

struct detectedLine_s{
	uint8_t start;
	uint8_t end;
	uint8_t width;
	int8_t center;
	float certainty;
	float widthCertainty;
	float relativePositionCertainty;
	float relativeWidthCertainty;
};

struct detectedLines_s{
	struct detectedLine_s line[MAX_NUMBER_OF_LINES];
	uint8_t numberOfLines;
};

//// Double Line /////////////////////////////////////////////////

struct detectedTrack_s{
	uint8_t start;
	uint8_t end;
	uint8_t width;
	int8_t center;
	float certainty;
	float widthCertainty;
	float relativePositionCertainty;
};

struct detectedTracks_s{
	struct detectedTrack_s track[MAX_NUMBER_OF_LINES];
	uint8_t numberOfTracks;
};

struct detectedDoubleLine_s{
	uint8_t position;
	doubleLineType_t lineType;
	int16_t derivative;
	float relativePositionCertainty;
	float derivativeCertainty;
	float certainty;
};

struct detectedDoubleLines_s{
	struct detectedDoubleLine_s line[MAX_NUMBER_OF_DOUBLE_LINES];
	uint8_t numberOfLines;
	int16_t maxDerivativeLeft;
	int16_t maxDerivativeRight;
};

//////////////////////////////////////////////////////////////////

struct stopLine_s{
	struct detectedLine_s line[2]; 	//Shouldn't actually need multiple copies of the lines, pointers to their existing 
									//locations would suffice. Must however be careful to ensure that the new width
									//probabilities don't start overwriting each other.
	int8_t gap;
	float gapWidthCertainty;
	float certainty;
};

struct stopLines_s{
	struct stopLine_s stopLine[MAX_NUMBER_OF_STOP_LINES];
	uint8_t numberOfStopLines;
};

void findLine(volatile uint16_t* lineScanImage, volatile uint16_t* lineScanImage1, carState_s* carState);
int8_t findTransitions(struct detectedTransitions_s* detectedTransitions, int16_t* derivative, uint16_t derivativeThreshold);
uint8_t findLineDistance(int16_t* derivative, uint16_t derivativeThreshold, uint8_t maxDistance);
int8_t transitionsToLines(struct detectedLines_s* detectedLines, struct detectedTransitions_s* detectedTransitions);
void transitionsToTracks(struct detectedTracks_s* detectedTracks, struct detectedTransitions_s* detectedTransitions);
void transitionsToDoubleLines(struct detectedDoubleLines_s* detectedDoubleLines, struct detectedTransitions_s* detectedTransitions);
void weightTracks(struct detectedTrack_s* previousTrack, struct detectedTracks_s* detectedTracks);
void weightDoubleLines(struct detectedDoubleLine_s* previousLine, struct detectedDoubleLines_s* detectedDoubleLines, doubleLineType_t lineType);
void preloadProbabilityTables();
void findStop(carState_s* carState);
int8_t weightStopLines(struct stopLines_s* stopLines);

#endif /* LINEDETECTIONDOUBLE_H_ */


