/*
 * ServoMapping.c
 *
 *  Created on: April 17, 2015
 *  Author: Miroslav Dobrev
 *  
 *  Handles the mapping between servoValue and steering radius.
 */

#include "ServoMapping.h"

#define MAP_ARRAY_SIZE	101

#define SERVO_VALUE_MIN		0.04f	// Minimum servoValue that is mapped
#define SERVO_VALUE_MAX		0.36f	// Max servoValue that is mapped
#define STEP_RECIPROCAL	  (MAP_ARRAY_SIZE-1)/(SERVO_VALUE_MAX - SERVO_VALUE_MIN)	// 1/servoValueStep	

float getRadiusRoot(float servoValue)
{	
	if(servoValue < 0.0f)
	{
		servoValue = (-servoValue);
	}
		
	if(servoValue < SERVO_VALUE_MIN)
	{
		return 100000.0f;		// Huge radius - practically moving straight.
	}
	else if(servoValue > SERVO_VALUE_MAX)
	{
		return mapArray[MAP_ARRAY_SIZE - 1];
	}
	
	uint8_t i = (uint8_t)((servoValue - SERVO_VALUE_MIN)*STEP_RECIPROCAL + 0.5f);
	
	return mapArray[i];
}

float getRadius(float servoValue)
{
	float radiusRoot = getRadiusRoot(servoValue);
	
	return radiusRoot*radiusRoot;
}

float mapArray[MAP_ARRAY_SIZE] = {
		1.645865	,
		1.614463	,
		1.583798	,
		1.553862	,
		1.524644	,
		1.496134	,
		1.468322	,
		1.441198	,
		1.414753	,
		1.388976	,
		1.363858	,
		1.339388	,
		1.315557	,
		1.292355	,
		1.269771	,
		1.247797	,
		1.226421	,
		1.205635	,
		1.185427	,
		1.165789	,
		1.146710	,
		1.128181	,
		1.110191	,
		1.092730	,
		1.075789	,
		1.059358	,
		1.043426	,
		1.027984	,
		1.013022	,
		0.998530	,
		0.984499	,
		0.970917	,
		0.957775	,
		0.945064	,
		0.932773	,
		0.920892	,
		0.909412	,
		0.898323	,
		0.887614	,
		0.877276	,
		0.867299	,
		0.857673	,
		0.848387	,
		0.839433	,
		0.830800	,
		0.822478	,
		0.814457	,
		0.806727	,
		0.799279	,
		0.792103	,
		0.785188	,
		0.778525	,
		0.772103	,
		0.765913	,
		0.759946	,
		0.754190	,
		0.748636	,
		0.743274	,
		0.738094	,
		0.733087	,
		0.728242	,
		0.723549	,
		0.718999	,
		0.714582	,
		0.710287	,
		0.706105	,
		0.702025	,
		0.698039	,
		0.694135	,
		0.690305	,
		0.686537	,
		0.682823	,
		0.679152	,
		0.675515	,
		0.671900	,
		0.668300	,
		0.664702	,
		0.661099	,
		0.657479	,
		0.653833	,
		0.650151	,
		0.646422	,
		0.642638	,
		0.638788	,
		0.634862	,
		0.630850	,
		0.626743	,
		0.622530	,
		0.618201	,
		0.613747	,
		0.609158	,
		0.604424	,
		0.599534	,
		0.594479	,
		0.589249	,
		0.583834	,
		0.578224	,
		0.572409	,
		0.566379	,
		0.560125	,
		0.553636
};

