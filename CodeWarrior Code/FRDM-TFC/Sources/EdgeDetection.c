/*
 * EdgeDetection.c
 *
 *  Created on: Dec 17, 2014
 *  Author: Miroslav Dobrev
 *  
 *  Implements 1D simplified Canny edge detector, for edge detection in images.
 */

#include <math.h>

#define KERNEL_LENGTH	9			// for SIGMA = 2.0, nominally LENGTH should be 9
#define SIGMA			2.0f		// Gaussian blur SIGMA value

#define PI				3.14159f
#define E				2.71828f
uint16_t Kernel[KERNEL_LENGTH] = {0};
#define SIDE_BAND_LENGTH	(KERNEL_LENGTH - 1)/2

#define FIXED_POINT_RESCALE		8192	// 2^13 used for faster division

static uint32_t blurredImage[128];
static int16_t derivativeImage[128];

//////////////////////////////////////////////////////////////////////////
//// Generates Gaussian Kernel
//	 KERNEL_LENGTH - number of kernel elements including negative and positive areas
//	 SIGMA		   - standard deviation
//////////////////////////////////////////////////////////////////////////
void generateKernel()
{
	unsigned char i;
	
	if (KERNEL_LENGTH % 2 != 1 || KERNEL_LENGTH < 3)		// KERNEL_LENGTH should be odd => 3
		return;
	
	for(i = 0; i <= SIDE_BAND_LENGTH; i++)
		Kernel[i + SIDE_BAND_LENGTH] = FIXED_POINT_RESCALE*(0.39894f*(1.0f/SIGMA)*pow(E,(-pow((i),2.0f))/(2.0f*pow(SIGMA,2.0f))));
	
	for(i = 0; i < SIDE_BAND_LENGTH; i++)
		Kernel[i] = Kernel[KERNEL_LENGTH - 1 - i];
}


//////////////////////////////////////////////////////////////////////////
//// Blurs image array using Gaussian kernel
//	 First and last SIDE_BAND_LENGTH pixels are corrupted due to
//	 end effects of the convolution used for the bluring
//////////////////////////////////////////////////////////////////////////
void blurImage(volatile uint16_t* Image, const uint32_t ImageLen, uint32_t* Result)
{
	uint32_t n;
	register uint32_t kmin, kmax, k;
	register uint32_t offset = 0;
	  
	if (KERNEL_LENGTH % 2 != 1 || KERNEL_LENGTH < 3)		// KERNEL_LENGTH should be odd => 3
		return;
	
	for (n = 0 ; n <= ImageLen - 1 ; n++)
    {
		Result[n] = 0;
	
		kmin = (n >= SIDE_BAND_LENGTH) ? (n - SIDE_BAND_LENGTH) : 0;
		kmax = (n <= ImageLen - 1 - SIDE_BAND_LENGTH) ? (n + SIDE_BAND_LENGTH) : (ImageLen - 1);	
		offset = (n <= ImageLen - 1 - SIDE_BAND_LENGTH) ? (KERNEL_LENGTH - (kmax - kmin + 1)) : 0;
	
		for (k = kmin; k <= kmax; k++)
		{
			Result[n] += Image[k] * Kernel[k - kmin + offset];
		}
		
		Result[n] = (Result[n]/FIXED_POINT_RESCALE) & 0x0000FFFF;		// Fixed point rescaling
    }
}
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//// Applies a Sobel operator to the image to get the gradient of the image
//	 The first and last pixels of the image do not provide a derivative value
//////////////////////////////////////////////////////////////////////////
void sobelDerivative(const uint32_t* Image, const uint32_t ImageLen, int16_t* Result)
{
	register uint32_t i;
	
	if (ImageLen < 3)	// The image should be at least 3 pixels long
		return;
	
	for (i = 1; i <= ImageLen - 2; i++)
	{
		Result[i] = Image[i+1] - Image[i-1];
	}
	
	Result[0] = 0;
	Result[ImageLen - 1] = 0;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//// Non-Maximal Supression
//	 Leaves just the local maximums (centre of the edges)
//	 The scan starts from 3rd pixel since 1st = 0 and 2nd needed for calculation
//////////////////////////////////////////////////////////////////////////
void findLocalMax(const int16_t* derivativeArray, const uint32_t arrayLen, int16_t* Result)
{
	register uint32_t i;
	
	for (i = 2; i <= arrayLen - 3; i++)
	{   
		if ((derivativeArray[i] >= derivativeArray[i-1] && derivativeArray[i] >= derivativeArray[i+1]) || (derivativeArray[i] <= derivativeArray[i-1] && derivativeArray[i] <= derivativeArray[i+1]))
		{
			Result[i] = derivativeArray[i];
		}
		else
		{
			Result[i] = 0;
		}
	}
	
	Result[0] = 0;
	Result[1] = 0;
	Result[arrayLen-1] = 0;
	Result[arrayLen-2] = 0;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//// Main function for the edge detection
//	 input - captured image from camera
//	 output - array with detected edges
//	 length - length of the arrays
//////////////////////////////////////////////////////////////////////////
void edgeDetection(volatile uint16_t* input, int16_t* output, const uint32_t length)
{		
	blurImage(input, length, blurredImage);
	sobelDerivative(blurredImage, length, derivativeImage);
	findLocalMax(derivativeImage, length, output);
}
//////////////////////////////////////////////////////////////////////////

