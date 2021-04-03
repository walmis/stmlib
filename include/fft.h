/*
 * fft.h
 *
 *  Created on: Jan 6, 2021
 *      Author: walmis
 */

#ifndef FFT_H_
#define FFT_H_

#include "half_float.h"

typedef HalfFloat FFTReal;

void FFT(int NumSamples,
         bool InverseTransform,
         FFTReal *RealIn, FFTReal *ImagIn, FFTReal *RealOut, FFTReal *ImagOut);


#endif /* FFT_H_ */
