/**********************************************************************
 
 fft.cpp
 
 
 This class is a C++ wrapper for original code 
 written by Dominic Mazzoni in September 2000
 
 This file contains a few FFT routines, including a real-FFT
 routine that is almost twice as fast as a normal complex FFT,
 and a power spectrum routine which is more convenient when
 you know you don't care about phase information.  It now also
 contains a few basic windowing functions.
 
 Some of this code was based on a free implementation of an FFT
 by Don Cross, available on the web at:
 
 http://www.intersrv.com/~dcross/fft.html
 
 The basic algorithm for his code was based on Numerical Recipes
 in Fortran.  I optimized his code further by reducing array
 accesses, caching the bit reversal table, and eliminating
 float-to-double conversions, and I added the routines to
 calculate a real FFT and a real power spectrum.
 
 Note: all of these routines use single-precision floats.
 I have found that in practice, floats work well until you
 get above 8192 samples.  If you need to do a larger FFT,
 you need to use doubles.
 
 **********************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "fft.h"

static inline int IsPowerOfTwo(int x)
{
	if (x < 2)
		return false;
	
	if (x & (x - 1))             /* Thanks to 'byang' for this cute trick! */
		return false;
	
	return true;
}

static inline int NumberOfBitsNeeded(int PowerOfTwo)
{
	int i;
	
	if (PowerOfTwo < 2) {
		abort();
	}
	
	for (i = 0;; i++)
		if (PowerOfTwo & (1 << i))
			return i;
}

int ReverseBits(int index, int NumBits)
{
	int i, rev;
	
	for (i = rev = 0; i < NumBits; i++) {
		rev = (rev << 1) | (index & 1);
		index >>= 1;
	}
	
	return rev;
}


template<typename T>
void brswap(T *a, unsigned n) {
    for (unsigned i = 0, j = 0; i < n; i++) {
        if (i < j) {
            T tmp = a[i];
            a[i] = a[j];
            a[j] = tmp;
        }

        // Length of the mask.
        unsigned len = __builtin_ctz(i + 1) + 1;
        // XOR with mask.
        j ^= n - (n >> len);
    }
}

/*
 * Complex Fast Fourier Transform
 */


void FFT(int NumSamples,
         bool InverseTransform,
         FFTReal *RealIn, FFTReal *ImagIn, FFTReal *RealOut, FFTReal *ImagOut)
{
	int NumBits;                 /* Number of bits needed to store indices */
	int i, j, k, n;
	int BlockSize, BlockEnd;
	
	float angle_numerator = 2.0 * M_PI;
	float tr, ti;                /* temp real, temp imaginary */
	
	if (!IsPowerOfTwo(NumSamples)) {
		return;
	}

	if (InverseTransform)
		angle_numerator = -angle_numerator;
	
	NumBits = NumberOfBitsNeeded(NumSamples);
	
	/*
	 **   Do simultaneous data copy and bit-reversal ordering into outputs...
	 */

    if(RealOut) {
        memmove(RealOut, RealIn, NumSamples*sizeof(FFTReal));
    } else {
        RealOut = RealIn;
    }
    if(ImagOut) {
        memmove(ImagOut, ImagIn, NumSamples*sizeof(FFTReal));
    } else {
        ImagOut = ImagIn;
    }

	brswap(RealOut, NumSamples);
    brswap(ImagOut, NumSamples);

	// for (i = 0; i < NumSamples; i++) {
	// 	j = ReverseBits(i, NumBits);
    //     printf("rev a[%d] = a[%d]\n", j, i);
	// 	RealOut[j] = RealIn[i];
	// 	ImagOut[j] = (ImagIn == NULL) ? (StorageType)0.0 : ImagIn[i];
	// }
	
	/*
	 **   Do the FFT itself...
	 */
	
	BlockEnd = 1;
	for (BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1) {
		
		float delta_angle = angle_numerator / (float) BlockSize;
		
		float sm2 = sin(-2 * delta_angle);
		float sm1 = sin(-delta_angle);
		float cm2 = cos(-2 * delta_angle);
		float cm1 = cos(-delta_angle);


		float w = 2 * cm1;
		float ar0, ar1, ar2, ai0, ai1, ai2;
		
		for (i = 0; i < NumSamples; i += BlockSize) {
			ar2 = cm2;
			ar1 = cm1;
			
			ai2 = sm2;
			ai1 = sm1;
			
			for (j = i, n = 0; n < BlockEnd; j++, n++) {
				ar0 = w * ar1 - ar2;
				ar2 = ar1;
				ar1 = ar0;
				
				ai0 = w * ai1 - ai2;
				ai2 = ai1;
				ai1 = ai0;
				
				k = j + BlockEnd;
				tr = ar0 * RealOut[k] - ai0 * ImagOut[k];
				ti = ar0 * ImagOut[k] + ai0 * RealOut[k];
				
				RealOut[k] = RealOut[j] - tr;
				ImagOut[k] = ImagOut[j] - ti;
				
				RealOut[j] += tr;
				ImagOut[j] += ti;
			}
		}
		
		BlockEnd = BlockSize;
	}
	
	/*
	 **   Need to normalize if inverse transform...
	 */
	
	if (InverseTransform) {
		float denom = (float) NumSamples;
		
		for (i = 0; i < NumSamples; i++) {
			RealOut[i] /= denom;
			ImagOut[i] /= denom;
		}
	}
}

/*
 * Real Fast Fourier Transform
 *
 * This function was based on the code in Numerical Recipes in C.
 * In Num. Rec., the inner loop is based on a single 1-based array
 * of interleaved real and imaginary numbers.  Because we have two
 * separate zero-based arrays, our indices are quite different.
 * Here is the correspondence between Num. Rec. indices and our indices:
 *
 * i1  <->  real[i]
 * i2  <->  imag[i]
 * i3  <->  real[n/2-i]
 * i4  <->  imag[n/2-i]
 */

void RealFFT(int NumSamples, FFTReal *RealIn, FFTReal *RealOut, FFTReal *ImagOut)
{
	int Half = NumSamples / 2;
	int i;
	
	float theta = M_PI / Half;
	
	for (i = 0; i < Half; i++) {
		RealOut[i] = RealIn[2 * i];
		ImagOut[i] = RealIn[2 * i + 1];
	}
	
	FFT(Half, 0, RealOut, ImagOut, 0, 0);
	
	float wtemp = float (sin(0.5 * theta));
	
	float wpr = -2.0 * wtemp * wtemp;
	float wpi = float (sin(theta));
	float wr = 1.0 + wpr;
	float wi = wpi;
	
	int i3;
	
	float h1r, h1i, h2r, h2i;
	
	for (i = 1; i < Half / 2; i++) {
		
		i3 = Half - i;
		
		h1r = 0.5 * (RealOut[i] + RealOut[i3]);
		h1i = 0.5 * (ImagOut[i] - ImagOut[i3]);
		h2r = 0.5 * (ImagOut[i] + ImagOut[i3]);
		h2i = -0.5 * (RealOut[i] - RealOut[i3]);
		
		RealOut[i] = h1r + wr * h2r - wi * h2i;
		ImagOut[i] = h1i + wr * h2i + wi * h2r;
		RealOut[i3] = h1r - wr * h2r + wi * h2i;
		ImagOut[i3] = -h1i + wr * h2i + wi * h2r;
		
		wr = (wtemp = wr) * wpr - wi * wpi + wr;
		wi = wi * wpr + wtemp * wpi + wi;
	}
	
	RealOut[0] = (h1r = RealOut[0]) + ImagOut[0];
	ImagOut[0] = h1r - ImagOut[0];

}

/*
 * PowerSpectrum
 *
 * This function computes the same as RealFFT, above, but
 * adds the squares of the real and imaginary part of each
 * coefficient, extracting the power and throwing away the
 * phase.
 *
 * For speed, it does not call RealFFT, but duplicates some
 * of its code.
 */

void PowerSpectrum(int NumSamples, FFTReal *In, FFTReal *Out)
{
	int Half = NumSamples / 2;
	int i;
	
	float theta = M_PI / Half;
	
	FFTReal *tmpReal = new FFTReal[Half];
	FFTReal *tmpImag = new FFTReal[Half];
	FFTReal *RealOut = new FFTReal[Half];
	FFTReal *ImagOut = new FFTReal[Half];
	
	for (i = 0; i < Half; i++) {
		tmpReal[i] = In[2 * i];
		tmpImag[i] = In[2 * i + 1];
	}
	
	FFT(Half, 0, tmpReal, tmpImag, RealOut, ImagOut);
	
	float wtemp = float (sin(0.5 * theta));
	
	float wpr = -2.0 * wtemp * wtemp;
	float wpi = float (sin(theta));
	float wr = 1.0 + wpr;
	float wi = wpi;
	
	int i3;
	
	float h1r, h1i, h2r, h2i, rt, it;
	//float total=0;
	
	for (i = 1; i < Half / 2; i++) {
		
		i3 = Half - i;
		
		h1r = 0.5 * (RealOut[i] + RealOut[i3]);
		h1i = 0.5 * (ImagOut[i] - ImagOut[i3]);
		h2r = 0.5 * (ImagOut[i] + ImagOut[i3]);
		h2i = -0.5 * (RealOut[i] - RealOut[i3]);
		
		rt = h1r + wr * h2r - wi * h2i; //printf("Realout%i = %f",i,rt);total+=fabs(rt);
		it = h1i + wr * h2i + wi * h2r; // printf("  Imageout%i = %f\n",i,it);
		
		Out[i] = rt * rt + it * it;
		
		rt = h1r - wr * h2r + wi * h2i;
		it = -h1i + wr * h2i + wi * h2r;
		
		Out[i3] = rt * rt + it * it;
		
		wr = (wtemp = wr) * wpr - wi * wpi + wr;
		wi = wi * wpr + wtemp * wpi + wi;
	}
	//printf("total = %f\n",total);
	rt = (h1r = RealOut[0]) + ImagOut[0];
	it = h1r - ImagOut[0];
	Out[0] = rt * rt + it * it;
	
	rt = RealOut[Half / 2];
	it = ImagOut[Half / 2];
	Out[Half / 2] = rt * rt + it * it;
	
	delete[]tmpReal;
	delete[]tmpImag;
	delete[]RealOut;
	delete[]ImagOut;
}

/*
 * Windowing Functions
 */

int NumWindowFuncs()
{
	return 4;
}

char *WindowFuncName(int whichFunction)
{
	switch (whichFunction) {
		default:
		case 0:
			return "Rectangular";
		case 1:
			return "Bartlett";
		case 2:
			return "Hamming";
		case 3:
			return "Hanning";
	}
}

void WindowFunc(int whichFunction, int NumSamples, FFTReal *in)
{
	int i;
	
	if (whichFunction == 1) {
		// Bartlett (triangular) window
		for (i = 0; i < NumSamples / 2; i++) {
			in[i] *= (i / (float) (NumSamples / 2));
			in[i + (NumSamples / 2)] *=
			(1.0 - (i / (float) (NumSamples / 2)));
		}
	}
	
	if (whichFunction == 2) {
		// Hamming
		for (i = 0; i < NumSamples; i++)
			in[i] *= 0.54 - 0.46 * cos(2 * M_PI * i / (NumSamples - 1));
	}
	
	if (whichFunction == 3) {
		// Hanning
		for (i = 0; i < NumSamples; i++)
			in[i] *= 0.50 - 0.50 * cos(2 * M_PI * i / (NumSamples - 1));
	}
}


/* Calculate the power spectrum */
void powerSpectrum(int start, int half, FFTReal *data, int windowSize,FFTReal *magnitude,FFTReal *phase, FFTReal *power, FFTReal *avg_power) {
	int i;
	int windowFunc = 3;
	float total_power = 0.0f;
	
	/* processing variables*/
	FFTReal *in_real = new FFTReal[windowSize];
	FFTReal *in_img = new FFTReal[windowSize];
	FFTReal *out_real = new FFTReal[windowSize];
	FFTReal *out_img = new FFTReal[windowSize];
	
	for (i = 0; i < windowSize; i++) {
		in_real[i] = data[start + i];
	}
	
	WindowFunc(windowFunc, windowSize, in_real);
	RealFFT(windowSize, in_real, out_real, out_img);
	
	for (i = 0; i < half; i++) {
		/* compute power */
		power[i] = out_real[i]*out_real[i] + out_img[i]*out_img[i];
		total_power += power[i];
		/* compute magnitude and phase */
		magnitude[i] = 2.0*sqrt(power[i]);
		phase[i] = atan2(out_img[i],out_real[i]);
	}
	/* calculate average power */
	*(avg_power) = total_power / (float) half;
	
	delete[]in_real;
	delete[]in_img;   
	delete[]out_real;
	delete[]out_img;
}

void inversePowerSpectrum(int start, int half, int windowSize, FFTReal *finalOut,FFTReal *magnitude,FFTReal *phase) {
	int i;
	int windowFunc = 3;
	
	/* processing variables*/
	FFTReal *in_real = new FFTReal[windowSize];
	FFTReal *in_img = new FFTReal[windowSize];
	FFTReal *out_real = new FFTReal[windowSize];
	FFTReal *out_img = new FFTReal[windowSize];
	
	/* get real and imag part */
	for (i = 0; i < half; i++) {	
		in_real[i] = magnitude[i]*cos(phase[i]);
		in_img[i]  = magnitude[i]*sin(phase[i]);
	}
	
	/* zero negative frequencies */
	for (i = half; i < windowSize; i++) {	
		in_real[i] = 0.0;
		in_img[i] = 0.0;
	}
	
	FFT(windowSize, 1, in_real, in_img, out_real, out_img); // second parameter indicates inverse transform
	WindowFunc(windowFunc, windowSize, out_real);
	
	for (i = 0; i < windowSize; i++) {
		finalOut[start + i] += out_real[i];
	}
	
	delete[]in_real;
	delete[]in_img;   
	delete[]out_real;
	delete[]out_img;
}

float mag(float real, float im) {
    return sqrt(real*real + im*im);
}

double sincpi(double x)
/*
 * This atrociously slow function call should eventually be replaced
 * by a tabular approach. 
 */
{
    if (x == 0.0)
	return (1.0);
    return (sin(M_PI * x) / (M_PI * x));
}

int upsample(short *x, short *y, int xsamps, double up, double down)
/*
 * Convert low frequency data x, having xsamps 16-bit samples, to a
 * certain number ysamps of high-frequency y samples. The exact
 * number ysamps is the return value of this function, said value
 * determined by a formula in the declarations below. The high and
 * low frequencies are up, down respectively. 
 */
{
    double down_ratio = down / up;
    double a, b;
    int ysamps = 1 + (int)((xsamps - 6) / down_ratio);
    int i, j;

    for (j = 0; j < ysamps; j++) {
	b = j * down_ratio;
	i = (int)b;
	a = b - i;
	y[j] = 0.5 * (
		   x[i]   * sincpi(2+a) + x[i+1] * sincpi(1+a) +
		   x[i+2] * sincpi(a)   + x[i+3] * sincpi(1-a) +
		   x[i+4] * sincpi(2-a) + x[i+5] * sincpi(3-a));
    }
    return (ysamps);
}

#ifndef __arm__
#include <fstream>
#include <iostream>


constexpr float lerp(int xp, int x0, int x1, float y0, float y1) {
	return y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
}

int main() {
    HalfFloat a = 3.1415f;
    HalfFloat b = 3.1415f;
    a = a - 1.5*b;
    int size = 1024;
	int fft_size = 1024;
    FFTReal samples[4096];

    FFTReal real[size] = {};
    FFTReal imag[size] = {};

	std::ifstream f("cogmap");
	std::string str;
	int i = 0;
	while (std::getline(f, str)) {
		int x,y;
		sscanf(str.c_str(), "%d,0=%d\n", &x, &y);
		//printf("%d\n", y);
		samples[i] = y*0.0001 ;
		if(i % 4 == 0) {
			real[i/4] = samples[i];
		}
		i++;
	}

    // for(int i = 0; i < size; i++) {
    //     samples[i] = sin(2*i*(M_PI/255)) + sin(15*i*(M_PI/255)) + cos(65*i*(M_PI/255));
    //     real[i] = samples[i];
    // }

    for(int i = 0; i < size; i++) {
       // printf("%d,%f\n", i, (float)samples[i]);
    }

	int pole_pairs = 4;
	int stator_slots = 3*4;

    //RealFFT(256, samples, real, imag);
    FFT(size, 0, real, imag, 0, 0);
#if 1
	uint8_t harmonics[50] = {};

	harmonics[1] = 1;

	for(int i = 0; i < pole_pairs; i++) {
		// fprintf(stderr, "h: %d", (i+1)*stator_slots);
		harmonics[(i+1)*stator_slots] = 1;
	}

	harmonics[pole_pairs] = 1;
	for(int i = 0; i < stator_slots/4; i++) {
		// fprintf(stderr, "h: %d", (i+1)*stator_slots);
		harmonics[(i+1)*2*pole_pairs] = 1;
	}

	for(int i = 0; i < sizeof(harmonics); i++) {
		harmonics[i] = 1;
		if(!harmonics[i]) {
			real[i] = 0;
			imag[i] = 0;
			real[(size-i)%size] = 0;
			imag[(size-i)%size] = 0;
		} else {
			fprintf(stderr, "h: %d\n", i);
		}
	}

	for(int i = sizeof(harmonics); i <= size/2; i++) {
		real[i] = 0;
		imag[i] = 0;
		real[(size-i)%size] = 0;
		imag[(size-i)%size] = 0;
	}


	for(int i = 0; i < size/2; i++) {
		//real[(size-i-1)] = real[i];
		//imag[(size-i-1)] = imag[i];
	}
#endif
    FFTReal oreal[size] = {};
    FFTReal oimag[size] = {};

	// for(int i = 512; i < 2048; i++) {
	// 	real[i] = 0;
	// 	imag[i] = 0;
		
	// 	real[4096-i] = 0;
	// 	imag[4096-i] = 0;
	// }

	// for(int i = 0; i < 1024; i++) {
	// 	real[1024+i] = real[4096-1024+i]/2;
	// 	imag[1024+i] = imag[4096-1024+i]/2;
	// }

    FFT(size, 1, real, imag, 0, 0);
	int k = 4;
	float filtmap[4096]={};
	for (int i = 0; i < fft_size; i++) {
		float y2, y1, v;
		y1 = real[i];
		y2 = real[(i + 1)&(fft_size-1)];
		//std::cerr << i+1 << " " << k*i << std::endl;
		filtmap[k*i]   = lerp(0, 0, k, y1, y2);
		filtmap[k*i+1] = lerp(1, 0, k, y1, y2);
		filtmap[k*i+2] = lerp(2, 0, k, y1, y2);
		filtmap[k*i+3] = lerp(3, 0, k, y1, y2);
	}

    for(int i = 0; i < 4096; i++) {
        //printf("%d,%f,%f,%f\n", i, (float)samples[i], mag(real[i], imag[i]), oreal);
		float v = lerp(i&3, 0, 4, oreal[i/4], oreal[i/4+1]);
		//fprintf(stderr, "%d %d %f %f %f\n", i/4, i/4+1, (float)oreal[i/4], (float)oreal[i/4+4]);
        printf("%d,%f,%f,%f\n", i, (float)samples[i], (float)filtmap[i] /*mag(real[i], imag[i])*/, (float)real[i/4]);
    }

}

#endif
