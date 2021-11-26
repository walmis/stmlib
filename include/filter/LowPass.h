/*
 * LowPass.h
 *
 *  Created on: Jan 26, 2020
 *      Author: walmis
 */

#ifndef FILTER_LOWPASS_H_
#define FILTER_LOWPASS_H_

#include <math.h>

template<int scale>
class ILowPassFilter {
public:
  constexpr ILowPassFilter(float fcut, float dt = 0.001) :
    value(0), inv_alpha(1.0f / (dt / (dt + 1.0f / (2.0f * M_PI * fcut))))
  {}

  constexpr void set_cutoff(float fcut, float dt = 0.001) {
    inv_alpha = 1.0f / (dt / (dt + 1.0f / (2.0f * M_PI * fcut)));
  }

  constexpr void update(int sample) {
    value += (sample*scale - value) / inv_alpha;
  }

  constexpr int val() {
    return value/scale;
  }

  constexpr void reset(int new_value) {
    value = new_value*scale;
  }

  int value;
  int inv_alpha;
};

template<typename T>
class LowPassFilter {
public:
	constexpr LowPassFilter(T fcut, T dt = 0.001) :
		value(0), alpha(dt / T(dt + T(1.0f) / (T(2.0f * M_PI) * fcut)))
	{}

	constexpr void set_cutoff(T fcut, T dt = 0.001) {
		alpha = dt / (dt + T(1.0f) / (2.0f * M_PI * fcut));
	}

	T operator()(T x) {
	  return update(x);
	}

	constexpr T update(T sample) {
		value += alpha * (sample - value);
		return value;
	}

	constexpr T val() {
	  return value;
	}

	constexpr void reset(T new_value) {
		value = new_value;
	}

	T value;
	T alpha;
};


#endif /* FILTER_LOWPASS_H_ */
