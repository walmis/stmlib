#pragma once
#include <algorithm>
#include <cstring>
#include <stdint.h>

#define _MEDIAN__SORT(a,b) do { if (a > b) { std::swap(a, b); } } while (0);

//namespace filter
//{
/**
 * \brief	Median filter
 *
 * Calculates the median of a input set. Useful for eliminating spikes
 * from the input. Adds a group delay of N/2 ticks for the signal.
 *
 * Implementation are available for N = 3, 5, 7 and 9. To find
 * the median the signal values will be partly sorted, but only as much
 * as needed to find the median.
 *
 * \code
 * // create a new filter for five samples
 * xpcc::filter::Median<uint8_t, 5> filter;
 *
 * // append new signal values
 * filter.append(10);
 * filter.append(10);
 * filter.append(20);
 *
 * // calculate the median
 * filter.update();
 *
 * output = filter.getValue();
 * \endcode
 *
 * \tparam	T	Input type
 * \tparam	N	Number of samples
 *
 * \ingroup	filter
 */


template<typename T, int N>
struct median_sort {
  static void sort(T sorted[]);
};

template<typename T>
struct median_sort<T,3> {
  static void sort(T sorted[]) {
    _MEDIAN__SORT(sorted[0], sorted[1]);
    _MEDIAN__SORT(sorted[1], sorted[2]);
    _MEDIAN__SORT(sorted[0], sorted[1]);
  }
};
template<typename T>
struct median_sort<T,5> {
  static void sort(T sorted[]) {
    _MEDIAN__SORT(sorted[0], sorted[1]);
    _MEDIAN__SORT(sorted[3], sorted[4]);
    _MEDIAN__SORT(sorted[0], sorted[3]);
    _MEDIAN__SORT(sorted[1], sorted[4]);
    _MEDIAN__SORT(sorted[1], sorted[2]);
    _MEDIAN__SORT(sorted[2], sorted[3]);
    _MEDIAN__SORT(sorted[1], sorted[2]);
  }
};
template<typename T>
struct median_sort<T,7> {
  static void sort(T sorted[]) {
    _MEDIAN__SORT(sorted[0], sorted[5]);
    _MEDIAN__SORT(sorted[0], sorted[3]);
    _MEDIAN__SORT(sorted[1], sorted[6]);
    _MEDIAN__SORT(sorted[2], sorted[4]);
    _MEDIAN__SORT(sorted[0], sorted[1]);
    _MEDIAN__SORT(sorted[3], sorted[5]);
    _MEDIAN__SORT(sorted[2], sorted[6]);
    _MEDIAN__SORT(sorted[2], sorted[3]);
    _MEDIAN__SORT(sorted[3], sorted[6]);
    _MEDIAN__SORT(sorted[4], sorted[5]);
    _MEDIAN__SORT(sorted[1], sorted[4]);
    _MEDIAN__SORT(sorted[1], sorted[3]);
    _MEDIAN__SORT(sorted[3], sorted[4]);
  }
};
template<typename T>
struct median_sort<T,9> {
  static void sort(T sorted[]) {
    _MEDIAN__SORT(sorted[1], sorted[2]);
    _MEDIAN__SORT(sorted[4], sorted[5]);
    _MEDIAN__SORT(sorted[7], sorted[8]);
    _MEDIAN__SORT(sorted[0], sorted[1]);
    _MEDIAN__SORT(sorted[3], sorted[4]);
    _MEDIAN__SORT(sorted[6], sorted[7]);
    _MEDIAN__SORT(sorted[1], sorted[2]);
    _MEDIAN__SORT(sorted[4], sorted[5]);
    _MEDIAN__SORT(sorted[7], sorted[8]);
    _MEDIAN__SORT(sorted[0], sorted[3]);
    _MEDIAN__SORT(sorted[5], sorted[8]);
    _MEDIAN__SORT(sorted[4], sorted[7]);
    _MEDIAN__SORT(sorted[3], sorted[6]);
    _MEDIAN__SORT(sorted[1], sorted[4]);
    _MEDIAN__SORT(sorted[2], sorted[5]);
    _MEDIAN__SORT(sorted[4], sorted[7]);
    _MEDIAN__SORT(sorted[4], sorted[2]);
    _MEDIAN__SORT(sorted[6], sorted[4]);
    _MEDIAN__SORT(sorted[4], sorted[2]);
  }
};


// ----------------------------------------------------------------------------


template <typename T, int N>
class Median
{
public:
    Median(const T& initialValue = 0) {
      reset(initialValue);
    }

    void reset(const T& initialValue = 0) {
      index = 0;
      for (uint_fast8_t i = 0; i < N; ++i) {
          buffer[i] = initialValue;
          sorted[i] = initialValue;
      }
    }

    T operator() (const T& input) {
        return push(input);
    }

    T push(const T& input) {
        buffer[index] = input;
        if (++index >= N) {
            index = 0;
        }

        std::memcpy((void *) sorted, (const void * const) buffer, sizeof(sorted));
        median_sort<T,N>::sort(sorted);

        return getValue();
    }


    const T	getValue() const {
        return sorted[N/2];
    }

private:
    uint_fast8_t index;
    T buffer[N];
    T sorted[N];
};

//}
