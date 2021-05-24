#pragma once
#include <algorithm>
#include <cstring>
#include <stdint.h>

#define _MEDIAN__SORT(a,b) do { if (a > b) { std::swap(a, b); } } while (0);

namespace filter
{
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
class Median
{
public:
    /**
     * \brief	Constructor
     *
     * \param	initialValue	Value will be set for the complete
     * 							input buffer.
     */
    Median(const T& initialValue = 0);

    /// Append new value
    T push(const T& input);

    /// Get median value
    const T getValue() const;
};



// ----------------------------------------------------------------------------


template <typename T>
class Median<T, 3>
{
public:
    Median(const T& initialValue = 0) {
        for (uint_fast8_t i = 0; i < 3; ++i) {
            buffer[i] = initialValue;
            sorted[i] = initialValue;
        }
    }

    T operator() (const T& input) {
        return push(input);
    }

    T push(const T& input) {
        buffer[index] = input;
        if (++index >= 3) {
            index = 0;
        }

        std::memcpy((void *) sorted, (const void * const) buffer, sizeof(sorted));

        _MEDIAN__SORT(sorted[0], sorted[1]);
        _MEDIAN__SORT(sorted[1], sorted[2]);
        _MEDIAN__SORT(sorted[0], sorted[1]);

        return getValue();
    }


    const T	getValue() const {
        return sorted[1];
    }

private:
    uint_fast8_t index;
    T buffer[3];
    T sorted[3];
};

}

