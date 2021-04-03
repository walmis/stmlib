#pragma once

typedef unsigned short ushort;
typedef unsigned int uint;


constexpr uint as_uint(const float x) {
    return *(uint*)&x;
}
constexpr float as_float(const uint x) {
    return *(float*)&x;
}

class HalfFloat {
public:
    HalfFloat() {
        storage = 0;
    }
    HalfFloat(float x) {
        *this = x;
    }

    template<typename T>
    HalfFloat(T x) {
        *this = (float)x;
    }

    template<typename T>
    operator T() {
        return (float)*this;
    }

    template<typename T>
    HalfFloat& operator =(T x) {
        *this = (float)x;
        return *this;
    }

    HalfFloat& operator =(float x) {
        const uint b = as_uint(x)+0x00001000; // round-to-nearest-even: add last bit after truncated mantissa
        const uint e = (b&0x7F800000)>>23; // exponent
        const uint m = b&0x007FFFFF; // mantissa; in line below: 0x007FF000 = 0x00800000-0x00001000 = decimal indicator flag - initial rounding
        storage = (b&0x80000000)>>16 | (e>112)*((((e-112)<<10)&0x7C00)|m>>13) | ((e<113)&(e>101))*((((0x007FF000+m)>>(125-e))+1)>>1) | (e>143)*0x7FFF; // sign : normalized : denormalized : saturate
        return *this;
    }

    operator float () { // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
        const uint e = (storage&0x7C00)>>10; // exponent
        const uint m = (storage&0x03FF)<<13; // mantissa
        const uint v = as_uint((float)m)>>23; // evil log2 bit hack to count leading zeros in denormalized format
        return as_float((storage&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000))); // sign : normalized : denormalized
    }

    template<typename T>
    float operator +(T x) {
        return (float)*this + (float)x;
    }
    template<typename T>
    float operator +=(T x) {
        float add = (float)*this + (float)x;
        *this = add;
        return add;
    }
    template<typename T>
    float operator -(T x) {
        return (float)*this - (float)x;
    }
    template<typename T>
    float operator *(T x) {
        return (float)*this * (float)x;
    }
    template<typename T>
    float operator *=(T x) {
        float mul = (float)*this * (float)x;
        *this = mul;
        return mul;
    }
    template<typename T>
    float operator /(T x) {
        return (float)*this / (float)x;
    }
    template<typename T>
    float operator /=(T x) {
        float div = (float)*this / (float)x;
        *this = div;
        return div;
    }
    ushort storage;
};

template<typename A>
float operator *(A a, HalfFloat b) {
    return (float)a * (float)b;
}
template<typename A>
float operator /(A a, HalfFloat b) {
    return (float)a / (float)b;
}


inline float sin(HalfFloat x) {
    return sin(float(x));
}
inline float cos(HalfFloat x) {
    return cos((float)x);
}
inline float sqrt(HalfFloat x) {
    return sqrt((float)x);
}
inline float atan2(HalfFloat x, HalfFloat y) {
    return atan2((float)x, (float)y);
}

