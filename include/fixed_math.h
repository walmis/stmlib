#pragma once

#include "fixed_point.h"
#include <cassert>
//#define assert(...)
namespace detail
{

// Returns the index of the most-signifcant set bit
inline long find_highest_bit(unsigned long value) noexcept
{
    //assert(value != 0);
#if defined(_MSC_VER)
    unsigned long index;
    _BitScanReverse(&index, value);
    return index;
#elif defined(__GNUC__) || defined(__clang__)
    return sizeof(value) * 8 - 1 - __builtin_clzl(value);
#else
#   error "your platform does not support find_highest_bit()"
#endif
}

}

template <int F>
fp<F> exp2(fp<F> x) noexcept
{
    using Fixed = fp<F>;
    if (x < Fixed(0)) {
        return Fixed(1) / exp2(-x);
    }

    const decltype(x.val) x_int = (int)x;
    x -= x_int;
    assert(x >= Fixed(0) && x < Fixed(1));

    constexpr auto fA = fix31(1.8964611454333148e-3); //Fixed::template from_fixed_point<63>(  17491766697771214ll); // 1.8964611454333148e-3
    constexpr auto fB = fix31(8.9428289841091295e-3); //;Fixed::template from_fixed_point<63>(  82483038782406547ll); // 8.9428289841091295e-3
    constexpr auto fC = fix31(5.5866246304520701e-2); //Fixed::template from_fixed_point<63>( 515275173969157690ll); // 5.5866246304520701e-2
    constexpr auto fD = fix31(2.4013971109076949e-1); //Fixed::template from_fixed_point<63>(2214897896212987987ll); // 2.4013971109076949e-1
    constexpr auto fE = fix31(6.9315475247516736e-1); //Fixed::template from_fixed_point<63>(6393224161192452326ll); // 6.9315475247516736e-1
    constexpr auto fF = fix31(9.9999989311082668e-1); //Fixed::template from_fixed_point<63>(9223371050976163566ll); // 9.9999989311082668e-1
    return Fixed(Fixed(1 << x_int) * Fixed(((((fA * x + fB) * x + fC) * x + fD) * x + fE) * x + fF));
}


//
// Power functions
//

template <int F, typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
fp<F> pow(fp<F> base, T exp) noexcept
{
    using Fixed = fp<F>;
    constexpr auto FRAC = uint32_t(1) << F;

    if (base == Fixed(0)) {
        assert(exp > 0);
        return Fixed(0);
    }

    Fixed result {1};
    if (exp < 0)
    {
        for (Fixed intermediate = base; exp != 0; exp /= 2, intermediate *= intermediate)
        {
            if ((exp % 2) != 0)
            {
                result /= intermediate;
            }
        }
    }
    else
    {
        for (Fixed intermediate = base; exp != 0; exp /= 2, intermediate *= intermediate)
        {
            if ((exp % 2) != 0)
            {
                result *= intermediate;
            }
        }
    }
    return result;
}

template <int F>
fp<F> log2(fp<F> x) noexcept
{
    using Fixed = fp<F>;
    assert(x > Fixed(0));

    // Normalize input to the [1:2] domain
    auto value = x.val;
    const long highest = detail::find_highest_bit(value);
    if (highest >= F) {
        value >>= (highest - F);
    } else {
        value <<= (F - highest);
    }
    x = Fixed::fromRaw(value);
    assert(x >= Fixed(1) && x < Fixed(2));

    constexpr auto fA = fix31(4.4873610194131727e-2);
    constexpr auto fB = fix31(-4.1656368651734915e-1); //Fixed::template from_fixed_point<63>(-3842121857793256941ll); // -4.1656368651734915e-1
    constexpr auto fC = fix29(1.6311487636297217); //Fixed::template from_fixed_point<62>( 7522345947206307744ll); //  1.6311487636297217
    constexpr auto fD = fix29(-3.5507929249026341); //Fixed::template from_fixed_point<61>(-8187571043052183818ll); // -3.5507929249026341
    constexpr auto fE = fix28(5.0917108110420042); //Fixed::template from_fixed_point<60>( 5870342889289496598ll); //  5.0917108110420042
    constexpr auto fF = fix28(-2.8003640347009253); //Fixed::template from_fixed_point<61>(-6457199832668582866ll); // -2.8003640347009253
   
    return Fixed(highest - F) + Fixed(((((fA * x + fB) * x + fC) * x + fD) * x + fE) * x + fF);
}

template <int F>
fp<F> pow(fp<F> base, fp<F> exp) noexcept
{
    using Fixed = fp<F>;

    if (base == Fixed(0)) {
        assert(exp > Fixed(0));
        return Fixed(0);
    }

    if (exp < Fixed(0))
    {
        return 1 / pow(base, -exp);
    }

    constexpr auto FRAC = uint32_t(1) << F;
    if (exp.val % FRAC == 0)
    {
        // Non-fractional exponents are easier to calculate
        return pow(base, exp.val / FRAC);
    }

    // For negative bases we do not support fractional exponents.
    // Technically fractions with odd denominators could work,
    // but that's too much work to figure out.
    assert(base > Fixed(0));
    return exp2(Fixed(log2(base) * exp));
}

template <int F>
fp<F> log(fp<F> x) noexcept
{
    return log2(x) / log2(fp<F>(M_E));
}


template <int F>
fp<F> sqrt(fp<F> x) noexcept
{
    using Fixed = fp<F>;

    assert(x >= Fixed(0));
    if (x == Fixed(0))
    {
        return x;
    }
    using I = int64_t;
    // Finding the square root of an integer in base-2, from:
    // https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29

    // Shift by F first because it's fixed-point.
    I num = I{x.val} << F;
    I res = 0;

    // "bit" starts at the greatest power of four that's less than the argument.
    for (I bit = I{1} << ((detail::find_highest_bit(x.val) + F) / 2 * 2); bit != 0; bit >>= 2)
    {
        const I val = res + bit;
        res >>= 1;
        if (num >= val)
        {
            num -= val;
            res += bit;
        }
    }

    // Round the last digit up if necessary
    if (num > res)
    {
        res++;
    }

    return Fixed::fromRaw(static_cast<decltype(x.val)>(res));
}

/* a single instruction in many 32-bit architectures */
inline uint32_t umul32hi (uint32_t a, uint32_t b)
{
    return (uint32_t)(((uint64_t)a * b) >> 32);
}

/* a single instruction in many 32-bit architectures */
inline int32_t mul32hi (int32_t a, int32_t b)
{
    return (int32_t)(uint32_t)((uint64_t)((int64_t)a * b) >> 32);
}
#if 0
/*
  compute sine and cosine of argument in [0, PI/2]
  input and output in S8.23 format
  max err sine = 9.86237533e-8  max err cosine = 1.02729891e-7
  rms err sine = 4.11141973e-8  rms err cosine = 4.11752018e-8
  sin correctly rounded: 10961278 (83.19%)  
  cos correctly rounded: 11070113 (84.01%)
*/
void sincos_fixed_nj (int32_t x, int32_t *sine, int32_t *cosine)
{
    // minimax polynomial approximation for sine on [0, PI/4]
    const uint32_t s0 = (uint32_t)(1.9510998390614986e-4 * (1LL << 32) + 0.5);
    const uint32_t s1 = (uint32_t)(8.3322080317884684e-3 * (1LL << 32) + 0.5);
    const uint32_t s2 = (uint32_t)(1.6666648373939097e-1 * (1LL << 32) + 0.5);
    const uint32_t s3 = (uint32_t)(9.9999991734512150e-1 * (1LL << 32) + 0.5);
    // minimax polynomial approximation for cosine on [0, PI/4]
    const uint32_t c0 = (uint32_t)(1.3578890357166529e-3 * (1LL << 32) + 0.5);
    const uint32_t c1 = (uint32_t)(4.1654359549283981e-2 * (1LL << 32) + 0.5);
    const uint32_t c2 = (uint32_t)(4.9999838648363948e-1 * (1LL << 32) + 0.5);
    const uint32_t c3 = (uint32_t)(9.9999997159466147e-1 * (1LL << 32) + 0.5);
    // auxilliary constants
    const int32_t hpi_p23 = (int32_t)(3.141592653590 / 2 * (1LL << 23) + 0.5);
    const int32_t qpi_p23 = (int32_t)(3.141592653590 / 4 * (1LL << 23) + 0.5);
    const int32_t one_p23 = (int32_t)(1.0000000000000e+0 * (1LL << 23) + 0.5);
    uint32_t a, s, q, h, l, t, sn, cs;

    /* reduce range from [0, PI/2] to [0, PI/4] */
    t = (x > qpi_p23) ? (hpi_p23 - x) : x; // S8.23

    /* scale up argument for maximum precision in intermediate computation */
    a = t << 9; // U0.32

    /* pre-compute a**2 and a**4 */
    s = umul32hi (a, a); // U0.32
    q = umul32hi (s, s); // U0.32

    /* approximate sine on [0, PI/4] */
    h = s3 - umul32hi (s2, s); // U0.32
    l = umul32hi (s1 - umul32hi (s0, s), q); // U0.32
    sn = umul32hi (h + l, a); // U0.32

    /* approximate cosine on [0, PI/4] */
    h = c3 - umul32hi (c2, s); // U0.32
    l = umul32hi (c1 - umul32hi (c0, s), q); // U0.32
    cs = h + l; // U0.32

    /* round results to target precision */
    sn = ((sn + 256) >> 9); // S8.23
    cs = ((cs + 256) >> 9); // S8.23

    /* cosine result overflows U0.32 format for small arguments */
    cs = (t < 0xb50) ? one_p23 : cs; // S8.23

    /* map sine/cosine approximations based on quadrant */
    *sine   = (t != x) ? cs : sn; // S8.23
    *cosine = (t != x) ? sn : cs; // S8.23
}   
#endif
/*
  compute sine and cosine of argument in [0, PI/2]
  input and output in S8.23 format
  max err sine = 1.13173883e-7  max err cosine = 1.13158773e-7
  rms err sine = 4.30955921e-8  rms err cosine = 4.31472191e-8
  sin correctly rounded: 10844170 (82.30%)  
  cos correctly rounded: 10855609 (82.38%)

  Based on an approach by OllyW (http://www.olliw.eu/2014/fast-functions/, 
  retrieved 10/23/2020). We transform a = 2/PI*x-1/2, then we approximate
  sin ((2*PI*a + PI)/4 and cos ((2*PI*a + PI)/4. Except for sign flipping
  in the odd-power terms of the expansions the two series expansions match:

https://www.wolframalpha.com/input/?i=series++sin+%28%282*pi*a+%2B+pi%29%2F4%29
https://www.wolframalpha.com/input/?i=series++cos+%28%282*pi*a+%2B+pi%29%2F4%29

  This means we can sum the odd-power and the even-power terms seperately,
  then compute the sum and difference of those sums giving sine and cosine.
*/
inline void sincos_kernel (ufix31 x, fix29 *sine, fix29 *cosine)
{
    // minimax polynomial approximation for sin ((2*PI*a + PI)/4 on [-0.5, 0.5]
    const uint32_t c0 = (uint32_t)(7.0710676768794656e-1 * (1LL << 32) + 0.5);
    const uint32_t c1 = (uint32_t)((1.110721191857 -.25) * (1LL << 32) + 0.5);
    const uint32_t c2 = (uint32_t)(8.7235601339489222e-1 * (1LL << 32) + 0.5);
    const uint32_t c3 = (uint32_t)(4.5677902549505234e-1 * (1LL << 32) + 0.5);
    const uint32_t c4 = (uint32_t)(1.7932640877552330e-1 * (1LL << 32) + 0.5);
    const uint32_t c5 = (uint32_t)(5.6449491763487458e-2 * (1LL << 32) + 0.5);
    const uint32_t c6 = (uint32_t)(1.4444266213104129e-2 * (1LL << 32) + 0.5);
    const uint32_t c7 = (uint32_t)(3.4931597765535116e-3 * (1LL << 32) + 0.5);
    // auxiliary constants
    //const uint32_t twoopi = (uint32_t)(2/M_PI * (1LL << 32) + 0.5);
    const uint32_t half_p31 = (uint32_t)(0.5000000000000 * (1LL << 31) + 0.5);
    const uint32_t quarter_p30 = (uint32_t)(0.2500000000 * (1LL << 30) + 0.5);
    uint32_t s, q, h, l;
    int32_t a, o, e, sn, cs;


    /* a = (2/PI)*x - 0.5 */
    //a = umul32hi (twoopi, t) - half_p31; // S0.31
    a = x.val - half_p31;

    /* precompute a**2 and a**4 */
    s = (uint32_t)mul32hi (a, a) << 2; // U0.32
    q = umul32hi (s, s); // U0.32

    /* sum odd power terms; add in second portion of c1 (= 0.25) at the end */
    h = c1 - umul32hi (c3, s); // U0.32
    l = umul32hi ((c5 - umul32hi (c7, s)), q); // U0.32
    o = ((h + l) >> 2) + quarter_p30; // S1.30
    o = mul32hi (o, a); // S2.29

    /* sum even power terms */
    h = c0 - umul32hi (c2, s); // U0.32
    l = umul32hi ((c4 - umul32hi (c6, s)), q); // U0.32
    e = (h + l) >> 3; // S2.29 

    /* compute sine and cosine as sum and difference of odd / even terms */
    sn = e + o; // S2.29 sum -> sine 
    cs = e - o; // S2.29 difference -> cosine

    sine->val = sn;
    cosine->val = cs;
}


inline void sincos_kernelf(float a, float* s, float *c) {
   const float a_0 = 0.707106781187;
   const float a_2 = -0.872348075361;
   const float a_4 = 0.179251759526;
   const float a_6 = -0.0142718282624;
   const float b_1 = -1.11067032264;
   const float b_3 = 0.4561589075945;
   const float b_5 = -0.0539104694791;

   a -= 0.5f;

   float A = a_0 + a*a*(a_2 + a*a*(a_4 + a*a*(a_6)));
   float B = a*(b_1 + a*a*(b_3 + a*a*(b_5)));

   *s = A - B;
   *c = A + B;
}

inline void sincos(float angle, float *sin, float *cos) {
  float s, c;

  angle *= float(2 / M_PI);
  int q = (int) angle;

  angle = (angle - q);
  if (angle < 0) {
    q -= 1;
    angle += 1;
  }

  sincos_kernelf(angle, &s, &c);

  switch (q & 3) {
  case 0:
    *sin = s;
    *cos = c;
    break;
  case 1:
    *sin = c;
    *cos = -s;
    break;
  case 2:
    *sin = -s;
    *cos = -c;
    break;
  default:
    *sin = -c;
    *cos = s;
    break;
  }
}

template<int B>
void sincos (fp<B> angle, fp<29> *sin, fp<29> *cos) {
    const uint32_t twoopi = (uint32_t)(2/M_PI * (1LL << 32) + 0.5f);
    fix29 s, c;

    int64_t a = (int64_t)angle.val*twoopi; 
    int q = a>>(32+B); // q encodes quadrant
    uint32_t tmp = uint32_t(a>>B)>>1; //make u1.30

    sincos_kernel(ufix31::fromRaw(tmp), &s, &c);

   switch( q & 3){
        case 0: *sin = s; *cos = c; break;
        case 1: *sin = c; *cos = -s; break;
        case 2: *sin = -s; *cos = -c; break;
        default: *sin = -c; *cos = s; break;
  }
}


template<typename T = fp29_t>
struct SinCos {
  template<typename A>
  SinCos(A angle) {
    T _s, _c;
    sincos(angle, &_s, &_c);
    s = (T)_s;
    c = (T)_c;
  }
  T s;
  T c;
};
