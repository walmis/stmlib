#pragma once
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <type_traits>

#define clz(x) (__builtin_clzl(x) - (8 * sizeof(long) - 32))
#define is_integer(T) typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr
#define is_float(T) typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr
#define is_number(T) typename std::enable_if<std::is_floating_point<T>::value||std::is_integral<T>::value, T>::type* = nullptr

#ifdef __arm__
extern "C" int32_t fix32_div(int32_t a, int32_t b, int32_t exp);
#endif

template<int A, int B>
struct max_of {
    enum { value = A > B ? A : B};
};
template<int A, int B>
struct min_of {
    enum { value = A < B ? A : B};
};

template<typename T>
constexpr T fastround(T in) {
  (in>0) ? in += 0.5f : in-=0.5f;
  return (int)in;
}

template<int B, typename BASE = int32_t>
class fp;

template<int B>
struct fpres {
  static constexpr uint32_t BITS = B;
  fpres(int64_t val) : val(val) {}

  template<typename T, is_float(T)>
  explicit constexpr fpres(T b) {
    val = fastround(b * (1LL << BITS));
  }

  template<int BB>
  explicit fpres(fp<BB> b);

  explicit operator int() {
    return val >> BITS;
  }

  template<int BB>
  operator fp<BB>() {
    return (fp<BB>)*this;
  }

  constexpr auto operator+(fpres b) {
    return fpres(val + b.val);
  }
  constexpr auto operator-(fpres b) {
    return fpres(val - b.val);
  }
  template<typename T, is_integer(T)>
  constexpr auto operator/(T b) {
    return fpres(val / b);
  }
  int64_t val;
};

template<int B, typename BASE>
class fp {
public:
  static constexpr uint32_t N = 1 << B;
  static constexpr uint32_t BITS = B;

  constexpr fp() :
      val(0) {
  }

  template<typename T, is_integer(T)>
  constexpr fp(const T x) :
      val(x * N) {
      //printf("newi %d %d\n",B,  x);
  }

  template<typename T, is_float(T)>
  constexpr fp(T x) : val(fastround(x*N))
  {
    //printf("newf %d %f\n", B, x);
    //T tmp = x * N;
    //tmp += (x > 0) ? 0.5f : -0.5f;
    //val = tmp;
  }

  template<int BB>
  constexpr fp(fpres<BB>& fp)
  {
    //printf("convert %d > %d %lld\n", BB, B, fp.val>> (BB-B));
    val = fp.val / (1LL<<(BB-B));
  }

  constexpr fp(const fp &other) = default;

  template<int C>
  constexpr fp(fp<C> other) {
    //printf("new %d>%d\n", C,B);
    if (C > B) {
      val = BASE(other.val) >> (C - B);
    } else {
      val = BASE(other.val) << (B - C);
    }
  }

  static constexpr fp fromRaw(int raw) {
    fp v;
    v.val = raw;
    return v;
  }

  static constexpr fp fromInt(int b) {
    return fp(b);
  }

  template<int BB>
  constexpr fpres<BB+B> operator *(const fp<BB> b) const {
    //printf("mul %d %d\n", B, BB);
    return (int64_t)val * b.val;
  }

  template<typename T, is_float(T)>
  constexpr fp operator *(const T& b) const {
    //printf("mulfp\n");
    return *this * fp(b);
  }

  template<typename T, is_integer(T)>
  constexpr fp operator *(const T& b) const {
    return fromRaw(val * b);
  }

  template<typename T, is_integer(T)>
  constexpr fp operator /(const T& b) const {
    return fromRaw(val / b);
  }

  constexpr fp operator /(const fp div) const {
  #ifdef __arm__
    return fromRaw(fix32_div(val, div.val, B));
  #else
    return fromRaw(((int64_t) val * N) / div.val);
  #endif
  }

  constexpr fp& operator*=(const fp& b) {
    val = fp(*this * b).val;
    return *this;
  }
  constexpr fp& operator/=(const fp& b) {
    val = fp(*this / b).val;
    return *this;
  }
  constexpr fp& operator%=(const fp& b) {

    val = val % b.val;
    return *this;
  }
  constexpr fp& operator+=(const fp& b) {
    val += b.val;
    return *this;
  }
  constexpr fp& operator-=(const fp& b) {
    val -= b.val;
    return *this;
  }

  template<int A>
  constexpr fp& operator=(fpres<A> r) {
    *this = fp(r);
    return *this;
  }

  constexpr fp operator-() const {
    return fromRaw(-val);
  }
  constexpr fp operator-(fp b) const {
    return fromRaw(val - b.val);
  }
  constexpr fp operator+(fp b) const {
    return fromRaw(val + b.val);
  }
  constexpr bool operator==(fp b) const {
    return val == b.val;
  }
  constexpr bool operator<=(fp b) const {
    return val <= b.val;
  }
  constexpr bool operator>=(fp b) const {
    return val >= b.val;
  }
  constexpr bool operator<(fp b) const {
    return val < b.val;
  }
  constexpr bool operator>(fp b) const {
    return val > b.val;
  }
  constexpr fp operator %(fp b) const {
    return fromRaw(val % b.val);
  }

  constexpr int toInt() const {
    return (val /*+ N/2*/) / N;
  }
  constexpr float toFloat() const {
    return val / float(N);
  }

  explicit constexpr operator float() const {
    return val / float(N);
  }

  explicit constexpr operator int() const {
    return toInt();
  }

  BASE val;
};


template<int A>
template<int B>
fpres<A>::fpres(fp<B> b) {
  val = (int64_t)b.val * (1LL<<(A-B));
}

template<typename T, int F>
constexpr bool operator>(T a, fp<F> b) {
  return fp<F>(a) > b;
}
template<typename T, int F>
constexpr bool operator<(T a, fp<F> b) {
  return fp<F>(a) < b;
}

template<int B, int F>
constexpr fpres<32> operator*(fpres<F> a, fp<B> b) {
  return (a.val/(1LL<<((F-32)+B))) * b.val;
}
template<int B, int F>
constexpr fpres<32> operator*(fp<B> a, fpres<F> b) {
  return b * a;
}

template<typename T, int F, is_integer(T)>
constexpr fpres<F> operator*(T a, fpres<F> b) {  return b * a; }

template<typename T, int B, is_number(T)>
constexpr auto operator*(T a, fp<B> b) {  return b * fp<B>(a); }


template<typename T, int B>
auto operator-(T a, fp<B> b) {
  return fp<B>(a) - b;
}
template<int A, typename T, is_integer(T)>
auto operator-(fpres<A> a, T b) {
  return fpres<A>(a.val - (int64_t)b * (1LLU << A));
}
template<int A, typename T, is_float(T)>
auto operator-(fpres<A> a, T b) {
  return a - fpres<A>(b);
}



template<int A, int B>
auto operator+(fp<A> a, fp<B> b) {
  return a + fp<A>(b);
}
template<int A, int B>
fpres<A> operator+(fpres<A> a, fp<B> b) {
   return a + fpres<A>(b);
}
template<int A, int B>
fpres<A> operator+(fp<B> b, fpres<A> a) {
   return a + fpres<A>(b);
}

template<int A, typename T, is_number(T)>
auto operator+(fp<A> a, T b) {
  return a + fp<A>(b);
}
template<int A, typename T, is_number(T)>
auto operator+(T a, fp<A> b) {
  return fp<A>(a) + b;
}


template<typename T, int B>
auto operator/(T a, fp<B> b) {
  return fp<B>(a) / b;
}
template<int B>
fp<B> abs(fp<B> a) {
  return a.val < 0 ? -a : a;
}

template<int B>
int round(fp<B> a) {
  int result;
  if(a.val < 0) {
    a.val -= 1;
  } else {
    a.val += 1;
  }
  result = (a.val + a.N/2)/a.N;
  return result;
}

using fix1 = fp<1>;
using fix2 = fp<2>;
using fix3 = fp<3>;
using fix4 = fp<4>;
using fix5 = fp<5>;
using fix6 = fp<6>;
using fix7 = fp<7>;
using fix8 = fp<8>;
using fix9 = fp<9>;
using fix10 = fp<10>;
using fix11 = fp<11>;
using fix12 = fp<12>;
using fix13 = fp<13>;
using fix14 = fp<14>;
using fix15 = fp<15>;
using fix16 = fp<16>;
using fix17 = fp<17>;
using fix18 = fp<18>;
using fix19 = fp<19>;
using fix20 = fp<20>;
using fix21 = fp<21>;
using fix22 = fp<22>;
using fix23 = fp<23>;
using fix24 = fp<24>;
using fix25 = fp<25>;
using fix26 = fp<26>;
using fix27 = fp<27>;
using fix28 = fp<28>;
using fix29 = fp<29>;
using fix30 = fp<30>;
using fix31 = fp<31>;


using ufix1 = fp<1,uint32_t>;
using ufix2 = fp<2,uint32_t>;
using ufix3 = fp<3,uint32_t>;
using ufix4 = fp<4,uint32_t>;
using ufix5 = fp<5,uint32_t>;
using ufix6 = fp<6,uint32_t>;
using ufix7 = fp<7,uint32_t>;
using ufix8 = fp<8,uint32_t>;
using ufix9 = fp<9,uint32_t>;
using ufix10 = fp<10,uint32_t>;
using ufix11 = fp<11,uint32_t>;
using ufix12 = fp<12,uint32_t>;
using ufix13 = fp<13,uint32_t>;
using ufix14 = fp<14,uint32_t>;
using ufix15 = fp<15,uint32_t>;
using ufix16 = fp<16,uint32_t>;
using ufix17 = fp<17,uint32_t>;
using ufix18 = fp<18,uint32_t>;
using ufix19 = fp<19,uint32_t>;
using ufix20 = fp<20,uint32_t>;
using ufix21 = fp<21,uint32_t>;
using ufix22 = fp<22,uint32_t>;
using ufix23 = fp<23,uint32_t>;
using ufix24 = fp<24,uint32_t>;
using ufix25 = fp<25,uint32_t>;
using ufix26 = fp<26,uint32_t>;
using ufix27 = fp<27,uint32_t>;
using ufix28 = fp<28,uint32_t>;
using ufix29 = fp<29,uint32_t>;
using ufix30 = fp<30,uint32_t>;
using ufix31 = fp<31,uint32_t>;

#if __FPU_USED==1
using fp_t = float;
using fp20_t = float;
using fp29_t = float;
#else
using fp_t = fix16;
using fp20_t = fix20;
using fp29_t = fix29;
#endif
