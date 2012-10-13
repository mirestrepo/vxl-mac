#include <vcl_iostream.h>
#include <vcl_iomanip.h>
#include <vcl_limits.h> // for infinity()
#include <vxl_config.h> // for VCL_STATIC_CONST_INIT_FLOAT_NO_DEFN
#include <vnl/vnl_math.h>
#include <vnl/vnl_complex.h> // for vnl_math_abs(std::complex)
#include <testlib/testlib_test.h>

#if !VCL_STATIC_CONST_INIT_FLOAT_NO_DEFN
static
void check_pointer( const void * )
{
}

static
void test_static_const_definition()
{
  check_pointer( &vnl_math::e );
  check_pointer( &vnl_math::euler );
  check_pointer( &vnl_math::log2e );
  check_pointer( &vnl_math::log10e );
  check_pointer( &vnl_math::ln2 );
  check_pointer( &vnl_math::ln10 );
  check_pointer( &vnl_math::pi );
  check_pointer( &vnl_math::twopi );
  check_pointer( &vnl_math::pi_over_2 );
  check_pointer( &vnl_math::pi_over_4 );
  check_pointer( &vnl_math::pi_over_180 );
  check_pointer( &vnl_math::one_over_pi );
  check_pointer( &vnl_math::two_over_pi );
  check_pointer( &vnl_math::sqrt2pi );
  check_pointer( &vnl_math::one_over_sqrt2pi );
  check_pointer( &vnl_math::two_over_sqrtpi );
  check_pointer( &vnl_math::deg_per_rad );
  check_pointer( &vnl_math::sqrt2 );
  check_pointer( &vnl_math::sqrt1_2 );
  check_pointer( &vnl_math::sqrt1_3 );
  check_pointer( &vnl_math::eps );
  check_pointer( &vnl_math::sqrteps );
}
#endif

// Test that the vnl_math constants don't have weird values
static void test_math_constants()
{
#define TEST_CONSTANT(a,v) TEST_NEAR("value: ", vnl_math::a, v, 1e-9);
  TEST_NEAR("log of e is 1"     , log(vnl_math::e), 1.0, 1e-9);
  TEST_CONSTANT(e               , 2.7182818285);
  TEST_NEAR("log2e * ln2 = 1"   , vnl_math::log2e * vnl_math::ln2, 1.0, 1e-9);
  TEST_CONSTANT(log2e           , 1.4426950409);
  TEST_CONSTANT(ln2             , 0.6931471806);
  TEST_NEAR("log10e * ln10 = 1" , vnl_math::log10e * vnl_math::ln10, 1.0, 1e-9);
  TEST_CONSTANT(log10e          , 0.4342944819);
  TEST_CONSTANT(ln10            , 2.3025850930);
  TEST_NEAR("cos(pi) = -1"      , cos(vnl_math::pi), -1.0, 1e-9);
  TEST_CONSTANT(pi              , 3.1415926536);
  TEST_NEAR("twopi = 2*pi"      , vnl_math::twopi, 2.0*vnl_math::pi, 1e-9);
  TEST_CONSTANT(twopi           , 6.2831853072);
  TEST_NEAR("pi_over_2 = pi/2"  , vnl_math::pi_over_2, 0.5*vnl_math::pi, 1e-9);
  TEST_CONSTANT(pi_over_2       , 1.5707963268);
  TEST_NEAR("pi_over_4 = pi/4"  , vnl_math::pi_over_4, 0.25*vnl_math::pi, 1e-9);
  TEST_CONSTANT(pi_over_4       , 0.7853981634);
  TEST_NEAR("pi_over_180=pi/180", vnl_math::pi_over_180, vnl_math::pi/180.0, 1e-9);
  TEST_CONSTANT(pi_over_180     , 0.0174532925);
  TEST_NEAR("pi*one_over_pi=1"  , vnl_math::pi*vnl_math::one_over_pi, 1.0, 1e-9);
  TEST_CONSTANT(one_over_pi     , 0.3183098862);
  TEST_NEAR("pi*two_over_pi=2"  , vnl_math::pi*vnl_math::two_over_pi, 2.0, 1e-9);
  TEST_CONSTANT(two_over_pi     , 0.6366197724);
  TEST_NEAR("deg_per_rad=180/pi", vnl_math::deg_per_rad, 180.0/vnl_math::pi, 1e-9);
  TEST_CONSTANT(deg_per_rad     , 57.295779513);
  TEST_NEAR("sqrt2pi^2"         , vnl_math::sqrt2pi*vnl_math::sqrt2pi, vnl_math::twopi, 1e-9);
  TEST_CONSTANT(sqrt2pi         , 2.5066282746);
  TEST_NEAR("two_over_sqrtpi"   , vnl_math::two_over_sqrtpi, 2.0/sqrt(vnl_math::pi), 1e-9);
  TEST_CONSTANT(two_over_sqrtpi , 1.1283791671);
  TEST_NEAR("one_over_sqrt2pi"  , vnl_math::one_over_sqrt2pi, 1.0/sqrt(vnl_math::twopi), 1e-9);
  TEST_CONSTANT(one_over_sqrt2pi, 0.3989422804);
  TEST_NEAR("sqrt2*sqrt2=2"     , vnl_math::sqrt2*vnl_math::sqrt2, 2.0, 1e-9);
  TEST_CONSTANT(sqrt2           , 1.4142135624);
  TEST_NEAR("sqrt1_2*sqrt2=1"   , vnl_math::sqrt1_2*vnl_math::sqrt2, 1.0, 1e-9);
  TEST_CONSTANT(sqrt1_2         , 0.7071067812);
  TEST_NEAR("sqrt1_3^2=1/3"     , vnl_math::sqrt1_3*vnl_math::sqrt1_3, 1.0/3.0, 1e-9);
  TEST_CONSTANT(sqrt1_3         , 0.5773502692);
  TEST_NEAR("euler"             , vnl_math::euler, 0.5772156649, 1e-9);
#undef TEST_CONSTANT
}

static void test_math()
{
#if !VCL_STATIC_CONST_INIT_FLOAT_NO_DEFN
  // Call it to avoid compiler warnings
  test_static_const_definition();
#endif
  test_math_constants();

  int n = -11;
  float f = -7.5f;
  double d = -vnl_math::pi;
  vcl_complex<double> i(0,1);
  vcl_complex<double> z(-1,2);
  vcl_complex<double> e_ipi = vcl_exp(d*i);

  vcl_cout << "n = " << n << vcl_endl
           << "f = " << f << vcl_endl
           << "d = " << d << vcl_endl
           << "i = " << i << vcl_endl
           << "z = " << z << vcl_endl
           << "exp(d*i) = " << e_ipi << vcl_endl
           << vcl_endl

           << "abs(n) = " << vnl_math_abs(n) << vcl_endl
           << "abs(f) = " << vnl_math_abs(f) << vcl_endl
           << "abs(d) = " << vnl_math_abs(d) << vcl_endl
           << "abs(i) = " << vnl_math_abs(i) << vcl_endl
           << "abs(z) = " << vnl_math_abs(z) << vcl_endl
           <<"norm(z) = " << vnl_math_squared_magnitude(z) << vcl_endl
           << vcl_endl;

  TEST("abs(n) == 11", vnl_math_abs(n), 11);
  TEST("abs(f) == 7.5f", vnl_math_abs(f), 7.5f);
  TEST("abs(d) == pi", vnl_math_abs(d), vnl_math::pi);
  TEST("abs(i) == 1", vnl_math_abs(i), 1.0);
  TEST_NEAR("abs(-1+2i)~=sqrt(5)",vnl_math_abs(z),vcl_sqrt(5.0), 1e-12);
  TEST_NEAR("norm(-1+2i) ~= 5", vnl_math_squared_magnitude(z),5, 1e-12);
  TEST_NEAR("exp(d*i) ~= -1", vnl_math_abs(e_ipi+1.0), 0, 1e-12);
  vcl_cout << vcl_endl;

  TEST("rnd(-8.4999)  == -8  ", vnl_math_rnd(-8.4999), -8);
  TEST("rnd(-8.4999f) == -8  ", vnl_math_rnd(-8.4999f), -8);
  TEST("rnd(-8.50)    == -8/9", vnl_math_rnd(-8.50)/2, -4);
  TEST("rnd(-8.50f)   == -8/9", vnl_math_rnd(-8.50f)/2, -4);
  TEST("rnd(-8.5001)  == -9  ", vnl_math_rnd(-8.5001), -9);
  TEST("rnd(-8.5001f) == -9  ", vnl_math_rnd(-8.5001f), -9);
  TEST("rnd(8.4999)   ==  8  ", vnl_math_rnd(8.4999),  8);
  TEST("rnd(8.4999f)  ==  8  ", vnl_math_rnd(8.4999f),  8);
  TEST("rnd(8.50)     ==  8/9", vnl_math_rnd(8.50)/2,  4);
  TEST("rnd(8.50f)    ==  8/9", vnl_math_rnd(8.50f)/2,  4);
  TEST("rnd(8.5001)   ==  9  ", vnl_math_rnd(8.5001),  9);
  TEST("rnd(8.5001f)  ==  9  ", vnl_math_rnd(8.5001f),  9);

  TEST("rnd(-9.4999)  == -9   ", vnl_math_rnd(-9.4999), -9);
  TEST("rnd(-9.4999f) == -9   ", vnl_math_rnd(-9.4999f), -9);
  TEST("rnd(-9.50)    == -9/10", (vnl_math_rnd(-9.50)+1)/2, -4);
  TEST("rnd(-9.50f)   == -9/10", (vnl_math_rnd(-9.50f)+1)/2, -4);
  TEST("rnd(-9.5001)  == -10  ", vnl_math_rnd(-9.5001), -10);
  TEST("rnd(-9.5001f) == -10  ", vnl_math_rnd(-9.5001f), -10);
  TEST("rnd(9.4999)   ==  9   ", vnl_math_rnd(9.4999),  9);
  TEST("rnd(9.4999f)  ==  9   ", vnl_math_rnd(9.4999f),  9);
  TEST("rnd(9.50)     ==  9/10", (vnl_math_rnd(9.50)-1)/2,  4);
  TEST("rnd(9.50f)    ==  9/10", (vnl_math_rnd(9.50f)-1)/2,  4);
  TEST("rnd(9.5001)   ==  10  ", vnl_math_rnd(9.5001),  10);
  TEST("rnd(9.5001f)  ==  10  ", vnl_math_rnd(9.5001f),  10);

  TEST("rnd_halfinttoeven(-8.4999)  == -8", vnl_math_rnd_halfinttoeven(-8.4999), -8);
  TEST("rnd_halfinttoeven(-8.4999f) == -8", vnl_math_rnd_halfinttoeven(-8.4999f), -8);
  TEST("rnd_halfinttoeven(-8.50)    == -8", vnl_math_rnd_halfinttoeven(-8.50), -8);
  TEST("rnd_halfinttoeven(-8.50f)   == -8", vnl_math_rnd_halfinttoeven(-8.50f), -8);
  TEST("rnd_halfinttoeven(-8.5001)  == -9", vnl_math_rnd_halfinttoeven(-8.5001), -9);
  TEST("rnd_halfinttoeven(-8.5001f) == -9", vnl_math_rnd_halfinttoeven(-8.5001f), -9);
  TEST("rnd_halfinttoeven(8.4999)   ==  8", vnl_math_rnd_halfinttoeven(8.4999),  8);
  TEST("rnd_halfinttoeven(8.4999f)  ==  8", vnl_math_rnd_halfinttoeven(8.4999f),  8);
  TEST("rnd_halfinttoeven(8.50)     ==  8", vnl_math_rnd_halfinttoeven(8.50),  8);
  TEST("rnd_halfinttoeven(8.50f)    ==  8", vnl_math_rnd_halfinttoeven(8.50f),  8);
  TEST("rnd_halfinttoeven(8.5001)   ==  9", vnl_math_rnd_halfinttoeven(8.5001),  9);
  TEST("rnd_halfinttoeven(8.5001f)  ==  9", vnl_math_rnd_halfinttoeven(8.5001f),  9);

  TEST("rnd_halfinttoeven(-9.4999)  == -9 ", vnl_math_rnd_halfinttoeven(-9.4999), -9);
  TEST("rnd_halfinttoeven(-9.4999f) == -9 ", vnl_math_rnd_halfinttoeven(-9.4999f), -9);
  TEST("rnd_halfinttoeven(-9.50)    == -10", vnl_math_rnd_halfinttoeven(-9.50), -10);
  TEST("rnd_halfinttoeven(-9.50f)   == -10", vnl_math_rnd_halfinttoeven(-9.50f), -10);
  TEST("rnd_halfinttoeven(-9.5001)  == -10", vnl_math_rnd_halfinttoeven(-9.5001), -10);
  TEST("rnd_halfinttoeven(-9.5001f) == -10", vnl_math_rnd_halfinttoeven(-9.5001f), -10);
  TEST("rnd_halfinttoeven(9.4999)   ==  9 ", vnl_math_rnd_halfinttoeven(9.4999),  9);
  TEST("rnd_halfinttoeven(9.4999f)  ==  9 ", vnl_math_rnd_halfinttoeven(9.4999f),  9);
  TEST("rnd_halfinttoeven(9.50)     ==  10", vnl_math_rnd_halfinttoeven(9.50),  10);
  TEST("rnd_halfinttoeven(9.50f)    ==  10", vnl_math_rnd_halfinttoeven(9.50f),  10);
  TEST("rnd_halfinttoeven(9.5001)   ==  10", vnl_math_rnd_halfinttoeven(9.5001),  10);
  TEST("rnd_halfinttoeven(9.5001f)  ==  10", vnl_math_rnd_halfinttoeven(9.5001f),  10);

  TEST("rnd_halfintup(-8.4999)  == -8", vnl_math_rnd_halfintup(-8.4999), -8);
  TEST("rnd_halfintup(-8.4999f) == -8", vnl_math_rnd_halfintup(-8.4999f), -8);
  TEST("rnd_halfintup(-8.50)    == -8", vnl_math_rnd_halfintup(-8.50), -8);
  TEST("rnd_halfintup(-8.50f)   == -8", vnl_math_rnd_halfintup(-8.50f), -8);
  TEST("rnd_halfintup(-8.5001)  == -9", vnl_math_rnd_halfintup(-8.5001), -9);
  TEST("rnd_halfintup(-8.5001f) == -9", vnl_math_rnd_halfintup(-8.5001f), -9);
  TEST("rnd_halfintup(8.4999)   ==  8", vnl_math_rnd_halfintup(8.4999),  8);
  TEST("rnd_halfintup(8.4999f)  ==  8", vnl_math_rnd_halfintup(8.4999f),  8);
  TEST("rnd_halfintup(8.50)     ==  9", vnl_math_rnd_halfintup(8.50),  9);
  TEST("rnd_halfintup(8.50f)    ==  9", vnl_math_rnd_halfintup(8.50f),  9);
  TEST("rnd_halfintup(8.5001)   ==  9", vnl_math_rnd_halfintup(8.5001),  9);
  TEST("rnd_halfintup(8.5001f)  ==  9", vnl_math_rnd_halfintup(8.5001f),  9);

  TEST("rnd_halfintup(-9.4999)  == -9 ", vnl_math_rnd_halfintup(-9.4999), -9);
  TEST("rnd_halfintup(-9.4999f) == -9 ", vnl_math_rnd_halfintup(-9.4999f), -9);
  TEST("rnd_halfintup(-9.50)    == -9 ", vnl_math_rnd_halfintup(-9.50), -9);
  TEST("rnd_halfintup(-9.50f)   == -9 ", vnl_math_rnd_halfintup(-9.50f), -9);
  TEST("rnd_halfintup(-9.5001)  == -10", vnl_math_rnd_halfintup(-9.5001), -10);
  TEST("rnd_halfintup(-9.5001f) == -10", vnl_math_rnd_halfintup(-9.5001f), -10);
  TEST("rnd_halfintup(9.4999)   ==  9 ", vnl_math_rnd_halfintup(9.4999),  9);
  TEST("rnd_halfintup(9.4999f)  ==  9 ", vnl_math_rnd_halfintup(9.4999f),  9);
  TEST("rnd_halfintup(9.50)     ==  10", vnl_math_rnd_halfintup(9.50),  10);
  TEST("rnd_halfintup(9.50f)    ==  10", vnl_math_rnd_halfintup(9.50f),  10);
  TEST("rnd_halfintup(9.5001)   ==  10", vnl_math_rnd_halfintup(9.5001),  10);
  TEST("rnd_halfintup(9.5001f)  ==  10", vnl_math_rnd_halfintup(9.5001f),  10);

  TEST("floor(8.0)      ==  8", vnl_math_floor(8.0),  8);
  TEST("floor(8.0f)     ==  8", vnl_math_floor(8.0f),  8);
  TEST("floor(8.9999)   ==  8", vnl_math_floor(8.9999),  8);
  TEST("floor(8.9999f)  ==  8", vnl_math_floor(8.9999f),  8);
  TEST("floor(8.0001)   ==  8", vnl_math_floor(8.0001),  8);
  TEST("floor(8.0001f)  ==  8", vnl_math_floor(8.0001f),  8);
  TEST("floor(-8.0)     == -8", vnl_math_floor(-8.0), -8);
  TEST("floor(-8.0f)    == -8", vnl_math_floor(-8.0f), -8);
  TEST("floor(-8.9999)  == -9", vnl_math_floor(-8.9999), -9);
  TEST("floor(-8.9999f) == -9", vnl_math_floor(-8.9999f), -9);
  TEST("floor(-8.0001)  == -9", vnl_math_floor(-8.0001), -9);
  TEST("floor(-8.0001f) == -9", vnl_math_floor(-8.0001f), -9);

  TEST("floor(9.0)      ==  9 ", vnl_math_floor(9.0),  9);
  TEST("floor(9.0f)     ==  9 ", vnl_math_floor(9.0f),  9);
  TEST("floor(9.9999)   ==  9 ", vnl_math_floor(9.9999),  9);
  TEST("floor(9.9999f)  ==  9 ", vnl_math_floor(9.9999f),  9);
  TEST("floor(9.0001)   ==  9 ", vnl_math_floor(9.0001),  9);
  TEST("floor(9.0001f)  ==  9 ", vnl_math_floor(9.0001f),  9);
  TEST("floor(-9.0)     == -9 ", vnl_math_floor(-9.0), -9);
  TEST("floor(-9.0f)    == -9 ", vnl_math_floor(-9.0f), -9);
  TEST("floor(-9.9999)  == -10", vnl_math_floor(-9.9999), -10);
  TEST("floor(-9.9999f) == -10", vnl_math_floor(-9.9999f), -10);
  TEST("floor(-9.0001)  == -10", vnl_math_floor(-9.0001), -10);
  TEST("floor(-9.0001f) == -10", vnl_math_floor(-9.0001f), -10);

  TEST("ceil(8.0)      ==  8", vnl_math_ceil(8.0),  8);
  TEST("ceil(8.0f)     ==  8", vnl_math_ceil(8.0f),  8);
  TEST("ceil(8.9999)   ==  9", vnl_math_ceil(8.9999),  9);
  TEST("ceil(8.9999f)  ==  9", vnl_math_ceil(8.9999f),  9);
  TEST("ceil(8.0001)   ==  9", vnl_math_ceil(8.0001),  9);
  TEST("ceil(8.0001f)  ==  9", vnl_math_ceil(8.0001f),  9);
  TEST("ceil(-8.0)     == -8", vnl_math_ceil(-8.0), -8);
  TEST("ceil(-8.0f)    == -8", vnl_math_ceil(-8.0f), -8);
  TEST("ceil(-8.9999)  == -8", vnl_math_ceil(-8.9999), -8);
  TEST("ceil(-8.9999f) == -8", vnl_math_ceil(-8.9999f), -8);
  TEST("ceil(-8.0001)  == -8", vnl_math_ceil(-8.0001), -8);
  TEST("ceil(-8.0001f) == -8", vnl_math_ceil(-8.0001f), -8);

  TEST("ceil(9.0)      ==  9", vnl_math_ceil(9.0),  9);
  TEST("ceil(9.0f)     ==  9", vnl_math_ceil(9.0f),  9);
  TEST("ceil(9.9999)   == 10", vnl_math_ceil(9.9999), 10);
  TEST("ceil(9.9999f)  == 10", vnl_math_ceil(9.9999f), 10);
  TEST("ceil(9.0001)   == 10", vnl_math_ceil(9.0001), 10);
  TEST("ceil(9.0001f)  == 10", vnl_math_ceil(9.0001f), 10);
  TEST("ceil(-9.0)     == -9", vnl_math_ceil(-9.0), -9);
  TEST("ceil(-9.0f)    == -9", vnl_math_ceil(-9.0f), -9);
  TEST("ceil(-9.9999)  == -9", vnl_math_ceil(-9.9999), -9);
  TEST("ceil(-9.9999f) == -9", vnl_math_ceil(-9.9999f), -9);
  TEST("ceil(-9.0001)  == -9", vnl_math_ceil(-9.0001), -9);
  TEST("ceil(-9.0001f) == -9", vnl_math_ceil(-9.0001f), -9);

  TEST(" isfinite(f)    ",  vnl_math_isfinite(f), true);
  TEST(" isfinite(d)    ",  vnl_math_isfinite(d), true);
  TEST(" isfinite(i)    ",  vnl_math_isfinite(i), true);
  TEST(" isfinite(z)    ",  vnl_math_isfinite(z), true);


  // There is an assumption in this code that vcl_numeric_limits<float/double>::has_infinity==true

  TEST("vcl_numeric_limits<float>::has_infinity==true assumption",vcl_numeric_limits<float>::has_infinity, true);
  TEST("vcl_numeric_limits<double>::has_infinity==true assumption",vcl_numeric_limits<double>::has_infinity, true);
  TEST("vcl_numeric_limits<ldouble>::has_infinity==true assumption",vcl_numeric_limits<long double>::has_infinity, true);
  if (! vcl_numeric_limits<float>::has_infinity && ! vcl_numeric_limits<double>::has_infinity)
  {
    vcl_cout << "Your platform doesn't appear to have an infinity. VXL is in places relatively\n"
             << "dependent on the existence of an infinity. There are two solutions.\n"
             << "A. If your platform really doesn't have an infinity, VXL's configuration code\n"
             << "   can be modified to correctly detect and use the infinity.\n"
             << "B. Fix VXL so that it can cope with the lack of an infinity.\n" << vcl_endl;
  }
  TEST("vcl_numeric_limits<float>::has_quiet_NaN==true assumption",vcl_numeric_limits<float>::has_quiet_NaN, true);
  TEST("vcl_numeric_limits<double>::has_quiet_NaN==true assumption",vcl_numeric_limits<double>::has_quiet_NaN, true);
  TEST("vcl_numeric_limits<ldouble>::has_quiet_NaN==true assumption",vcl_numeric_limits<long double>::has_quiet_NaN, true);
  if (! vcl_numeric_limits<float>::has_quiet_NaN && ! vcl_numeric_limits<double>::has_quiet_NaN)
  {
    vcl_cout << "Your platform doesn't appear to have a quiet NaN. VXL is in places relatively\n"
             << "dependent on the existence of a quiet NaN. There are two solutions.\n"
             << "A. If your platform really doesn't have a quiet NaN, VXL's configuration code\n"
             << "   can be modified to correctly detect and use the NaN.\n"
             << "B. Fix VXL so that it can cope with the lack of a quiet NaN.\n" << vcl_endl;
  }
  // Create Inf and -Inf:
  float pinf_f =   vcl_numeric_limits<float>::infinity();
  float ninf_f = - vcl_numeric_limits<float>::infinity();
  double pinf_d =   vcl_numeric_limits<double>::infinity();
  double ninf_d = - vcl_numeric_limits<double>::infinity();
  long double pinf_q =   vcl_numeric_limits<long double>::infinity();
  long double ninf_q = - vcl_numeric_limits<long double>::infinity();

  // Create NaN
  float qnan_f = vcl_numeric_limits<float>::quiet_NaN();
  double qnan_d = vcl_numeric_limits<double>::quiet_NaN();
  long double qnan_q = vcl_numeric_limits<long double>::quiet_NaN();

#define print_hex(p) \
  vcl_hex<<vcl_setfill('0')<<vcl_setw(2)<<(short)reinterpret_cast<unsigned char*>(&p)[sizeof(p)-1]; \
  for (unsigned int i=2; i<=sizeof(p); ++i) \
    vcl_cout<<vcl_setfill('0')<<vcl_setw(2)<<(short)(reinterpret_cast<unsigned char*>(&p))[sizeof(p)-i]; \
  vcl_cout<<vcl_dec

  vcl_cout << "pinf_f = " << pinf_f << " = " << print_hex(pinf_f) << vcl_endl
           << "ninf_f = " << ninf_f << " = " << print_hex(ninf_f) << vcl_endl
           << "pinf_d = " << pinf_d << " = " << print_hex(pinf_d) << vcl_endl
           << "ninf_d = " << ninf_d << " = " << print_hex(ninf_d) << vcl_endl
           << "pinf_q = " << pinf_q << " = " << print_hex(pinf_q) << vcl_endl
           << "ninf_q = " << ninf_q << " = " << print_hex(ninf_q) << vcl_endl
           << "qnan_f = " << qnan_f << " = " << print_hex(qnan_f) << vcl_endl
           << "qnan_d = " << qnan_d << " = " << print_hex(qnan_d) << vcl_endl
           << "qnan_q = " << qnan_q << " = " << print_hex(qnan_q) << vcl_endl
           << vcl_endl;
#undef print_hex

#ifndef __alpha__ // on alpha, infinity() == max()
  TEST("!isfinite(pinf_f)", vnl_math_isfinite(pinf_f), false);
  TEST("!isfinite(ninf_f)", vnl_math_isfinite(ninf_f), false);
  TEST(" isinf(pinf_f)   ",  vnl_math_isinf(pinf_f), true);
  TEST(" isinf(ninf_f)   ",  vnl_math_isinf(ninf_f), true);
#endif
  TEST("!isnan(pinf_f)   ", vnl_math_isnan(pinf_f), false);
  TEST("!isnan(ninf_f)   ", vnl_math_isnan(ninf_f), false);
  TEST("!isfinite(qnan_f)", vnl_math_isfinite(qnan_f), false);
  TEST("!isinf(qnan_f)   ", vnl_math_isinf(qnan_f), false);
  TEST(" isnan(qnan_f)   ",  vnl_math_isnan(qnan_f), true);

#ifndef __alpha__ // on alpha, infinity() == max()
  TEST("!isfinite(pinf_d)", vnl_math_isfinite(pinf_d), false);
  TEST("!isfinite(ninf_d)", vnl_math_isfinite(ninf_d), false);
  TEST(" isinf(pinf_d)   ",  vnl_math_isinf(pinf_d), true);
  TEST(" isinf(ninf_d)   ",  vnl_math_isinf(ninf_d), true);
#endif
  TEST("!isnan(pinf_d)   ", vnl_math_isnan(pinf_d), false);
  TEST("!isnan(ninf_d)   ", vnl_math_isnan(ninf_d), false);
  TEST("!isfinite(qnan_d)", vnl_math_isfinite(qnan_d), false);
  TEST("!isinf(qnan_d)   ", vnl_math_isinf(qnan_d), false);
  TEST(" isnan(qnan_d)   ",  vnl_math_isnan(qnan_d), true);

#ifndef __ICC // "long double" has no standard internal representation on different platforms/compilers
#ifndef __alpha__ // on alpha, infinity() == max()
  TEST("!isfinite(pinf_q)", vnl_math_isfinite(pinf_q), false);
  TEST("!isfinite(ninf_q)", vnl_math_isfinite(ninf_q), false);
  TEST(" isinf(pinf_q)   ",  vnl_math_isinf(pinf_q), true);
  TEST(" isinf(ninf_q)   ",  vnl_math_isinf(ninf_q), true);
#endif
  TEST("!isnan(pinf_q)   ", vnl_math_isnan(pinf_q), false);
  TEST("!isnan(ninf_q)   ", vnl_math_isnan(ninf_q), false);
  TEST("!isfinite(qnan_q)", vnl_math_isfinite(qnan_q), false);
  TEST("!isinf(qnan_q)   ", vnl_math_isinf(qnan_q), false);
#if 0 // even more nonstandard ...
  TEST(" isnan(qnan_q)   ",  vnl_math_isnan(qnan_q));
#endif // 0
#endif // __ICC

  TEST("!isfinite(huge_val(double))", vnl_math_isfinite(vnl_huge_val(double())), false);
  TEST("!isfinite(huge_val(float))",  vnl_math_isfinite(vnl_huge_val(float())),  false);

  vcl_cout << vcl_endl;

  // test vnl_math::angle_0_to_2pi() for "extreme values":
  TEST("vnl_math::angle_0_to_2pi(2pi)", vnl_math::angle_0_to_2pi(vnl_math::twopi), 0.0);
  double eps = 2e-16; // which is smaller than the precision of vnl_math::pi
  double conv_eps = vnl_math::angle_0_to_2pi(-eps);
  vcl_cout << "conv_eps = " << conv_eps << " = 2pi - " << vnl_math::twopi-conv_eps << vcl_endl;
  TEST("vnl_math::angle_0_to_2pi(-eps)", conv_eps < vnl_math::twopi && conv_eps > 6.283, true);
  eps = 2e-15; // which is larger than the precision of vnl_math::pi
  conv_eps = vnl_math::angle_0_to_2pi(-eps);
  vcl_cout << "conv_eps = " << conv_eps << " = 2pi - " << vnl_math::twopi-conv_eps << vcl_endl;
  TEST("vnl_math::angle_0_to_2pi(-10eps)", conv_eps < vnl_math::twopi - 1e-15 && conv_eps > 6.283, true);
  double ang = vnl_math::twopi - eps;
  double conv_ang = vnl_math::angle_0_to_2pi(ang);
  vcl_cout << "conv_ang = " << conv_ang << " = 2pi - " << vnl_math::twopi-conv_ang << vcl_endl;
  TEST("vnl_math::angle_0_to_2pi(2pi-10eps)", conv_ang, ang);
  // test vnl_math::angle_minuspi_to_pi() for "extreme values":
  TEST("vnl_math::angle_minuspi_to_pi(2pi)", vnl_math::angle_minuspi_to_pi(vnl_math::twopi), 0.0);
  TEST("vnl_math::angle_minuspi_to_pi(pi)", vnl_math::angle_minuspi_to_pi(vnl_math::pi), vnl_math::pi);
  TEST("vnl_math::angle_minuspi_to_pi(-pi)", vnl_math::angle_minuspi_to_pi(-vnl_math::pi), -vnl_math::pi);
  eps = 2e-16; // which is smaller than the precision of vnl_math::pi
  conv_eps = vnl_math::angle_minuspi_to_pi(-eps);
  vcl_cout << "conv_eps = " << conv_eps << vcl_endl;
  TEST("vnl_math::angle_minuspi_to_pi(-eps)", conv_eps, -eps);
  eps = 2e-15; // which is larger than the precision of vnl_math::pi
  conv_eps = vnl_math::angle_minuspi_to_pi(-eps);
  vcl_cout << "conv_eps = " << conv_eps << vcl_endl;
  TEST("vnl_math::angle_minuspi_to_pi(-10eps)", conv_eps, -eps);
}

TESTMAIN(test_math);
