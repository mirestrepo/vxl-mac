#ifndef vcl_win32_vc60_complex_fwd_h_
#define vcl_win32_vc60_complex_fwd_h_

// forward declare the native std::complex<T> :
namespace std { template <class T> class complex; }
#ifndef vcl_complex
# define vcl_complex std :: complex
#endif

#endif // vcl_win32_vc60_complex_fwd_h_
