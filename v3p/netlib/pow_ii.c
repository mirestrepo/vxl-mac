#include "f2c.h"

#ifdef KR_headers
integer pow_ii(ap, bp) integer *ap, *bp;
#else
integer pow_ii(integer *ap, integer *bp)
#endif
{
  integer pow = 1, x = *ap, n = *bp;
  unsigned long u;

  if (n <= 0)
  {
    if (n == 0 || x == 1)
      return 1;
    if (x != -1)
      return x == 0 ? 1/x : 0;
    n = -n;
  }
  u = n;
  while(1)
  {
    if(u & 01)
      pow *= x;
    if(u >>= 1)
      x *= x;
    else
      break;
  }
  return(pow);
}
