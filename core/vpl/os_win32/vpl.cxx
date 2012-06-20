// This is core/vpl/os_win32/vpl.cxx
#include <sys/types.h>
#include <sys/stat.h>
#include <direct.h>
#include <io.h>
#include <process.h>
#include <windows.h>

char *
vpl_getcwd( char *buf, vcl_size_t buf_size )
{
  return _getcwd( buf, (int)buf_size );
}

int
vpl_mkdir( const char *dir, unsigned short /*mode*/ )
{
  _mkdir( dir );
  return 0;
}

int
vpl_rmdir( const char *dir )
{
#if _MSC_VER >= 1400
  return _rmdir( dir );
#else
  return rmdir( dir );
#endif
}

int
vpl_chdir( const char *dir )
{
#if _MSC_VER >= 1400
  return _chdir( dir );
#else
  return chdir( dir );
#endif
}

int
vpl_unlink( const char *file )
{
#if defined(VCL_BORLAND)
  return unlink( file );
#else
  return _unlink( file );
#endif
}

unsigned int
vpl_sleep( unsigned int t )
{
  Sleep( long(t) * 1000 );
  return 0;
}

int
vpl_usleep( unsigned int t )
{
  Sleep( t / 1000 );
  return 0;
}

unsigned
vpl_getpid( )
{
#if defined(VCL_BORLAND)
  return getpid();
#else
  return _getpid();
#endif
}

int vpl_putenv ( const char * envvar )
{
#if defined(VCL_BORLAND)
  return putenv(envvar);
#else
  return _putenv(envvar);
#endif
}


int vpl_gethostname(char *name, vcl_size_t len)
{
#if defined(VCL_VC)
  static bool wsa_initialised = false;

  if (!wsa_initialised)
  {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2,2), &wsaData);
  }
#endif
  return gethostname(name, len);
}
