/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation "vil_stream_url"
#endif
#include "vil_stream_url.h"

#include <vcl_cstdio.h>  // sprintf()
#include <vcl_cstring.h>
#include <vcl_cstdlib.h>
#include <vcl_string.h>
#include <vcl_iostream.h>
#include <vil/vil_stream_core.h>

#if defined(__unix__)

# include <unistd.h>       // read(), write(), close()
# include <netdb.h>        // gethostbyname(), sockaddr_in()
# include <sys/socket.h>
# include <netinet/in.h>   // htons()
# ifdef __alpha
#  include <fp.h>           // htons() [ on e.g. DEC alpha, htons is in machine/endian.h]
# endif
# define SOCKET int
#elif defined (VCL_WIN32)
# include <winsock2.h>
#endif

vil_stream_url::vil_stream_url(char const *url)
  : underlying(0)
{
  // split URL into auth, host, path and port number.
  vcl_string host;
  vcl_string path;
  vcl_string auth;
  int port = 80; // default
  if (vcl_strncmp(url, "http://", 7) != 0)
    return; // doesn't look like a URL to me....

  char const *p = url + 7;
  while (*p && *p!='/')
    ++ p;
  host = vcl_string(url+7, p);


  if (*p)
    path = p+1;
  else
    path = "";

  // port?
  for (unsigned int i=0; i<host.size(); ++i)
    if (host[i] == ':') {
      port = vcl_atoi(host.c_str() + i + 1);
      host = vcl_string(host.c_str(), host.c_str() + i);
      break;
    }

  //authentification
  for (unsigned int i=0; i<host.size(); ++i)
    if (host[i] == '@') {
      auth = vcl_string(host.c_str(), host.c_str()+i);
      host = vcl_string(host.c_str()+i+1, host.c_str() + host.size());
      break;
    }


  // so far so good.
#ifdef DEBUG
  vcl_cerr << "auth = \'" << auth << "\'" << vcl_endl
           << "host = \'" << host << "\'" << vcl_endl
           << "path = \'" << path << "\'" << vcl_endl
           << "port = " << port << vcl_endl;
#endif

#ifdef VCL_WIN32
  static int called_WSAStartup;
  if (called_WSAStartup==0)
  {
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

    wVersionRequested = MAKEWORD( 2, 2 );

    err = WSAStartup( wVersionRequested, &wsaData );
  }
#endif

  // create socket endpoint.
  SOCKET tcp_socket = socket(PF_INET,      // IPv4 protocols.
                             SOCK_STREAM,  // two-way, reliable, connection-based stream socket.
                             PF_UNSPEC);   // protocol number.

#ifdef VCL_WIN32
  if (tcp_socket == INVALID_SOCKET) {
    vcl_cerr << __FILE__ ": failed to create socket." << vcl_endl;
# ifndef NDEBUG
    vcl_cerr << "error code : " << WSAGetLastError() << vcl_endl;
# endif
    return;
  }
#else
  if (tcp_socket < 0)
    vcl_cerr << __FILE__ ": failed to create socket." << vcl_endl;
#endif

#ifdef DEBUG
  vcl_cerr << __FILE__ ": tcp_sockect = " << tcp_socket << vcl_endl;
#endif

  // get network address of server.
  hostent *hp = gethostbyname(host.c_str());
  if (! hp) {
    vcl_cerr << __FILE__ ": failed to lookup host" << vcl_endl;
    return;
  }

  // make socket address.
  sockaddr_in my_addr;
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);  // convert port number to network byte order..
  vcl_memcpy(&my_addr.sin_addr, hp->h_addr_list[0], hp->h_length);

  // connect to server.
  if (connect(tcp_socket , (sockaddr *) &my_addr, sizeof my_addr) < 0) {
    vcl_cerr << __FILE__ ": failed to connect to host" << vcl_endl;
    //perror(__FILE__);
    return;
  }

  // buffer for data transfers over socket.
  char buffer[4096];

  // send HTTP 1.0 request.
  vcl_sprintf(buffer, "GET http://%s/%s\n", host.c_str(), path.c_str());
  if (auth != "")
    vcl_sprintf(buffer+vcl_strlen(buffer), "Authorization:  user %s\n", auth.c_str());

#ifdef VCL_WIN32
  if (send(tcp_socket, buffer, vcl_strlen(buffer), 0) < 0)
#else
  if (::write(tcp_socket, buffer, vcl_strlen(buffer)) < 0)
#endif
  {
    vcl_cerr << __FILE__ ": error sending HTTP request" << vcl_endl;
    return;
  }

  // force the data to be sent.
#if 1
  shutdown(tcp_socket, 1); // disallow further sends.
#else
  for (int i=0; i<4096; ++i) ::write(tcp_socket, "\n\n\n\n", 4);
#endif

  // read from socket into memory.
  underlying = new vil_stream_core;
  underlying->ref();
  {
    int n;
#ifdef VCL_WIN32
    while ((n = recv(tcp_socket, buffer, sizeof buffer,0 )) > 0)
#else
    while ((n = ::read(tcp_socket, buffer, sizeof buffer)) > 0)
#endif
    {
      underlying->write(buffer, n);
      //vcl_cerr << n << " bytes" << vcl_endl;
    }
  }

  // close connection to server.
#ifdef VCL_WIN32
  closesocket(tcp_socket);
#else
  close(tcp_socket);
#endif
}

vil_stream_url::~vil_stream_url()
{
  if (underlying) {
    underlying->unref();
    underlying = 0;
  }
}

bool vil_stream_url::ok()
{
  return underlying && underlying->ok();
}

int vil_stream_url::write(void const *buf, int n)
{
  // strictly speaking, writes should fail, but that
  // isn't useful in any way.
  return underlying ? underlying->write(buf, n) : 0;
}

int vil_stream_url::read(void *buf, int n)
{
  return underlying ? underlying->read(buf, n) : 0;
}

int vil_stream_url::tell()
{
  return underlying ? underlying->tell() : -1;
}

void vil_stream_url::seek(int position)
{
  if (underlying) underlying->seek(position);
}
