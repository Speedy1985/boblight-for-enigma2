/*
 * boblight
 * Copyright (C) Bob  2009, Modded by Speedy1985 (c) 2013
 * 
 * boblight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * boblight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "inclstdint.h"

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream> //debug
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/un.h>

#include "sockets.h"
#include "misc.h"

using namespace std;

void CData::SetData(uint8_t* data, int size, bool append)
{
  CopyData(reinterpret_cast<char*>(data), size, append);
}

/*void CData::SetData(char* data, int size, bool append)
{
  CopyData(data, size, append);
}

void CData::SetData(const uint8_t* data, int size, bool append)
{
  CopyData(reinterpret_cast<char*>(const_cast<uint8_t*>(data)), size, append);
}

void CData::SetData(const char* data, int size, bool append)
{
  CopyData(const_cast<char*>(data), size, append);
}*/

void CData::SetData(std::string data, bool append)
{
  CopyData(const_cast<char*>(data.c_str()), data.length(), append);
}

//store data as c-string
void CData::CopyData(char* data, int size, bool append)
{
  if (append)
  {
    int start = m_data.size() - 1;
    m_data.resize(m_data.size() + size);
    memcpy(&m_data[start], data, size);
    m_data.back() = 0;
  }
  else
  {
    m_data.resize(size + 1);
    memcpy(&m_data[0], data, size);
    m_data.back() = 0;
  }
}

void CData::Clear()
{
  m_data.resize(1);
  m_data.back() = 0;
}

CSocket::CSocket()
{
  m_sock = -1;
  m_port = -1;
}

CSocket::~CSocket()
{
  Close();
}

//can't open int the base class
int CSocket::TcpOpen(std::string address, int port, int usectimeout)
{
  return FAIL;
}

int CSocket::UnixOpen(std::string address, int port, int usectimeout)
{
  return FAIL;
}

void CSocket::Close()
{
  if (m_sock != -1)
  {
    SetNonBlock(false);
    close(m_sock);
    m_sock = -1;
  }
}

int CSocket::SetNonBlock(bool nonblock)
{
  //non-blocking socket, because we work with select
  int flags = fcntl(m_sock, F_GETFL);
  if (flags == -1)
  {
    m_error = "F_GETFL " + GetErrno();
    return FAIL;
  }

  if (nonblock)
    flags |= O_NONBLOCK;
  else
    flags &= ~O_NONBLOCK;
  
  if (fcntl(m_sock, F_SETFL, flags) == -1)
  {
    m_error = "F_SETFL " + GetErrno();
    return FAIL;
  }

  return SUCCESS;
}

int CSocket::SetSockOptions()
{
  //set tcp keepalive
  SetKeepalive();

  //disable nagle algorithm
  int flag = 1;
  if (setsockopt(m_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) == -1)
  {
    m_error = "TCP_NODELAY " + GetErrno();
    return FAIL;
  }

  return SUCCESS;
}

int CSocket::SetKeepalive()
{
#if defined(SO_KEEPALIVE) && defined(TCP_KEEPCNT) && defined(TCP_KEEPIDLE) && defined(TCP_KEEPINTVL)

  int flag = 1;

  //turn keepalive on
  if (setsockopt(m_sock, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag)) == -1)
  {
    m_error = "SO_KEEPALIVE " + GetErrno();
    return FAIL;
  }

  //two keepalive probes
  flag = 2;
  if (setsockopt(m_sock, IPPROTO_TCP, TCP_KEEPCNT, &flag, sizeof(flag)) == -1)
  {
    m_error = "TCP_KEEPCNT " + GetErrno();
    return FAIL;
  }

  //20 seconds before we start using keepalive
  flag = 20;
  if (setsockopt(m_sock, IPPROTO_TCP, TCP_KEEPIDLE, &flag, sizeof(flag)) == -1)
  {
    m_error = "TCP_KEEPIDLE " + GetErrno();
    return FAIL;
  }

  //20 seconds timeout of each keepalive packet
  flag = 20;
  if (setsockopt(m_sock, IPPROTO_TCP, TCP_KEEPINTVL, &flag, sizeof(flag)) == -1)
  {
    m_error = "TCP_KEEPINTVL " + GetErrno();
    return FAIL;
  }

#else

#warning keepalive support not compiled in

#endif

  return SUCCESS;
}

//wait until the socket becomes readable or writeable
int CSocket::WaitForSocket(bool write, std::string timeoutstr)
{
  int returnv;
  fd_set rwsock;
  struct timeval *tv = NULL;

  //add the socket to the fd_set
  FD_ZERO(&rwsock);
  FD_SET(m_sock, &rwsock);

  //set the timeout
  struct timeval timeout;
  if (m_usectimeout > 0)
  {
    timeout.tv_sec = m_usectimeout / 1000000;
    timeout.tv_usec = m_usectimeout % 1000000;
    tv = &timeout;
  }

  if (write)
    returnv = select(m_sock + 1, NULL, &rwsock, NULL, tv);
  else
    returnv = select(m_sock + 1, &rwsock, NULL, NULL, tv);
  
  if (returnv == 0) //select timed out
  {
    m_error = m_address + ":" + ToString(m_port) + " " + timeoutstr + " timed out"; 
    return TIMEOUT;
  }
  else if (returnv == -1) //select had an error
  {
    m_error = "select() " + GetErrno();
    return FAIL;
  }

  //check if the socket had any errors, connection refused is a common one
  int sockstate, sockstatelen = sizeof(sockstate);
  returnv = getsockopt(m_sock, SOL_SOCKET, SO_ERROR, &sockstate, reinterpret_cast<socklen_t*>(&sockstatelen));
  
  if (returnv == -1) //getsockopt had an error
  {
    m_error = "getsockopt() " + GetErrno();
    return FAIL;
  }
  else if (sockstate) //socket had an error
  {
    m_error = "SO_ERROR " + m_address + ":" + ToString(m_port) + " " + GetErrno(sockstate);
    return FAIL;
  }

  return SUCCESS;
}

//open a client tpsocket
int CClientSocket::TcpOpen(std::string address, int port, int usectimeout /*= -1*/)
{
  Close(); //close it if it was opened

  //store address, port and timeout
  m_address = address;
  m_port = port;
  m_usectimeout = usectimeout;
  
  m_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (m_sock == -1) //can't make socket
  {
    m_error = "socket() " + GetErrno();
    return FAIL;
  }

  if (SetNonBlock() != SUCCESS)
  {
    return FAIL;
  }
  
  struct sockaddr_in server; //where we connect to
  memset(&server, 0, sizeof(server));

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address.c_str());
  server.sin_port = htons(port);

  struct hostent *host = gethostbyname(address.c_str());
  if (!host) //can't find host
  {
    m_error = "gethostbyname() " + address + ":" + ToString(m_port) + " " + GetErrno();
    return FAIL;
  }
  server.sin_addr.s_addr = *reinterpret_cast<in_addr_t*>(host->h_addr);

  if (connect(m_sock, reinterpret_cast<struct sockaddr*>(&server), sizeof(server)) < 0)
  {
    if (errno != EINPROGRESS) //because of the non blocking socket, this means we're still connecting
    {
      m_error = "connect() " + address + ":" + ToString(port) + " " + GetErrno();
      return FAIL;
    }
  }

  int returnv = WaitForSocket(true, "Connect");//wait for the socket to become writeable, that means the connection is established

  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;

  //if this fails the socket might still work, so don't return an error
  SetSockOptions();
  
  return SUCCESS;
}

//open a client unixsocket
int CClientSocket::UnixOpen(std::string address, int port, int usectimeout /*= -1*/)
{

  Close(); //close it if it was opened
  
  //store address, port and timeout
  m_address = SERVER_PATH;
  m_port = 19333;
  m_usectimeout = usectimeout;
  
  m_sock = socket(AF_UNIX, SOCK_STREAM, 0);

  if (m_sock == -1) //can't make socket
  {
    m_error = "socket() " + GetErrno();
    return FAIL;
  }
  
  if (SetNonBlock() != SUCCESS)
  {
    return FAIL;
  }
  
  struct sockaddr_un server; //where we connect to
  memset(&server, 0, sizeof(server));

  server.sun_family = AF_UNIX;
  strcpy(server.sun_path, SERVER_PATH);

  if (connect(m_sock, reinterpret_cast<struct sockaddr*>(&server), sizeof(server)) < 0)
  {
    if (errno != EINPROGRESS) //because of the non blocking socket, this means we're still connecting
    {
      m_error = "connect() " + ToString(SERVER_PATH) + ": " + GetErrno();
      return FAIL;
    }
  }
  
  int returnv = WaitForSocket(true, "Connect");//wait for the socket to become writeable, that means the connection is established

  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;

  //if this fails the socket might still work, so don't return an error
  SetSockOptions();

  return SUCCESS;
}

int CClientSocket::Read(CData& data)
{
  uint8_t buff[1000];
  
  if (m_sock == -1)
  {
    m_error = "socket closed";
    return FAIL;
  }

  int returnv = WaitForSocket(false, "Read");//wait until the socket has something to read
  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;  
  
  //clear the tcpdata
  data.Clear();

  //loop until the socket has nothing more in its buffer
  while(1)
  {
    int size = recv(m_sock, buff, sizeof(buff), 0);

    if (errno == EAGAIN && size == -1) //we're done here, no more data, the call to WaitForSocket made sure there was at least some data to read
    {
      return SUCCESS;
    }    
    else if (size == -1) //socket had an error
    {
      m_error = "recv() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
      return FAIL;
    }
    else if (size == 0 && data.GetSize() == 0) //socket closed and no data received
    {
      m_error = m_address + ":" + ToString(m_port) + " Connection closed";
      return FAIL;
    }
    else if (size == 0) //socket closed but data received
    {
      return SUCCESS;
    }

    data.SetData(buff, size, true); //append the data
  }

  return SUCCESS;
}

int CClientSocket::Write(CData& data)
{
  if (m_sock == -1)
  {
    m_error = "socket closed";
    return FAIL;
  }

  int bytestowrite = data.GetSize();
  int byteswritten = 0;

  //loop until we've written all bytes
  while (byteswritten < bytestowrite)
  {
    //wait until socket becomes writeable
    int returnv = WaitForSocket(true, "Write");

    if (returnv == FAIL || returnv == TIMEOUT)
      return returnv;

    int size = send(m_sock, data.GetData() + byteswritten, data.GetSize() - byteswritten, 0);
    
    if (size == -1)
    {
      m_error = "send() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
      return FAIL;
    }

    byteswritten += size;
  }
  return SUCCESS;
}

int CClientSocket::SetInfo(std::string address, int port, int sock)
{
  m_address = address;
  m_port = port;
  m_sock = sock;

  int returnv = SetNonBlock();
  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;

  returnv = SetSockOptions();
  
  return returnv;
}

int CServerSocket::TcpOpen(std::string address, int port, int usectimeout)
{
  struct sockaddr_in bindaddr;

  Close();
  
  if (address.empty())
    m_address = "*";
  else
    m_address = address;
  
  m_port = port;
  m_usectimeout = usectimeout;
  m_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (m_sock == -1)
  {
    m_error = "socket() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
    return FAIL;
  }

  int opt = 1;
  setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  
  //bind the socket
  memset(&bindaddr, 0, sizeof(bindaddr));
  bindaddr.sin_family = AF_INET;
  bindaddr.sin_port = htons(m_port);
  if (address.empty())
  {
    bindaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  else
  {
    struct hostent *host = gethostbyname(address.c_str());
    if (host == NULL)
    {
      m_error = "gethostbyname() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
      return FAIL;
    }
    bindaddr.sin_addr.s_addr = *reinterpret_cast<in_addr_t*>(host->h_addr);
  }

  if (bind(m_sock, reinterpret_cast<struct sockaddr*>(&bindaddr), sizeof(bindaddr)) < 0)
  {
    m_error = "bind() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
    return FAIL;
  }

  if (listen(m_sock, SOMAXCONN) < 0)
  {
    m_error = "listen() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
    return FAIL;
  }

  if (SetNonBlock() != SUCCESS)
  {
    return FAIL;
  }
  
  return SUCCESS;
}

int CServerSocket::UnixOpen(std::string address, int port, int usectimeout)
{
  if (address.empty())
    m_address = "*";
  else
    m_address = address;
  
  m_port = port;
  m_usectimeout = usectimeout;
  
  // Close old conncection if there is.
  Close();

  m_sock = socket(AF_UNIX, SOCK_STREAM, 0);

  if (m_sock == -1)
  {
    m_error = "socket() " + ToString(SERVER_PATH) + ":" + GetErrno();
    return FAIL;
  }
  
  int opt = 1;
  setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
      
   // Remove old file
  unlink(SERVER_PATH);
  
  struct sockaddr_un bindaddr;
  
  // bind the socket
  memset(&bindaddr, 0, sizeof(bindaddr));
  bindaddr.sun_family = AF_UNIX;
  strcpy(bindaddr.sun_path, SERVER_PATH);
  
  bind(m_sock, reinterpret_cast<struct sockaddr*>(&bindaddr), sizeof(bindaddr));

  if (listen(m_sock, SOMAXCONN) < 0)
  {
    m_error = "listen() " + m_address + ":" + ToString(m_port) + " " + GetErrno();
    return FAIL;
  }

  if (SetNonBlock() != SUCCESS)
  {
    return FAIL;
  }
  return SUCCESS;
}

int CServerSocket::TcpAccept(CClientSocket& socket)
{
  struct sockaddr_in client;
  socklen_t clientlen = sizeof(client);

  if (m_sock == -1)
  {
    m_error = "socket closed";
    return FAIL;
  }

  int returnv = WaitForSocket(false, "Accept");  //wait for socket to become readable
  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;
  
  int sock = accept(m_sock, reinterpret_cast<struct sockaddr*>(&client), &clientlen);
  if (sock < 0)
  {
    m_error = "accept() " + GetErrno();
    return FAIL;
  }

  if (socket.SetInfo(inet_ntoa(client.sin_addr), ntohs(client.sin_port), sock) != SUCCESS)
  {
    m_error = socket.GetError();
    return FAIL;
  }
  
  return SUCCESS;
}

int CServerSocket::UnixAccept(CClientSocket& socket)
{
  printf("CServerSocket::Accept\n");  
  struct sockaddr_un client;
  socklen_t clientlen = sizeof(client);

  if (m_sock == -1)
  {
    m_error = "socket closed";
    return FAIL;
  }

  int returnv = WaitForSocket(false, "Accept");  //wait for socket to become readable
  if (returnv == FAIL || returnv == TIMEOUT)
    return returnv;
  
  int sock = accept(m_sock, reinterpret_cast<struct sockaddr*>(&client), &clientlen);
  if (sock < 0)
  {
    m_error = "accept() " + GetErrno();
    return FAIL;
  }
  
  socket.SetInfo(SERVER_PATH, 19333, sock);
  /*
  if (socket.SetInfo(inet_ntoa(client.sun_addr), ntohs(client.sun_port), sock) != SUCCESS)
  {
    m_error = socket.GetError();
    return FAIL;
  }*/
  
  return SUCCESS;
}

