/*
** $Id: lusartlib.c,v 1.0.0.0 2013/12/25 11:52:03 badlyby Exp $
** Usart library
** Copyright 2013 badlyby
*/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#define lusartlib_c
#define LUA_LIB

#include "lua.h"

#include "lauxlib.h"
#include "lualib.h"

#if defined(WIN32) || defined(WIN64)
	#include <windows.h>
	#include <process.h>
	const char defport[]="COM1";
#else
	#include <termios.h>
	#define TIOCMGET	0x5415
	#define TIOCMSET	0x5418
	#define TIOCM_DTR	0x002
	#define TIOCM_RTS	0x004
	const char defport[]="/dev/ttyUSB0";
#endif
  #define LUA_USARTHANDLE	"USARTHANDLE"
#if defined(WIN32) || defined(WIN64)
  #define tofilep(L)	((HANDLE *)luaL_checkudata(L, 1, LUA_USARTHANDLE))
#else
  #define tofilep(L)	((int *)luaL_checkudata(L, 1, LUA_USARTHANDLE))
#endif

#define defspeed 9600

static int pushresult (lua_State *L, int i, const char *filename) {
  int en = errno;  // calls to Lua API may change this value
  if (i) {
    lua_pushboolean(L, 1);
    return 1;
  }
  else {
    lua_pushnil(L);
    if (filename)
      lua_pushfstring(L, "%s: %s", filename, strerror(en));
    else
      lua_pushfstring(L, "%s", strerror(en));
    lua_pushinteger(L, en);
    return 3;
  }
}

static void fileerror (lua_State *L, int arg, const char *filename) {
  lua_pushfstring(L, "%s: %s", filename, strerror(errno));
  luaL_argerror(L, arg, lua_tostring(L, -1));
}

#if defined(WIN32) || defined(WIN64)
static HANDLE tofile (lua_State *L) {
  HANDLE *f = tofilep(L);
  if (*f == NULL)
    luaL_error(L, "attempt to use a closed file");
  return *f;
}
#else
static int tofile (lua_State *L) {
  int *f = tofilep(L);
  if (-1 == *f)
    luaL_error(L, "attempt to use a closed file");
  return *f;
}
#endif

/*
** When creating file handles, always creates a `closed' file handle
** before opening the actual file; so, if there is a memory error, the
** file is not left opened.
*/
#if defined(WIN32) || defined(WIN64)
static HANDLE *newfile (lua_State *L) {
	HANDLE *pf = (HANDLE *)lua_newuserdata(L, sizeof(HANDLE));
#else
static int *newfile (lua_State *L) {
	int *pf = (int *)lua_newuserdata(L, sizeof(int));
#endif
  luaL_getmetatable(L, LUA_USARTHANDLE);
  lua_setmetatable(L, -2);
  return pf;
}

/*
** function to (not) close the standard files stdin, stdout, and stderr
*/
static int usart_noclose (lua_State *L) {
  lua_pushnil(L);
  lua_pushliteral(L, "cannot close standard file");
  return 2;
}

/*
** function to close 'popen' files
*/
/*static int usart_pclose (lua_State *L) {
//  FILE **p = tofilep(L);
//  int ok = lua_pclose(L, *p);
//  *p = NULL;
#if defined(WIN32) || defined(WIN64)
  HANDLE *p = tofilep(L);
  int ok = (lua_pclose(L, *p) == 0);
  *p = NULL;
#else
  int *p = tofilep(L);
  int ok = (lua_pclose(L, *p) == 0);
  *p = -1;
#endif
  return pushresult(L, ok, NULL);
}*/


/*
** function to close regular files
*/
static int usart_fclose (lua_State *L) {
#if defined(WIN32) || defined(WIN64)
  HANDLE *p = tofilep(L);
  int ok = (CloseHandle(*p) == 0);
  *p = NULL;
#else
  int *p = tofilep(L);
  int ok = (close(*p) == 0);
  *p = -1;
#endif
  return pushresult(L, ok, NULL);
}


static int aux_close (lua_State *L) {
  lua_getfenv(L, 1);
  lua_getfield(L, -1, "__close");
  return (lua_tocfunction(L, -1))(L);
}


static int usart_close (lua_State *L) {
//  if (lua_isnone(L, 1))
//    lua_rawgeti(L, LUA_ENVIRONINDEX, IO_OUTPUT);
  tofile(L);  // make sure argument is a file
  return aux_close(L);
}


static int usart_gc (lua_State *L) {
#if defined(WIN32) || defined(WIN64)
  HANDLE f = *tofilep(L);
  // ignore closed files
  if (f != NULL)
#else
  int f = *tofilep(L);
  // ignore closed files
  if (-1 != f)
#endif
    aux_close(L);
  return 0;
}


static int usart_tostring (lua_State *L) {
#if defined(WIN32) || defined(WIN64)
  HANDLE f = *tofilep(L);
  if (f == NULL)
#else
  int f = *tofilep(L);
  if (-1 == f)
#endif
    lua_pushliteral(L, "file (closed)");
  else
    lua_pushfstring(L, "file (%p)", f);
  return 1;
}

#if defined(WIN32) || defined(WIN64)
HANDLE set_uart(const char *portname, int speed, int databits, int stopbits, int parity)
{
	DCB dcb;
	DWORD count;
	COMMTIMEOUTS TimeOuts;
	HANDLE fuart;
	unsigned char *newport;
	newport = (unsigned char *)malloc(strlen(portname+5));
	sprintf(newport,"\\\\.\\%s",portname);
	fuart = CreateFile(newport,GENERIC_WRITE|GENERIC_READ,0,NULL,OPEN_EXISTING,0/*FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED*/,NULL);
	free(newport);
	if(fuart == INVALID_HANDLE_VALUE)
	{
		printf("Cannot open %s (error   %d)\n",portname,GetLastError());
		exit(-1);
	}
	GetCommState(fuart,&dcb);
	dcb.BaudRate=speed;
	dcb.ByteSize=databits;
	dcb.fParity = TRUE;
	switch(parity)
	{
	default:
	case 'n':
	case 'N':
		dcb.Parity=NOPARITY;
		dcb.fParity = FALSE;
		break;
	case 'o':
	case 'O':
		dcb.Parity=ODDPARITY;
		break;
	case 'e':
	case 'E':
		dcb.Parity=EVENPARITY;
		break;
	case 's':
	case 'S':  
		dcb.Parity=SPACEPARITY;
		break;
	case 'm':
	case 'M':
		dcb.Parity=MARKPARITY;
		break;
	}
	switch(stopbits)
	{
	case 1:
		dcb.StopBits=ONESTOPBIT;
		break;
	case 15:
		dcb.StopBits=ONE5STOPBITS;
		break;
	case 2:
		dcb.StopBits=TWOSTOPBITS;
		break;
	default:
		dcb.StopBits=ONESTOPBIT;
		break;
	}
	dcb.fBinary = TRUE;
	dcb.fNull = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	//dcb.fDtrControl = 0;
	SetCommState(fuart,&dcb);
	SetupComm(fuart, 1024, 1024);
	PurgeComm(fuart, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	TimeOuts.ReadIntervalTimeout=0;
	TimeOuts.ReadTotalTimeoutMultiplier=0;
	TimeOuts.ReadTotalTimeoutConstant=50;
	TimeOuts.WriteTotalTimeoutMultiplier=1;
	TimeOuts.WriteTotalTimeoutConstant=50;
	SetCommTimeouts(fuart,&TimeOuts);
	return fuart;
}
#else
int set_uart(const char *portname, int speed, int databits, int stopbits, int parity)
{
	int i,fuart;
	int status;
	int speed_arr[] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300 };
	int name_arr[] = { 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300 };
	struct termios opt;
	fuart=open(portname,O_RDWR | O_NOCTTY | O_NDELAY);
	tcgetattr(fuart, &opt);
	for(i = 0; i<sizeof(speed_arr)/sizeof(int); i++)
	{
		if (speed==name_arr[i])
		{
			tcflush(fuart, TCIOFLUSH);
			cfsetispeed(&opt, speed_arr[i]);
			cfsetospeed(&opt, speed_arr[i]);
			status = tcsetattr(fuart, TCSANOW, &opt);
			if (status = 0) tcflush(fuart, TCIOFLUSH);
		
		}
	}
	if( tcgetattr(fuart, &opt)  !=  0) return(-1);
	opt.c_cflag &= ~CSIZE;
	opt.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN); 
	opt.c_lflag |= NOFLSH;
	opt.c_oflag &= ~OPOST;
	switch (databits)
	{
	case 5:
		opt.c_cflag |= CS5;
		break;
	case 6:
		opt.c_cflag |= CS6;
		break;
	case 7:
		opt.c_cflag |= CS7;
		break;
	case 8:
		opt.c_cflag |= CS8;
		break;
	default:
		printf("databits error!\n");
		return(-1);
	}
	switch (parity)
	{
	case 'n':
	case 'N':
		opt.c_cflag &= ~PARENB;
		opt.c_iflag &= ~INPCK;
		break;
	case 'o':
	case 'O':
		opt.c_cflag |= (PARODD | PARENB);  /* parity checking */
		opt.c_iflag |= INPCK;
		break;
	case 'e':
	case 'E':
		opt.c_cflag |= PARENB;     /* Enable parity */
		opt.c_cflag &= ~PARODD;   /*  */
		opt.c_iflag |= INPCK;
		break;
	case 's':
	case 'S':
		opt.c_cflag |= PARENB;
		opt.c_cflag |= CMSPAR;
		opt.c_cflag &= ~PARODD;
		opt.c_iflag |= INPCK;
		break;
	case 'm':
	case 'M':
		opt.c_cflag |= PARENB;
		opt.c_cflag |= CMSPAR;
		opt.c_cflag |= PARODD;
		opt.c_iflag |= INPCK;
		break;
	default:
		printf("parity error!\n");
		return(-1);
	}
	switch (stopbits)
	{
	case 1:
		opt.c_cflag &= ~CSTOPB;
		break;
	case 2:
		opt.c_cflag |= CSTOPB;
		break;
	default:
		printf("stopbits error!\n");
		return(-1);
	}
	opt.c_iflag |= IGNBRK;
	opt.c_iflag |= IGNPAR;
	opt.c_iflag &=~(IXON|INLCR|IGNCR|ICRNL|IXOFF);
	opt.c_oflag &=~(ONLCR|OCRNL|ONOCR|ONLRET|OFILL); 
	tcflush(fuart, TCIFLUSH); 
	opt.c_cc[VTIME] = 1;
	opt.c_cc[VMIN] = 0;
	if(tcsetattr(fuart, TCSANOW, &opt) != 0)
	{
		printf("TCSANOW error!\n");
		return(-1);
	}
	return(fuart);
}
#endif

static int usart_open (lua_State *L) {
	int n = lua_gettop(L);
	const char *dev = defport;
	const char *paritys;
	int speed = defspeed;
	int databits = 8;
	int stopbits = 1;
	int parity = 'n';
	#if defined(WIN32) || defined(WIN64)
		HANDLE *fuart = newfile(L);
	#else
		int *fuart = newfile(L);
	#endif
	if((n>=1) && (!lua_isnil(L,1)))
	{
		dev = lua_tostring(L,1);
	}
	if((n>=2) && (!lua_isnil(L,2)))
	{
		speed = lua_tonumber(L,2);
	}
	if((n>=3) && (!lua_isnil(L,3)))
	{
		databits = lua_tonumber(L,3);
	}
	if((n>=4) && (!lua_isnil(L,4)))
	{
		stopbits = lua_tonumber(L,4);
	}
	if((n>=5) && (!lua_isnil(L,5)))
	{
		paritys = lua_tostring(L,5);
		parity = paritys[0];
	}
	*fuart = set_uart(dev,speed,databits,stopbits,parity);

#if defined(WIN32) || defined(WIN64)
	return (fuart == NULL) ? pushresult(L, 0, dev) : 1;
#else
	return (-1 == *fuart) ? pushresult(L, 0, dev) : 1;
#endif
  
}

static int uart_read(lua_State *L)
{
	int n = lua_gettop(L);
	unsigned char *data;
	int length = 1024;
	#if defined(WIN32) || defined(WIN64)
		DWORD count;
		HANDLE fuart;
	#else
		int count;
		int fuart;
	#endif
	if((n < 1) || (lua_isnil(L,1))) return 0;
	fuart = tofile(L);
	if((n >= 2) && (!lua_isnil(L,2)))
	{
		length = lua_tonumber(L, 2);
	}
	data = malloc(length);
	#if defined(WIN32) || defined(WIN64)
		ReadFile(fuart,data,length,&count,NULL);
	#else
		count = read(fuart,data,length);
		if(count<0) count = 0;
	#endif
	lua_pushlstring (L, data, count);
	lua_pushnumber(L, count);
	free(data);
	return 2;
}

static int uart_write(lua_State *L)
{
	int n = lua_gettop(L);
	const unsigned char *data;
	int length = 0;
	#if defined(WIN32) || defined(WIN64)
		DWORD count;
		HANDLE fuart;
	#else
		int count;
		int fuart;
	#endif
	if((n < 2) || (lua_isnil(L,1))) return 0;
	fuart = tofile(L);
	data = lua_tostring(L, 2);
	if((n >= 3) && (!lua_isnil(L,3)))
	{
		length = lua_tonumber(L, 3);
	}
	else
	{
		length = lua_strlen(L, 2);
	}
	#if defined(WIN32) || defined(WIN64)
		WriteFile(fuart,data,length,&count,NULL);
	#else
		count = write(fuart,data,length);
	#endif
	lua_pushnumber(L, count);
	return 1;
}

static int _msleep(lua_State *L)
{
	int n = lua_gettop(L);
	int nTimes = 1000;
	if((n >= 1) && (!lua_isnil(L,1)))
	{
		nTimes = lua_tonumber(L, 1);
	}
	#if defined(WIN32) || defined(WIN64)
	Sleep(nTimes);
	#else
	usleep(nTimes*1000);
	#endif
	return 0;
}
#if defined(WIN32) || defined(WIN64)
void DTR(HANDLE fuart, int set)
{
	DCB dcb;
	GetCommState(fuart,&dcb);
	if(set) dcb.fDtrControl = DTR_CONTROL_ENABLE;
	else dcb.fDtrControl = DTR_CONTROL_DISABLE;
	SetCommState(fuart,&dcb);
}

void RTS(HANDLE fuart, int set)
{
	DCB dcb;
	GetCommState(fuart,&dcb);
	if(set) dcb.fRtsControl = RTS_CONTROL_ENABLE;
	else dcb.fRtsControl = RTS_CONTROL_DISABLE;
	SetCommState(fuart,&dcb);
}
#else
void DTR(int fuart, int set)
{
	int status;
	ioctl(fuart, TIOCMGET, &status);
	if(set) status |= TIOCM_DTR;
	else status &= ~TIOCM_DTR;
	ioctl(fuart, TIOCMSET, &status);
}

void RTS(int fuart, int set)
{
	int status;
	ioctl(fuart, TIOCMGET, &status);
	if(set) status |= TIOCM_RTS;
	else status &= ~TIOCM_RTS;
	ioctl(fuart, TIOCMSET, &status);
}
#endif
static int setDTR(lua_State *L)
{
	int n = lua_gettop(L),set;
	#if defined(WIN32) || defined(WIN64)
		HANDLE fuart;
	#else
		int fuart;
	#endif
	if((n < 2) || (lua_isnil(L,1))) return 0;
	fuart = tofile(L);
	set = lua_toboolean(L,2);
	DTR(fuart, set);
	return 0;
}

static int setRTS(lua_State *L)
{
	int n = lua_gettop(L),set;
	#if defined(WIN32) || defined(WIN64)
		HANDLE fuart;
	#else
		int fuart;
	#endif
	if((n < 2) || (lua_isnil(L,1))) return 0;
	fuart = tofile(L);
	set = lua_toboolean(L,2);
	RTS(fuart, set);
	return 0;
}

static const luaL_Reg usartlib[] = {
  {"close", usart_close},
  {"open", usart_open},
  {"msleep", _msleep},
  {NULL, NULL}
};


static const luaL_Reg flib[] = {
  {"close", usart_close},
  {"read", uart_read},
  {"write", uart_write},
  {"setDTR", setDTR},
  {"setRTS", setRTS},
  {"__gc", usart_gc},
  {"__tostring", usart_tostring},
  {NULL, NULL}
};


static void createmeta (lua_State *L) {
  luaL_newmetatable(L, LUA_USARTHANDLE);  /* create metatable for file handles */
  lua_pushvalue(L, -1);  /* push metatable */
  lua_setfield(L, -2, "__index");  /* metatable.__index = metatable */
  luaL_register(L, NULL, flib);  /* file methods */
}

static void newfenv (lua_State *L, lua_CFunction cls) {
  lua_createtable(L, 0, 1);
  lua_pushcfunction(L, cls);
  lua_setfield(L, -2, "__close");
}

LUALIB_API int luaopen_usart (lua_State *L) {
  createmeta(L);
  /* create (private) environment (with fields IO_INPUT, IO_OUTPUT, __close) */
  newfenv(L, usart_fclose);
  lua_replace(L, LUA_ENVIRONINDEX);
  /* open library */
  luaL_register(L, "usart", usartlib);
  /* create (and set) default files */
  newfenv(L, usart_noclose);  /* close function for default files */
  lua_pop(L, 1);  /* pop environment for default files */
  return 1;
}

