#include <err.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h>

#include "Printer.h"


static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
	switch(baudrate) {
		B(50);     B(75);     B(110);    B(134);    B(150);
		B(200);    B(300);    B(600);    B(1200);   B(1800);
		B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
		B(57600);  B(115200); B(230400); B(460800); B(500000); 
		B(576000); B(921600); B(1000000);B(1152000);B(1500000); 
	default: return 0;
	}
#undef B
}    

static int serialOpen(const char *device, int rate)
{
	struct termios options;
	struct serial_struct serinfo;
	int fd;
	int speed = 0;

	/* Open and configure serial port */
	if ((fd = open(device,O_RDWR|O_NOCTTY)) == -1)
		return -1;

	speed = rate_to_constant(rate);

	if (speed == 0) {
		/* Custom divisor */
		serinfo.reserved_char[0] = 0;
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
			return -1;
		serinfo.flags &= ~ASYNC_SPD_MASK;
		serinfo.flags |= ASYNC_SPD_CUST;
		serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
		if (serinfo.custom_divisor < 1) 
			serinfo.custom_divisor = 1;
		if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
			return -1;
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
			return -1;
		if (serinfo.custom_divisor * rate != serinfo.baud_base) {
			warnx("actual baudrate is %d / %d = %f",
			      serinfo.baud_base, serinfo.custom_divisor,
			      (float)serinfo.baud_base / serinfo.custom_divisor);
		}
	}

	fcntl(fd, F_SETFL, 0);
	tcgetattr(fd, &options);
	cfsetispeed(&options, speed ? speed : B38400);
	cfsetospeed(&options, speed ? speed : B38400);
	cfmakeraw(&options);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CRTSCTS;
	if (tcsetattr(fd, TCSANOW, &options) != 0)
		return -1;

	return fd;
}

Printer::Printer(const char *file,
				 int baudRate,
				 int protocol)
	: fd(serialOpen(file, baudRate)),
	  protocol(protocol),
	  baudRate(baudRate),
	  x(NAN),
	  y(NAN),
	  z(NAN)
{
	setHome(0, 0, 0, 0);
}

Printer::Printer(const char *file,
				 int baudRate,
				 int protocol,
				 float home_x,
				 float home_y,
				 float home_z,
				 float home_e)
	: fd(serialOpen(file, baudRate)),
	  protocol(protocol),
	  x(NAN),
	  y(NAN),
	  z(NAN)
{
	setHome(home_x, home_y, home_z, home_e);
}

void
Printer::home() {
	sendBinaryCode(G, 0, 0, 28, 0, 0, 0, 0, 0, 0, 0, 0);
}

void
Printer::move(float x, float y, float z, float e, float f) {
	uint16_t flag = G;
	if(x != NAN) {
		flag |= X;
		this->x = x;
	}
	if(y != NAN) {
		flag |= Y;
		this->y = y;
	}
	if(z != NAN) {
		flag |= Z;
		this->z = z;
	}
	if(e != NAN) {
		flag |= E;
		this->e = e;
	}
	if(f != NAN) {
		flag |= F;
		this->f = f;
	}
	sendBinaryCode(flag, 0, 0, 1, x, y, z, e, f, 0, 0, 0);
}

void
Printer::moveRelative(float x, float y, float z, float e, float f) {
	move(this->x+x, this->y+y, this->z+z, this->e+e, f);
}

void
Printer::sendBinaryCode(uint16_t flag,
						int16_t n,
						uint8_t m,
						uint8_t g,
						float x, float y, float z,
						float e,
						float f,
						int8_t t,
						int32_t s,
						int32_t p) {
	uint8_t buf[29];
	int len = 0;

	flag &= ~ (V2 | EXT | INT | TXT);
	flag |= NOTASCII;

	memcpy(buf + len, &flag, 2); len += 2;

#define check(X, x, l) if(flag & X) { memcpy(buf + len, &x, l); len += l; }
	check(N, n, 2);
	check(M, m, 1);
	check(G, g, 1);
	check(X, x, 4);
	check(Y, y, 4);
	check(Z, z, 4);
	check(E, e, 4);
	check(F, f, 4);
	check(T, t, 1);
	check(S, s, 4);
	check(P, p, 4);
#undef check

	uint8_t fl1 = 0,
		fl2 = 0;
	for(int i = 0; i < len; i++) {
		fl1 += buf[i];
		fl2 += fl1;
	}
	buf[len++] = fl2;
	buf[len++] = fl1;

	write(fd, buf, len);
}

void
Printer::setHome(float x, float y, float z, float home_e) {
	home_x = x;
	home_y = y;
	home_z = z;
	home_e = e;
}

