#include <err.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h>

#include "Printer.h"

#define START 1
#define WAIT 2
#define OK 4
#define RESEND 8

#define START_CHR "start\r\n"
#define WAIT_CHR "wait\r\n"
#define OK_CHR "ok "
#define RESEND_CHR "Resend:"

static int wait(FILE *f, int type, int lineNumber) {
	char buf[128];
	for(int i = 0; i < 10; i++) {
		fgets(buf, 127, f);
		printf("%s", buf);
#define CHECK(SYM) if(type & SYM && !strncmp(buf, SYM ## _CHR, strlen(SYM ## _CHR))) return SYM;
		CHECK(START);
		CHECK(WAIT);
		CHECK(RESEND);
#undef CHECK
		
		if(type & OK) {
		    if(!strncmp(buf, OK_CHR, strlen(OK_CHR)) && atoi(buf + strlen(OK_CHR)) == lineNumber) {
		        return OK;
        	}
	    }
	}
}

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
	if ((fd = open(device,O_RDWR|O_NOCTTY|O_NONBLOCK)) == -1)
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
		if (1 || serinfo.custom_divisor * rate != serinfo.baud_base) {
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
				 int protocol,
				 float home_x,
				 float home_y,
				 float home_z,
				 float home_e)
	: fd(serialOpen(file, baudRate)),
	  file(fdopen(fd, "rw")),
	  protocol(protocol),
	  x(NAN),
	  y(NAN),
	  z(NAN),
	  lineNumber(0)
{
	wait(this->file, START, 0);
	setHome(home_x, home_y, home_z, home_e);
	
	// Reset line number
	sendBinaryCode(M, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

Printer::~Printer() {
	close(fd);
}

void
Printer::home() {
	sendBinaryCode(G, 0, 28, 0, 0, 0, 0, 0, 0, 0, 0);
}

void
Printer::move(float x, float y, float z, float e, float f) {
	uint16_t flag = G;
	if(!isnan(x)) {
		flag |= X;
		this->x = x;
	}
	if(!isnan(y)) {
		flag |= Y;
		this->y = y;
	}
	if(!isnan(z)) {
		flag |= Z;
		this->z = z;
	}
	if(!isnan(e)) {
		flag |= E;
		this->e = e;
	}
	if(!isnan(f)) {
		flag |= F;
		this->f = f;
	}
	sendBinaryCode(flag, 0, 1, x, y, z, e, f, 0, 0, 0);
}

void
Printer::moveRelative(float x, float y, float z, float e, float f) {
	move(this->x+x, this->y+y, this->z+z, this->e+e, f);
}

void
Printer::sendBinaryCode(uint16_t flag,
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

	// Disable v2 stuff
	flag &= ~ (V2 | EXT | INT | TXT);

	// Always send it as binary and with a linenumber
	flag |= NOTASCII | N;

	if(m == 110 && flag&M) {
		lineNumber = 0;
	}

	memcpy(buf + len, &flag, 2); len += 2;

#define check(XX, xx, ll) if(flag & XX) { memcpy(buf + len, &xx, ll); len += ll; }
	check(N, lineNumber, 2);
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

	uint16_t fl1 = 0,
		fl2 = 0;
	for(int i = 0; i < len; i++) {
		fl1 = (fl1 + buf[i]) % 255;
		fl2 = (fl2 + fl1) % 255;
	}
	buf[len++] = fl1 & 0xFF;
	buf[len++] = fl2 & 0xFF;

	do {
		// Use a loop, or it won't work.
		// Some timing issue?
		for(int i = 0; i < len; i++) {
			write(fd, buf + i, 1);
		}
	} while (wait(file, RESEND|OK, lineNumber) == RESEND);

	lineNumber++;
}


void
Printer::setHome(float x, float y, float z, float home_e) {
	home_x = x;
	home_y = y;
	home_z = z;
	home_e = e;
}

