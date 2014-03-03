#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h>

#include <chrono>

#include "Printer.h"

#define START "start\r\n"
#define OKCRLF "ok\r\n"
#define OKNUM "ok "
#define RESEND "Resend:"
#define WAIT "wait\r\n"

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
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
			perror("TIOCGSERIAL");
			return -1;
		}
		serinfo.flags &= ~ASYNC_SPD_MASK;
		serinfo.flags |= ASYNC_SPD_CUST;
		serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
		if (serinfo.custom_divisor < 1) 
			serinfo.custom_divisor = 1;
		if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
			perror("TIOCSSERIAL");
			return -1;
		}
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
			perror("TIOCGSERIAL");
			return -1;
		}
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
	: 
	  fd(serialOpen(file, baudRate)),
	  file(fdopen(fd, "rw")),
	  serialReadNum(0),
	  running(true),
	  started(false),
	  protocol(protocol),
	  x(NAN),
	  y(NAN),
	  z(NAN),
	  e(NAN),
	  f(NAN),
	  lineNumber(-1),
	  confirmedLineNumber(-1)
{
	if(fd == -1) {
		throw "Blah :|";
	}

	std::unique_lock<std::mutex> writeLock(writeMutex);
	serialReadThread = std::thread(&Printer::serialRead, this);
	while(!started) {
		signalNextCommand.wait(writeLock);
	}
	writeLock.unlock();
	serialWriteThread = std::thread(&Printer::serialWrite, this);

	setHome(home_x, home_y, home_z, home_e);
	GCode gc;
	gc.setM(110);
	addCommand(gc);
	gc = GCode();
	gc.setM(111);
	gc.setS(7);
	addCommand(gc);
	gc = GCode();
	gc.setM(114);
	addCommand(gc);
	gc = GCode();
	gc.setM(84);
	addCommand(gc);
}

Printer::~Printer() {
	close(fd);
}

void
Printer::home() {
	GCode gc;
	gc.setG(28);
	addCommand(gc);
}

void
Printer::move(float x, float y, float z, float e, float f) {
	GCode gc;
#define upd(v) if(!isnan(v)) { this->v = v; }
	upd(x);
	upd(y);
	upd(z);
	upd(e);
	upd(f);
#undef upd
	gc.setMove(x, y, z, e, f);
	addCommand(gc);
}

void
Printer::moveRelative(float x, float y, float z, float e, float f) {
	move(this->x+x, this->y+y, this->z+z, this->e+e, f);
}

void
Printer::setHome(float x, float y, float z, float home_e) {
	home_x = x;
	home_y = y;
	home_z = z;
	home_e = e;
}

void 
Printer::addCommand(GCode &gc, bool front) {
	std::unique_lock<std::mutex> writeLock(writeMutex);
	if(front) {
		commandQueue.push_front(gc);
	} else {
		commandQueue.push_back(gc);
	}
	signalNextCommand.notify_one();
}

bool
Printer::serialStartsWith(const char *x) {
	if(!strncmp(serialReadBuffer, x, strlen(x))) {
		return true;
	} else {
		return false;
	}
}

void
Printer::serialParse() {
	if(serialStartsWith(START)) {
		started = true;
		signalNextCommand.notify_all();
	} else if(started) {
		if(serialStartsWith(OKCRLF)) {
			confirmedLineNumber++;
			commandQueue.pop_front();
			signalNextCommand.notify_all();
		} else if(serialStartsWith(OKNUM)) {
			if(atoi(serialReadBuffer + strlen(OKNUM))
			   == lineNumber) {
				confirmedLineNumber = lineNumber;
				commandQueue.pop_front();
				signalNextCommand.notify_all();
			}
		} else if(serialStartsWith(RESEND)) {
			if(atoi(serialReadBuffer + strlen(RESEND))
			   == lineNumber - 1) {
				// resend the last one
				lineNumber = confirmedLineNumber;
				signalNextCommand.notify_all();
			} else {
				// just reset the line numbers.
				GCode gc;
				gc.setM(110);
				lineNumber = -1;
				confirmedLineNumber = -1;
				commandQueue.push_front(gc);
				signalNextCommand.notify_all();
			}
		}
	}
}

void
Printer::serialRead() {
	struct timeval tv;
	fd_set rfd;
	while(running) {
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		
		FD_ZERO(&rfd);
		FD_SET(fd, &rfd);
	
		select(fd+1, &rfd, NULL, NULL, &tv);
		if(FD_ISSET(fd, &rfd)) {
			if(serialReadNum == 128) {
				// Screw it
				serialReadNum = 0;
			}
			writeMutex.lock();
			fgets(serialReadBuffer + serialReadNum,
				  128 - serialReadNum,
				  file);
			serialReadNum = strlen(serialReadBuffer);
			if(serialReadBuffer[serialReadNum - 1] == '\n') {
				serialReadNum = 0;
				serialParse();
			}
			writeMutex.unlock();
		}
	}
}

void
Printer::serialWrite() {
	while(running) {
		std::unique_lock<std::mutex> writeLock(writeMutex);
		signalNextCommand.wait(writeLock);
		if(!running) {
			break;
		}
		if(started
		   && lineNumber == confirmedLineNumber
		   && commandQueue.size() > 0) {
			GCode gc = commandQueue.front();
			uint8_t len;
			
			gc.setN(++lineNumber);
			
			const uint8_t *bgc = gc.getBuffer(&len);
			for(int i = 0; i < len; i++) {
				write(fd, bgc + i, 1);
			}
			signalFinishedCommand.notify_one();
		}
	}
}

void
Printer::wait() {
	std::unique_lock<std::mutex> writeLock(writeMutex);
	while(commandQueue.size() > 0) {
		signalNextCommand.wait(writeLock);
	}
}
