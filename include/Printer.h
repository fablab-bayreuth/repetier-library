#pragma once
#include <stdint.h>
#include <math.h>

class Printer {
public:
	Printer(const char *file,
			int baudRate,
			int protocol);
	Printer(const char *file,
			int baudRate,
			int protocol,
			float home_x,
			float home_y,
			float home_z,
			float home_e);
	~Printer();

	void home();
	void move(float x, float y, float z, float e = NAN, float f = NAN);
	void moveRelative(float x, float y, float z, float e = NAN, float f = NAN);
	void setHome(float x, float y, float z, float e);
	
private:
	
	int fd;
	int protocol;
	int baudRate;
	float x, y, z, e, f;
	float home_x, home_y, home_z, home_e;

	enum GCodeFlags {
		N				= 1<< 0,
		M				= 1<< 1,
		G				= 1<< 2,
		X				= 1<< 3,
		Y				= 1<< 4,
		Z				= 1<< 5,
		E				= 1<< 6,
		NOTASCII		= 1<< 7,
		F				= 1<< 8,
		T				= 1<< 9,
		S				= 1<<10,
		P				= 1<<11,
		V2				= 1<<12,
		EXT				= 1<<13,
		INT				= 1<<14,
		TXT				= 1<<15
	};

	void sendBinaryCode(uint16_t flag,
						int16_t n,
						uint8_t m,
						uint8_t g,
						float x, float y, float z,
						float e,
						float f,
						int8_t t,
						int32_t s,
						int32_t p);
						
};
