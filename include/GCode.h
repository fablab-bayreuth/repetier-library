#include <cstddef>
#include <stdint.h>
#include <cmath>

class GCode {
 public:
	GCode(int32_t lineNumber = -1);
	~GCode();

	void setN(int16_t n);
	void setM(uint8_t m);
	void setG(uint8_t g);
	void setX(float x);
	void setY(float y);
	void setZ(float z);
	void setXYZ(float x = NAN,
				float y = NAN,
				float z = NAN);
	void setE(float e);
	void setF(float f);
	void setMove(float x = NAN,
				 float y = NAN,
				 float z = NAN,
				 float e = NAN,
				 float f = NAN);
	void setT(int8_t t);
	void setS(int32_t s);
	void setP(int32_t p);

	const uint8_t *getBuffer(uint8_t *length = NULL);
	uint8_t getBufferLength();

	enum GCodeFlags {
		FLAG_N			= 1<< 0,
		FLAG_M			= 1<< 1,
		FLAG_G			= 1<< 2,
		FLAG_X			= 1<< 3,
		FLAG_Y			= 1<< 4,
		FLAG_Z			= 1<< 5,
		FLAG_E			= 1<< 6,
		FLAG_NOTASCII	= 1<< 7,
		FLAG_F			= 1<< 8,
		FLAG_T			= 1<< 9,
		FLAG_S			= 1<<10,
		FLAG_P			= 1<<11,
		FLAG_V2			= 1<<12,
		FLAG_EXT		= 1<<13,
		FLAG_INT		= 1<<14,
		FLAG_TXT		= 1<<15
	};

 private:
	uint8_t buffer[64];
	uint8_t length;
	bool bufferValid;
	
	uint16_t flag;
	uint16_t n;
	uint8_t m, g;
	float x, y, z,
		e,
		f;
	int8_t t;
	int32_t s;
	int32_t p;

	void updateBuffer();
};
