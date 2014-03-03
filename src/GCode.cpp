#include "GCode.h"
#include <cstring>

GCode::GCode(int32_t lineNumber)
	: flag(0),
	  bufferValid(false)
{
	if(lineNumber > 0) {
		setN(lineNumber);
	}
}

GCode::~GCode() { }

void
GCode::setN(int16_t n) {
	this->flag |= FLAG_N;
	this->n = n;
	this->bufferValid = false;
}

void
GCode::setM(uint8_t m) {
	this->flag |= FLAG_M;
	this->m = m;
	this->bufferValid = false;
}

void
GCode::setG(uint8_t g) {
	this->flag |= FLAG_G;
	this->g = g;
	this->bufferValid = false;
}

void
GCode::setX(float x) {
	if(!isnan(x)) {
		this->flag |= FLAG_X;
		this->x = x;
	} else {
		this->flag &= ~FLAG_X;
	}
	this->bufferValid = false;
}

void
GCode::setY(float y) {
	if(!isnan(y)) {
		this->flag |= FLAG_Y;
		this->y = y;
	} else {
		this->flag &= ~FLAG_Y;
	}
	this->bufferValid = false;
}

void
GCode::setZ(float z) {
	if(!isnan(z)) {
		this->flag |= FLAG_Z;
		this->z = z;
	} else {
		this->flag &= ~FLAG_Z;
	}
	this->bufferValid = false;
}

void
GCode::setE(float e) {
	if(!isnan(e)) {
		this->flag |= FLAG_E;
		this->e = e;
	} else {
		this->flag &= ~FLAG_E;
	}
	this->bufferValid = false;
}

void
GCode::setF(float f) {
	if(!isnan(f)) {
		this->flag |= FLAG_F;
		this->f = f;
	} else {
		this->flag &= ~FLAG_F;
	}
	this->bufferValid = false;
}

void
GCode::setXYZ(float x, float y, float z) {
	setX(x);
	setY(y);
	setZ(z);
}

void
GCode::setMove(float x,
			   float y,
			   float z,
			   float e,
			   float f) {
	setG(1);
	setXYZ(x, y, z);
	setE(e);
	setF(f);
}

void
GCode::setT(int8_t t) {
	this->flag |= FLAG_T;
	this->t = t;
	this->bufferValid = false;
}

void
GCode::setS(int32_t s) {
	this->flag |= FLAG_S;
	this->s = s;
	this->bufferValid = false;
}

void
GCode::setP(int32_t p) {
	this->flag |= FLAG_P;
	this->p = p;
	this->bufferValid = false;
}

const uint8_t *
GCode::getBuffer(uint8_t *lengthPtr) {
	updateBuffer();
	if(lengthPtr != NULL) {
		*lengthPtr = length;
	}
	return buffer;
}

uint8_t
GCode::getBufferLength() {
	updateBuffer();
	return length;
}

void
GCode::updateBuffer() {
	if(!bufferValid) {
		*(uint16_t*)(buffer) = flag | FLAG_NOTASCII;
		length = 2;
#define check(X, x, l) if(flag & X) { memcpy(buffer + length, &x, l); length += l; }
		check(FLAG_N, n, 2);
		check(FLAG_M, m, 1);
		check(FLAG_G, g, 1);
		check(FLAG_X, x, 4);
		check(FLAG_Y, y, 4);
		check(FLAG_Z, z, 4);
		check(FLAG_E, e, 4);
		check(FLAG_F, f, 4);
		check(FLAG_T, t, 1);
		check(FLAG_S, s, 4);
		check(FLAG_P, p, 4);
#undef check
		int16_t fl1 = 0,
			fl2 = 0;
		for(int i = 0; i < length; i++) {
			fl1 = (fl1 + buffer[i]) % 255;
			fl2 = (fl2 + fl1) % 255;
		}
		buffer[length++] = fl1 & 0xFF;
		buffer[length++] = fl2 & 0xFF;
		bufferValid = true;
	}
}
