#pragma once
#include <stdint.h>
#include <math.h>

/**
 * \brief A class that allows communication with a printer using the repetier firmware.
 */
class Printer {
public:
	/**
	 * \brief Creates a printer using the specified tty, baudrate,
	 * protocol, and home position.
	 *
	 * After creation, the current coordinates are not yet available
	 * and relative movement is (currently) not possible. Home or move
	 * the printer to get access to relative movement.
	 *
	 * \param file The tty to use for communication. Usually something like "/dev/ttyACM0" or "/dev/ttyUSB0"
	 * \param baudRate The printer's baudrate. The default is 250 kbaud.
	 * \param protocol The communication protocol. 0 is ASCII, 1 and 2 are versions 1 and 2 of the binary protocol. At the moment, only 1 is supported and the parameter is unused.
	 * \param home_x The x coordinate after the printer has been homed.
	 * \param home_y The y coordinate after the printer has been homed.
	 * \param home_z The z coordinate after the printer has been homed.
	 * \param home_e The e coordinate after the printer has been homed.
	 */
	Printer(const char *file,
			int baudRate = 250000,
			int protocol = 1,
			float home_x = 0,
			float home_y = 0,
			float home_z = 0,
			float home_e = 0);
	~Printer();

	/**
	 * \brief Moves the printer to its home position
	 */
	void home();

	/**
	 * \brief Move the printer to a position.
	 * 
	 * If any parameter is set to NAN, it will be ignored. For the x,
	 * y, z, and e parameters this means that the printer will not
	 * move that axis. For the f parameter it means that it will use
	 * the currently active speed.
	 *
	 * Using this function enables relative movement with all the
	 * not-NAN axes.
	 *
	 * \param x The new x coordinate
	 * \param y The new y coordinate
	 * \param z The new z coordinate
	 * \param e The new e coordinate (extruder position)
	 * \param f The movement speed, usually in mm/minute. The printer will always move at the last used speed if this is set to NAN.
	 */
	void move(float x, float y, float z, float e = NAN, float f = NAN);

	/**
	 * \brief Move the printer relative to its current position.
	 *
	 * If any parameter is set to NAN, it will be ignored, just like with move()
	 *
	 * \param x The movement in the x axis
	 * \param y The movement in the y axis
	 * \param z The movement in the z axis
	 * \param e The movement in the extruder position
	 * \param f The movement speed, usually in mm/minute. The printer will always move at the last used speed if this is set to NAN.
	 */
	void moveRelative(float x, float y, float z, float e = NAN, float f = NAN);

	/**
	 * \brief Sets the home position of the printer.
	 *
	 * \param x The x coordinate of the home position
	 * \param y The y coordinate of the home position
	 * \param z The z coordinate of the home position
	 * \param e The e coordinate of the home position
	 */
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
