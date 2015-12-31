/*
 * wallaby_p.hpp
 *
 *  Created on: Nov 2, 2015
 *      Author: Joshua Southerland
 */

#ifndef SRC_WALLABY_P_HPP_
#define SRC_WALLABY_P_HPP_

namespace Private
{

class Wallaby
{

public:
	static Wallaby * instance();

	unsigned char readRegister8b(unsigned char address);
	void writeRegister8b(unsigned char address, unsigned char value);

	unsigned short readRegister16b(unsigned char address);
	void writeRegister16b(unsigned char address, unsigned short value);

	unsigned int readRegister32b(unsigned char address);
	void writeRegister32b(unsigned char address, unsigned int value);

	// for efficient bulk / low-level access:
	void updateReadBuffer();
	const char * getReadBuffer();
	unsigned int getBufferSize();

	virtual ~Wallaby();

private:
	Wallaby();
	bool transfer();
	void clear_buffers();
	int spi_fd_;
	unsigned char * write_buffer_;
	unsigned char * read_buffer_;
	const unsigned int buffer_size_; // same size for read/write buffers

};

} /* namespace Private */

#endif /* SRC_WALLABY_P_HPP_ */
