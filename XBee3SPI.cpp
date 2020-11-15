/*
 * Author: Dmitri Smith
 * Last modified: 11/8/2020
 * 
 * Add-on to Andrew Rapp's XBee-Arduino Library to add support for SPI operations.
 * Note that SPI requires AP mode = 1 (non-escaped), whereas UART through XBee-Arduino requires AP=2 (escaped).
 * 
 * 
 */
#include "Xbee3SPI.h"
XBee3SPI::XBee3SPI() _response(XBeeResponse() {
	_pos = 0;
    _escape = false;
    _checksumTotal = 0;
    _nextFrameId = 0;
    _response.init();
    _response.setFrameData(_responseFrameData);
	//Thanks to Paul Stoffregen for Teensy support
	#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__MK20DX256__)
        _spi = &SPI1;
	#else
        _spi = &SPI;
	#endif
	_interruptPin = DEFAULT_SPI_INTERRUPT_PIN;
	_transmittingFlag = false;
}


void XBee3SPI::begin(SPIClass &spi, uint8_t interruptPin) {
    _spi = &spi;
	_interruptPin = interruptPin;
}

/**
 * transmits a XBeeRequest (TX packet) over SPI
 */
void XBee3SPI::transmit(XBeeRequest &request) {
	_transmittingFlag = true;
    transmitByte(START_BYTE, false);

	// transmit length
	uint8_t msbLen = ((request.getFrameDataLength() + 2) >> 8) & 0xff;
	uint8_t lsbLen = (request.getFrameDataLength() + 2) & 0xff;

	transmitByte(msbLen);
	transmitByte(lsbLen);

	// api id
	transmitByte(request.getApiId());
	transmitByte(request.getFrameId());

	uint8_t checksum = 0;

	// compute checksum, start at api id
	checksum+= request.getApiId();
	checksum+= request.getFrameId();

	for (int i = 0; i < request.getFrameDataLength(); i++) {
		transmitByte(request.getFrameData(i));
		checksum+= request.getFrameData(i);
	}

	// perform 2s complement
	checksum = 0xff - checksum;

	// transmit checksum
	transmitByte(checksum);
	if (digitalRead(_interruptPin) == LOW) {
		flush();
	}
	_transmittingFlag = false;
}

/**
 * Specify the spi port
 */
void XBee3SPI::setSPI(SPIClass &spi) {
	_spi = spi;
}

//Sends a byte to the XBee, supports half- and full-duplex
void XBee3SPI::transmitByte(uint8_t b) {
	if(digitalRead(_interruptPin) == LOW)
	{
		readIntoBackup(_spi->transfer(b));
	}
	else
	{
		_spi->transfer(b);
	}
	
}

void XBee3SPI::readPacket() {
	// reset previous response
	if (_response.isAvailable() || _response.isError()) {
		// discard previous packet and start over
		resetResponse();
	}

    while (digitalRead(_interruptPin) == LOW) {

        b = read();

        if (_pos > 0 && b == START_BYTE) {
        	// new packet start before previous packeted completed -- discard previous packet and start over
        	_response.setErrorCode(UNEXPECTED_START_BYTE);
        	return;
        }
		// checksum includes all bytes starting with api id
		if (_pos >= API_ID_INDEX) {
			_checksumTotal+= b;
		}

        switch(_pos) {
			case 0:
		        if (b == START_BYTE) {
		        	_pos++;
		        }

		        break;
			case 1:
				// length msb
				_response.setMsbLength(b);
				_pos++;

				break;
			case 2:
				// length lsb
				_response.setLsbLength(b);
				_pos++;

				break;
			case 3:
				_response.setApiId(b);
				_pos++;

				break;
			default:
				// starts at fifth byte

				if (_pos > MAX_FRAME_DATA_SIZE) {
					// exceed max size.  should never occur
					_response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
					return;
				}

				// check if we're at the end of the packet
				// packet length does not include start, length, or checksum bytes, so add 3
				if (_pos == (_response.getPacketLength() + 3)) {
					// verify checksum

					//std::cout << "read checksum " << static_cast<unsigned int>(b) << " at pos " << static_cast<unsigned int>(_pos) << std::endl;

					if ((_checksumTotal & 0xff) == 0xff) {
						_response.setChecksum(b);
						_response.setAvailable(true);

						_response.setErrorCode(NO_ERROR);
					} else {
						// checksum failed
						_response.setErrorCode(CHECKSUM_FAILURE);
					}

					// minus 4 because we start after start,msb,lsb,api and up to but not including checksum
					// e.g. if frame was one byte, _pos=4 would be the byte, pos=5 is the checksum, where end stop reading
					_response.setFrameLength(_pos - 4);

					// reset state vars
					_pos = 0;

					_checksumTotal = 0;

					return;
				} else {
					// add to packet array, starting with the fourth byte of the apiFrame
					_response.getFrameData()[_pos - 4] = b;
					_pos++;
				}
        }
    }
}

bool XBee3SPI::readPacket(int timeout) {

	if (timeout < 0) {
		return false;
	}

	unsigned long start = millis();

    while (int((millis() - start)) < timeout) {

     	readPacket();

     	if (getResponse().isAvailable()) {
     		return true;
     	} else if (getResponse().isError()) {
     		return false;
     	}
    }

    // timed out
    return false;
}

//Half-duplex read, returns byte from XBee
uint8_t XBee3SOI::read()
{
	return _spi->transfer(GARBAGE_DATA);
}

//For half-duplex reading data into backup buffer from radio
void XBee3SPI::readIntoBackup() {
	//Send garbage hex values while listening - DO NOT SEND 0x7E!!!!
	readIntoBackup(GARBAGE_DATA);
}

//Read portion of full-duplex data transfer into backup buffer
void XBee3SPI::readIntoBackup(uint8_t b) {
	if(_backupPos < MAX_FRAME_SIZE) {
		_backupFrameBuffer[_backupPos] = _spi->transfer(b);
		_backupPos++;
	}
}

XBeeResponse& XBee3SPI::getResponse() {
	return _response;
}

bool XBee3SPI::available() {
	return (!_transmittingFlag && digitalRead(_interruptPin) == HIGH);
}

void XBee3SPI::resetResponse() {
	_pos = 0;
	_response.reset();
}

void XBee3SPI::flush() {
	while(_pos < MAX_FRAME_SIZE && digitalRead(_interruptPin) == LOW) {
		//Send garbage hex values while listening - DO NOT SEND 0x7E!!!!
		responseFrameData[_pos] = _spi->transfer(GARBAGE_DATA);
		_pos++;
	}
}

