/*
 * Author: Dmitri Smith
 * Last modified: 11/8/2020
 * 
 * Add-on to Andrew Rapp's XBee-Arduino Library to add support for SPI operations.
 * Currently only runs half-duplex
 * 
 * TODO: Support for full-duplex operations
 * 
 */

#ifndef XBee3SPI_h
#define XBee3SPI_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>

#include "XBee.h"
#include "SPI.h"


/* This value determines the size of the byte array for receiving RX packets
 * Most users won't be dealing with packets this large so you can adjust this
 * value to reduce memory consumption. But, remember that
 * if a RX packet exceeds this size, it cannot be parsed!
 */
// This value is determined by the largest packet size (256 byte payload + 64-bit address + option byte and rssi byte) of a series 3 radio
#define MAX_FRAME_SIZE 266

/**
 * DON'T USE THIS! Need a default pin value for the interrupt pin. Use begin() or setInterruptPin() to set this to the proper pin!
 */
#define DEFAULT_SPI_INTERRUPT_PIN 8

//Data to transfer when we want half-duplex operation
#define GARBAGE_DATA 0xff

class XBee3SPI {
public:
	XBee3SPI();
	/**
	 * Reads all available serial bytes until a packet is parsed, an error occurs, or the buffer is empty.
	 * You may call <i>xbee</i>.getResponse().isAvailable() after calling this method to determine if
	 * a packet is ready, or <i>xbee</i>.getResponse().isError() to determine if
	 * a error occurred.
	 * <p/>
	 * This method should always return quickly since it does not wait for serial data to arrive.
	 * You will want to use this method if you are doing other timely stuff in your loop, where
	 * a delay would cause problems.
	 * NOTE: calling this method resets the current response, so make sure you first consume the
	 * current response
	 */
	void readPacket();
	/**
	 * Waits a maximum of <i>timeout</i> milliseconds for a response packet before timing out; returns true if packet is read.
	 * Returns false if timeout or error occurs.
	 */
	bool readPacket(int timeout);
	/*
    * Sets the SPI port to use.
    * Named begin to match naming convention of Andrewrapp's XBee library
    */
    void begin(SPIClass &spi, unint8_t interruptPin);

    /** TODO: Move this into XBee.h XBeeResponse class!
     * Returns a pointer to a clone of the currently stored response
     */
	//XBeeResponse cloneResponse(XBeeResponse &response);
	/**
	 * Returns a reference to the current response
	 * Note: once readPacket is called again this response will be overwritten!
	 */
	XBeeResponse& getResponse();
	/**
     * transmits a XBeeRequest (TX packet) out the spi port
     */
    void transmit(XBeeRequest &request);

    /**
     * Specify the SPI port
     */
    void setSPI(SPIClass &spi);
    void setInterruptPin(uint8_t pin);
private:
	bool available();
    uint8_t read();
    /**
     * Half-duplex parsing of a received message frame
     * Intended for when slave begins transmitting in the middle of a master transmission
     */
	void readIntoBackup();
    /**
     * Full-duplex parsing of a frame
     * Intended for when slave begins transmitting in the middle of a master transmission
     */ 
    void readIntoBackup(uint8_t);

    //Used to complete data transmission if slave begins sending data while master is mid-transmission
	void flush();

    /**
     * Sends data byte in half-duplex, or full-duplex if _inpterruptPin is LOW
     */
	void transmitByte(uint8_t b);

    /**
     * Clear the received response frame so that a new one can be parsed
     */
	void resetResponse();
	XBeeResponse _response;
	// current packet position for response.  just a state variable for packet parsing and has no relevance for the response otherwise
	uint8_t _pos;
	// last byte read
	uint8_t b;
	uint8_t _checksumTotal;
	uint8_t _nextFrameId;
	// buffer for incoming RX packets.  holds only the api specific frame data, starting after the api id byte and prior to checksum
	uint8_t _responseFrameData[MAX_FRAME_DATA_SIZE];
	SPIClass* _spi;
    //Interrupt pin from XBee; HIGH indicates that XBee has data to transmit
    uint8_t _interruptPin;
    //Set when transmission is in progress
    //Set to true when master is sending data to XBee
    bool _transmittingFlag;
    //Backup buffer in case message xbee sends data mid-transmit
    uint8_t _backupFrameBuffer[MAX_FRAME_DATA_SIZE];
    uint8_t _backupPos;
};

#endif