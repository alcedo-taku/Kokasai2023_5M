/*
 * xbee.hpp
 *
 *  Created on: Aug 27, 2020
 *      Author: Satoshi Ohya
 */

#ifndef INC_XBEE_HPP_
#define INC_XBEE_HPP_

#include <stdint.h>
#include <array>

#include "usart.h"



template<unsigned Size = 30>
class xbee_c{
public:
	xbee_c(){
		buf[0] = 0x7E;
	}

	template <typename T> void assemblyTransmitPacket(const T& data){
		this->bufSize = 18+sizeof(T);
		buf[1] = (sizeof(T) + 14) >> 8;
		buf[2] = (sizeof(T) + 14) & 0xFF;
		buf[3] = 0x10;
		buf[4] = 0x01;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;
		buf[11] = 0xFF;
		buf[12] = 0xFF;
		buf[13] = 0xFF;
		buf[14] = 0xFE;
		buf[15] = 0x00;
		buf[16] = 0x00;
		for(uint8_t n=0; n<sizeof(T); n++){
			buf[n+17] = *((uint8_t *)&data + n);
		}
		this->checkSum(this->bufSize);
	}

	template <typename T> void assemblyTransmitPacket(const T& data, const uint16_t& address){
		this->assemblyTransmitPacket(data);
		buf[13] = address >> 8;
		buf[14] = address & 0xFF;
		this->checkSum(this->bufSize);
	}

	template <typename T> void assemblyTransmitPacket(const T& data, const uint64_t& address){
		this->assemblyTransmitPacket(data);
		for(uint8_t n=0; n<8; n++){
			buf[n+5] = address >> (8*(7-n));
		}
		this->checkSum(this->bufSize);
	}

	uint8_t *getBufAddress(){
		return &this->buf[0];
	}

	uint8_t getBufSize(){
		return this->bufSize;
	}

	bool isSuccessTransmitStatus(std::array<uint8_t, 11> transmitStatus){
		return buf[8] == 0x00;
	}

	bool isSuccessTransmitStatus(uint8_t *transmitStatus){
		return *(transmitStatus + 8) == 0x00;
	}


	template <typename T> T diassemblyReceivePacket(uint8_t *buf, uint16_t bufLength){
		T tmp;
		for(uint8_t n=0; n<sizeof(T); n++){
			*((uint8_t *)&tmp+n) = *(buf + n + 15);
		}
		return tmp;
	}

	constexpr uint8_t getReceivePacketSize(){
		return Size-2;
	}

	constexpr uint8_t getTransmitStatusPacketSize(){
		return 11;
	}

private:
	UART_HandleTypeDef *huart;
	std::array<uint8_t, Size> buf={};
	uint8_t bufSize;
	void checkSum(uint8_t length){
		uint32_t sum=0;
		for(uint8_t n=0; n<length-4; n++){
			sum += buf[n+3];
		}
		 buf[length-1] = 0xFF - (sum & 0xFF);
	}


};

#endif /* INC_XBEE_HPP_ */

