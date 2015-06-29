/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#include "../Wiring/WiringFrameworkDependencies.h"
#include "../Wiring/Stream.h"
#include "../SmingCore/Delegate.h"

#define NUMBER_UARTS 2

// Delegate constructor usage: (&YourClass::method, this)
typedef Delegate<void(Stream &source, char arrivedChar, uint16_t availableCharsCount)> StreamDataReceivedDelegate;

typedef struct {
	StreamDataReceivedDelegate HWSDelegate;
	bool useRxBuff;
} HWSerialMemberData;

class HardwareSerial : public Stream
{
public:
	HardwareSerial(const int uartPort);
	~HardwareSerial() {}

	void begin(const uint32_t baud = 9600);

	int available();
	int read();
	int peek();
	size_t write(uint8_t oneChar);
	void flush();

  void getRxQueue(xQueueHandle *qh);

	void systemDebugOutput(bool enabled);

	void setCallback(StreamDataReceivedDelegate reqCallback, bool useSerialRxBuffer = true);
	void resetCallback();

private:
	static void rxInterruptHandler(void *parameter);
  friend void ::delegateTask(void* parameter);

public:
	const uint8_t uart;

private:
  UartConfigTypeDef uartConfig;
	xQueueHandle rxQueue;
	//xQueueHandle txQueue;
  xTaskHandle delegateTaskHandle;
	static HWSerialMemberData memberData[NUMBER_UARTS];

};

extern HardwareSerial Serial;

#endif /* _HARDWARESERIAL_H_ */
