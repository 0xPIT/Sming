#include <user_config.h>
#include <SmingCore/SmingCore.h>

#define LED_PIN 2 // GPIO2

Timer procTimer;
bool state = true;

void blink()
{
	digitalWrite(LED_PIN, state);
	state = !state;
}

void init()
{
	pinMode(LED_PIN, OUTPUT);
  unsigned int t = 1000;
	procTimer.initializeMs(t, blink).start();
}
