/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

#include "../SmingCore/Timer.h"

Timer::Timer()
{
}

Timer::~Timer()
{
	stop();
}

Timer& Timer::initializeMs(/*uint32_t*/ unsigned long milliseconds, InterruptCallback callback)
{
	setCallback(callback);
	setIntervalMs(milliseconds);
	return *this;	
}

Timer& Timer::initializeUs(/*uint32_t*/ unsigned long microseconds, InterruptCallback callback)
{
	setCallback(callback);
	setIntervalUs(microseconds);
	return *this;
}

Timer& Timer::initializeMs(/*uint32_t*/ unsigned long milliseconds, TimerDelegate delegateFunction)
{
	setCallback(delegateFunction);
	setIntervalMs(milliseconds);
	return *this;
}

Timer& Timer::initializeUs(/*uint32_t*/ unsigned long microseconds, TimerDelegate delegateFunction)
{
	setCallback(delegateFunction);
	setIntervalUs(microseconds);
	return *this;
}

void Timer::start(bool repeating)
{
	stop();

	if(interval == 0 || (!callback && !delegate_func)) return;

	timer = xTimerCreate(
                     (const signed char *)"timer",       /* Just a text name, not used by the RTOS kernel. */
                     (100),        /* The timer period in ticks. */
                     pdTRUE,       /* The timers will auto-reload themselves when they expire. */
                     (void *)this, /* Assign each timer a unique id */
                     processing    /* Each timer calls the same callback when it expires. */
  );

	// ets_timer_setfn(&timer, (os_timer_func_t *)processing, this);

	// if (interval > 10000)
	// 	ets_timer_arm_new(&timer, (uint32_t)(interval / 1000), repeating, 1); // msec
	// else
	// 	ets_timer_arm_new(&timer, (uint32_t)interval, repeating, 0); 		  // usec

}

void Timer::stop()
{
	if (!xTimerIsTimerActive(timer)) return;	
	xTimerStop(timer, 0);
}

void Timer::restart()
{
	stop();
	start();
}

bool Timer::isStarted()
{
	return xTimerIsTimerActive(timer);
}

uint32_t Timer::getIntervalUs()
{
	return (uint32_t)interval;
}

uint32_t Timer::getIntervalMs()
{
	return (uint32_t)interval / 1000;
}

void Timer::setIntervalUs(uint32_t microseconds)
{
	interval = microseconds;
	xTimerChangePeriod(timer, interval, 0);
	if (isStarted())
		restart();
}

void Timer::setIntervalMs(uint32_t milliseconds)
{
	setIntervalUs(((uint64_t)milliseconds) * 1000);
}

void Timer::setCallback(InterruptCallback interrupt)
{
	vPortEnterCritical(); //portDISABLE_INTERRUPTS(); //ETS_INTR_LOCK();
	callback = interrupt;
	delegate_func = nullptr;
	vPortExitCritical(); //portENABLE_INTERRUPTS(); //ETS_INTR_UNLOCK();

	if (!interrupt)
		stop();
}

void Timer::setCallback(TimerDelegate delegateFunction)
{
	vPortEnterCritical(); //portDISABLE_INTERRUPTS(); //ETS_INTR_LOCK();
	callback = nullptr;
	delegate_func = delegateFunction;
	vPortExitCritical(); //portENABLE_INTERRUPTS(); //ETS_INTR_UNLOCK();

	if (!delegateFunction)
		stop();
}

void Timer::processing(xTimerHandle tmr)
{
	//Timer *ptimer = (Timer*)arg;
	Timer *ptimer = (Timer *)pvTimerGetTimerID(tmr);

	if (ptimer == NULL) {
	   return;
	}
	else if (ptimer->callback) {
		ptimer->callback();
	}
	else if (ptimer->delegate_func) {
		ptimer->delegate_func();
	}
}
