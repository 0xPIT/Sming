#include <user_config.h>
#include <esp_cplusplus.h>
#include "../SmingCore/SmingCore.h"

#define ESP8266_CLOCK 80000000UL
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define USD(u) ESP8266_REG(0x014+(0xF00*(u&1))) // CLKDIV

extern void init();

extern "C" void user_init(void)
{
	//system_timer_reinit();
	//uart_div_modify(UART_ID_0, UART_CLK_FREQ / 115200);
  USD(0) = (ESP8266_CLOCK / 115200);

	cpp_core_initialize();
	spiffs_mount();
	System.initialize();

	init(); // User code init
}

// For compatibility with SDK v1.1
extern "C" void __attribute__((weak)) user_rf_pre_init(void)
{
	// RTC startup fix, author pvvx
    volatile uint32 * ptr_reg_rtc_ram = (volatile uint32 *)0x60001000;
    if((ptr_reg_rtc_ram[24] >> 16) > 4) {
        ptr_reg_rtc_ram[24] &= 0xFFFF;
        ptr_reg_rtc_ram[30] &= 0;
    }
}
