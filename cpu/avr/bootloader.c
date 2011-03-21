#include "bootloader.h"
#include "dev/watchdog.h"
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "dev/usb/usb_drv.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

volatile uint32_t Boot_Key ATTR_NO_INIT;

bool
bootloader_is_present(void) {
	return pgm_read_word_far(BOOTLOADER_START_ADDRESS)!=0xFFFF;
}

void
Jump_To_Bootloader(void)
{	
	// Disable all interrupts
	cli();

#ifdef UDCON
	// If USB is used, detach from the bus
	Usb_detach();

	// Wait two seconds for the USB detachment to register on the host
	uint8_t i;
	for (i = 0; i < 20; i++) {
		_delay_ms(100);
		watchdog_periodic();
	}
#endif

	// Set the bootloader key to the magic value and force a reset
	Boot_Key = MAGIC_BOOT_KEY;
	
	watchdog_reboot();
}

extern void Bootloader_Jump_Check(void) ATTR_INIT_SECTION(3);

void
Bootloader_Jump_Check(void)
{
	// If the reset source was the bootloader and the key is correct, clear it and jump to the bootloader
	if(MCUSR & (1<<WDRF)) {
		MCUSR = 0;
		if(Boot_Key == MAGIC_BOOT_KEY) {
			Boot_Key = 0;
			wdt_disable();
			
			((void (*)(void))(BOOTLOADER_START_ADDRESS))();
		} else {
			// The watchdog fired. Probably means we
			// crashed. Wait two seconds before continuing.

			Boot_Key++;
			uint8_t i;
			for (i = 0; i < 200; i++) {
				_delay_ms(10);
				watchdog_periodic();
			}
		}
	} else {
		Boot_Key = MAGIC_BOOT_KEY-4;
	}
}
