/*! @file settings.c
**  Settings Manager
**  @author Robert Quattlebaum <darco@deepdarc.com>
**
**  Format based on OLPC manufacturing data, as described here:
**  <http://wiki.laptop.org/go/Manufacturing_data>
**
**  @par Features
**
**      - Robust data format which requires no initialization.
**      - Supports multiple values with the same key.
**      - Data can be appended without erasing EEPROM.
**      - Max size of settings data can be easily increased in the future,
**        as long as it doesn't overlap with application data.
**
**  @par Data Format
**  
**  Since the beginning of EEPROM often contains application-specific
**  information, the best place to store settings is at the end of
**  EEPROM. Because we are starting at the end of EEPROM, it makes sense
**  to grow the list of key-value pairs downward, toward the start of
**  EEPROM. 
**
**  Each key-value pair is stored in memory in the following format:
** <table>
**  <thead>
**   <td>Order</td>
**   <td>Size<small> (in bytes)</small></td>
**   <td>Name</td>
**   <td>Description</td>
**  </thead>
**  <tr>
**   <td>0</td>
**   <td>2</td>
**   <td>key</td>
**   <td></td>
**  </tr>
**  <tr>
**   <td>-2</td>
**   <td>1</td>
**   <td>size_check</td>
**   <td>One's-complement of next byte</td>
**  </tr>
**  <tr>
**   <td>-3</td>
**   <td>1 or 2</td>
**   <td>size</td>
**   <td>The size of the value, in bytes.</td>
**  </tr>
**  <tr>
**   <td>-4 or -5</td>
**   <td>variable</td>
**   <td>value</td>
**  </tr>
** </table>
**
**  The end of the key-value pairs is denoted by the first invalid entry.
**  An invalid entry has any of the following attributes:
**      - The size_check byte doesn't match the one's compliment
**        of the size byte (or size_low byte).
**      - The key has a value of 0x0000.
**
**  Note that we actually aren't starting at the very end of EEPROM, instead
**  we are starting 4 bytes from the end of EEPROM. This allows for things like
**  AVRDUDE's erase counter, and possibly bootloader flags.
**  
*/

#include <stdbool.h>
#include <avr/io.h>
#include "settings.h"

#include "contiki.h"
#include "dev/eeprom.h"

#include <stdio.h>

#if __AVR__
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#else
#define AVR_ENTER_CRITICAL_REGION()
#define AVR_LEAVE_CRITICAL_REGION()
#endif

#ifndef MIN
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#endif

#ifndef SETTINGS_TOP_ADDR
//! Defaults to end of EEPROM, minus 4 bytes for avrdude erase count
#define SETTINGS_TOP_ADDR	(settings_iter_t)(E2END-4)	
#endif

#ifndef SETTINGS_MAX_SIZE
#define SETTINGS_MAX_SIZE	(1024)	//!< Defaults to 1KB
#endif

/** This macro will protect the following code from interrupts.*/
#define AVR_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )

/** This macro must always be used in conjunction with AVR_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

typedef struct {
	uint8_t size_extra;
	uint8_t size_low;
	uint8_t size_check;
	settings_key_t key;
} item_header_t;

#pragma mark - Public Travesal Functions

settings_iter_t
settings_iter_begin() {
	return settings_iter_is_valid(SETTINGS_TOP_ADDR)?SETTINGS_TOP_ADDR:0;
}

settings_iter_t
settings_iter_next(settings_iter_t ret) {
    if(ret) {
        ret = settings_iter_get_value_addr(ret)-1;
        return settings_iter_is_valid(ret)?ret:0;
    }
    return SETTINGS_INVALID_ITER;
}

bool
settings_iter_is_valid(settings_iter_t item_addr) {
	item_header_t header = {};

	if(item_addr==EEPROM_NULL)
		return false;

	if((SETTINGS_TOP_ADDR-item_addr)>=SETTINGS_MAX_SIZE-3)
		return false;
	
	eeprom_read(
		item_addr+1-sizeof(header),
		(unsigned char*)&header,
		sizeof(header)
	);
	
	if((uint8_t)header.size_check!=(uint8_t)~header.size_low)
		return false;

	// TODO: Check length as well

	return true;
}

settings_key_t
settings_iter_get_key(settings_iter_t item_addr) {
	item_header_t header;
	
	eeprom_read(
		item_addr+1-sizeof(header),
		(unsigned char*)&header,
		sizeof(header)
	);
	
	if((uint8_t)header.size_check!=(uint8_t)~header.size_low)
		return SETTINGS_INVALID_KEY;

	return header.key;
}

settings_length_t
settings_iter_get_value_length(settings_iter_t item_addr) {
	item_header_t header;
	settings_length_t ret = 0;
	
	eeprom_read(
		item_addr+1-sizeof(header),
		(unsigned char*)&header,
		sizeof(header)
	);
	
	if((uint8_t)header.size_check!=(uint8_t)~header.size_low)
		goto bail;
	
	ret = header.size_low;
	
	if(ret&(1<<7)) {
		ret = ((ret&~(1<<7))<<8) | header.size_extra;
	}

bail:
	return ret;
}

eeprom_addr_t
settings_iter_get_value_addr(settings_iter_t item_addr) {
	settings_length_t len = settings_iter_get_value_length(item_addr);
	
	return item_addr+1-sizeof(item_header_t)-len + (len>128);
}

void
settings_iter_get_value_bytes(settings_iter_t item_addr, void* bytes) {
    // TODO: Writeme!
    return 0;
}

#pragma mark - Public Functions

bool
settings_check(settings_key_t key,uint8_t index) {
	bool ret = false;
	settings_iter_t current_item = SETTINGS_TOP_ADDR;

	for(current_item=SETTINGS_TOP_ADDR;settings_iter_is_valid(current_item);current_item=settings_iter_next(current_item)) {
		if(settings_iter_get_key(current_item)==key) {
			if(!index) {
				ret = true;
				break;
			} else {
				// Nope, keep looking
				index--;
			}
		}
	}

	return ret;
}

settings_status_t
settings_get(settings_key_t key,uint8_t index,unsigned char* value,settings_length_t* value_size) {
	settings_status_t ret = SETTINGS_STATUS_NOT_FOUND;
	settings_iter_t current_item = SETTINGS_TOP_ADDR;
	
	for(current_item=settings_iter_begin();current_item;current_item=settings_iter_next(current_item)) {
		if(settings_iter_get_key(current_item)==key) {
			if(!index) {
				// We found it!
				*value_size = MIN(*value_size,settings_iter_get_value_length(current_item));
				eeprom_read(
					settings_iter_get_value_addr(current_item),
					value,
					*value_size
				);
				ret = SETTINGS_STATUS_OK;
				break;
			} else {
				// Nope, keep looking
				index--;
			}
		}
	}

	return ret;
}

settings_status_t
settings_add(settings_key_t key,const unsigned char* value,settings_length_t value_size) {
	settings_status_t ret = SETTINGS_STATUS_FAILURE;
	settings_iter_t current_item = SETTINGS_TOP_ADDR;
	item_header_t header;
	
	// Find end of list
	for(current_item=settings_iter_begin();current_item;current_item=settings_iter_next(current_item)) { }
	
	if(current_item==EEPROM_NULL)
		goto bail;

	// TODO: size check!

	header.key = key;

	if(value_size<0x80) {
		// If the value size is less than 128, then
		// we can get away with only using one byte
		// as the size.
		header.size_low = value_size;
	} else if(value_size<=SETTINGS_MAX_VALUE_SIZE) {
		// If the value size of larger than 128,
		// then we need to use two bytes. Store
		// the most significant 7 bits in the first
		// size byte (with MSB set) and store the
		// least significant bits in the second
		// byte (with LSB clear)
		header.size_low = (value_size>>7) | 0x80;		
		header.size_extra = value_size & ~0x80;
	} else {
		// Value size way too big!
		goto bail;
	}

	header.size_check = ~header.size_low;

	// Write the header first
	eeprom_write(
		current_item+1-sizeof(header),
		(unsigned char*)&header,
		sizeof(header)
	);
	
	// Sanity check, remove once confident
	if(settings_iter_get_value_length(current_item)!=value_size) {
		goto bail;
	}
	
	// Now write the data
	eeprom_write(
		settings_iter_get_value_addr(current_item),
		(unsigned char*)value,
		value_size
	);
	
	ret = SETTINGS_STATUS_OK;
	
bail:
	return ret;
}

settings_status_t
settings_set(settings_key_t key,const unsigned char* value,settings_length_t value_size) {
	settings_status_t ret = SETTINGS_STATUS_FAILURE;
	settings_iter_t current_item = SETTINGS_TOP_ADDR;

	for(current_item=settings_iter_begin();current_item;current_item=settings_iter_next(current_item)) {
		if(settings_iter_get_key(current_item)==key) {
			break;
		}
	}

	if((current_item==EEPROM_NULL) || !settings_iter_is_valid(current_item)) {
		ret = settings_add(key,value,value_size);
		goto bail;
	}
	
	if(value_size!=settings_iter_get_value_length(current_item)) {
		// Requires the settings store to be shifted. Currently unimplemented.
		ret = SETTINGS_STATUS_UNIMPLEMENTED;
		goto bail;
	}
	
	// Now write the data
	eeprom_write(
		settings_iter_get_value_addr(current_item),
		(unsigned char*)value,
		value_size
	);

	ret = SETTINGS_STATUS_OK;
	
bail:
	return ret;
}

settings_status_t
settings_delete(settings_key_t key,uint8_t index) {
	// Requires the settings store to be shifted. Currently unimplemented.
	// TODO: Writeme!
	return SETTINGS_STATUS_UNIMPLEMENTED;
}


void
settings_wipe(void) {
	settings_length_t i = SETTINGS_TOP_ADDR-SETTINGS_MAX_SIZE;
	AVR_ENTER_CRITICAL_REGION();
	for(;i<=SETTINGS_TOP_ADDR;i++) {
		eeprom_write_byte((uint8_t*)i,0xFF);
		wdt_reset();
	}
	AVR_LEAVE_CRITICAL_REGION();
}


