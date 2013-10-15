/* Name: main.c
 * Project: hid-custom-rq example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: main.c 692 2008-11-07 15:07:40Z cs $
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
We assume that an LED is connected to port B bit 0. If you connect it to a
different port or bit, change the macros below:
*/
#define LED_PORT_DDR        DDRB
#define LED_PORT_OUTPUT     PORTB
#define R_BIT            4
#define G_BIT            3
#define B_BIT            1
#define BUTTON_PIN 4

#include <stdint.h>
#include <string.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "requests.h"       /* The custom request numbers we use */
#include "special_functions.h"

void update_pwm(void);

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */
const PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_A       4
#define KEY_B       5
#define KEY_C       6
#define KEY_D       7
#define KEY_E       8
#define KEY_F       9
#define KEY_G       10
#define KEY_H       11
#define KEY_I       12
#define KEY_J       13
#define KEY_K       14
#define KEY_L       15
#define KEY_M       16
#define KEY_N       17
#define KEY_O       18
#define KEY_P       19
#define KEY_Q       20
#define KEY_R       21
#define KEY_S       22
#define KEY_T       23
#define KEY_U       24
#define KEY_V       25
#define KEY_W       26
#define KEY_X       27
#define KEY_Y       28
#define KEY_Z       29
#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39

#define KEY_F1      58
#define KEY_F2      59
#define KEY_F3      60
#define KEY_F4      61
#define KEY_F5      62
#define KEY_F6      63
#define KEY_F7      64
#define KEY_F8      65
#define KEY_F9      66
#define KEY_F10     67
#define KEY_F11     68
#define KEY_F12     69

union {
	struct {
		uint16_t red;
		uint16_t green;
		uint16_t blue;
	} name;
	uint16_t idx[3];
} color;

#define UNI_BUFFER_SIZE 16

static union {
	uint8_t  w8[UNI_BUFFER_SIZE];
	uint16_t w16[UNI_BUFFER_SIZE/2];
	uint32_t w32[UNI_BUFFER_SIZE/4];
	void*    ptr[UNI_BUFFER_SIZE/sizeof(void*)];
} uni_buffer;

static uint8_t uni_buffer_fill;
static uint8_t current_command;
/* ------------------------------------------------------------------------- */


uint8_t read_button(void){
	uint8_t t,u,v=0;
	t = DDRB;
	u = PORTB;
	DDRB &= ~(1<<BUTTON_PIN);
	PORTB |= 1<<BUTTON_PIN;
	PORTB &= ~(1<<BUTTON_PIN);
	v |= PINB;
	DDRB |= t&(1<<BUTTON_PIN);
	PORTB &= ~(t&(1<<BUTTON_PIN));
	v >>= BUTTON_PIN;
	v &= 1;
	v ^= 1;
	return v;
}

void init_tmpsensor(void){
	ADMUX = 0x8F;
	ADCSRA = 0x87;
}

uint16_t read_tmpsensor(void){
	ADCSRA |= 0x40;
	while(ADCSRA & 0x40)
		;
	return ADC;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (usbRequest_t *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
	{
		current_command = rq->bRequest;
    	switch(rq->bRequest)
		{
		case CUSTOM_RQ_SET_RED:
			color.name.red = rq->wValue.bytes[0];
			break;	
		case CUSTOM_RQ_SET_GREEN:
			color.name.green = rq->wValue.bytes[0];
			break;	
		case CUSTOM_RQ_SET_BLUE:
			color.name.blue = rq->wValue.bytes[0];
			break;	
		case CUSTOM_RQ_SET_RGB:
			return USB_NO_MSG;
		case CUSTOM_RQ_GET_RGB:{
			usbMsgLen_t len=6;
			if(len>rq->wLength.word){
				len = rq->wLength.word;
			}
			usbMsgPtr = (uchar*)color.idx;
			return len;
		}
		case CUSTOM_RQ_READ_MEM:
			usbMsgPtr = (uchar*)rq->wValue.word;
			return rq->wLength.word;
		case CUSTOM_RQ_WRITE_MEM:
		case CUSTOM_RQ_EXEC_SPM:
			uni_buffer_fill = 4;
			uni_buffer.w16[0] = rq->wValue.word;
			uni_buffer.w16[1] = rq->wLength.word;
			return USB_NO_MSG;
		case CUSTOM_RQ_READ_FLASH:
			uni_buffer.w16[0] = rq->wValue.word;
			uni_buffer.w16[1] = rq->wLength.word;
			return USB_NO_MSG;
		case CUSTOM_RQ_RESET:
			soft_reset((uint8_t)(rq->wValue.word));
			break;
		case CUSTOM_RQ_READ_BUTTON:
			uni_buffer.w8[0] = read_button();
			usbMsgPtr = uni_buffer.w8;
			return 1;
		case CUSTOM_RQ_READ_TMPSENS:
			uni_buffer.w16[0] = read_tmpsensor();
			usbMsgPtr = uni_buffer.w8;
			return 2;
		}
    }
	else
	{
        /* calls requests USBRQ_HID_GET_REPORT and USBRQ_HID_SET_REPORT are
         * not implemented since we never call them. The operating system
         * won't call them either because our descriptor defines no meaning.
         */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
	switch(current_command){
	case CUSTOM_RQ_SET_RGB:
		if(len!=6){
			return 1;
		}
		memcpy(color.idx, data, 6);
		return 1;
	case CUSTOM_RQ_WRITE_MEM:
		memcpy(uni_buffer.ptr[0], data, len);
		uni_buffer.w16[0] += len;
		return !(uni_buffer.w16[1] -= len);
	case CUSTOM_RQ_EXEC_SPM:
		if(uni_buffer_fill<8){
			uint8_t l = 8-uni_buffer_fill;
			if(len<l){
				len = l;
			}
			memcpy(&(uni_buffer.w8[uni_buffer_fill]), data, len);
			uni_buffer_fill += len;
			return 0;
		}
		uni_buffer.w16[1] -= len;
		if(uni_buffer.w16[1]>8){
			memcpy(uni_buffer.ptr[0], data, len);
			uni_buffer.w16[0] += len;
			return 0;
		}else{
			memcpy(&(uni_buffer.w8[uni_buffer_fill]), data, len);
			exec_spm(uni_buffer.w16[2], uni_buffer.w16[3], uni_buffer.ptr[0], data, len);
			return 1;
		}
	default:
		return 1;
	}
	return 0;
}
uchar usbFunctionRead(uchar *data, uchar len){
	uchar ret=len;
	switch(current_command){
	case CUSTOM_RQ_READ_FLASH:
		while(len--){
			*data++ = pgm_read_byte((uni_buffer.w16[0])++);
		}
		return ret;
	default:
		break;
	}
	return 0;
}

static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
 
    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if(x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
 

void usbEventResetReady(void)
{
    cli();  // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
    calibrateOscillator();
    sei();
// we never read the value from eeprom so this causes only degradation of eeprom
//    eeprom_write_byte(0, OSCCAL);   // store the calibrated value in EEPROM
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	uchar   i;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

    init_tmpsensor();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    LED_PORT_DDR |= _BV(R_BIT) | _BV(G_BIT) | _BV(B_BIT);   /* make the LED bit an output */
	
    sei();

    for(;;){                /* main event loop */
		update_pwm();
		
        wdt_reset();
        usbPoll();
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
