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
#include <stdbool.h>

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
#include "hotp.h"
#include "percnt2.h"

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */
PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

uint16_t secret_length_ee EEMEM = 0;
uint8_t  secret_ee[32] EEMEM;
uint8_t  reset_counter_ee EEMEM = 0;
uint8_t  digits_ee EEMEM = 8;

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

#define NUM_LOCK 1
#define CAPS_LOCK 2
#define SCROLL_LOCK 4

static uint8_t dbg_buffer[8];

static uint8_t secret[32];
static uint16_t secret_length_b;
static char token[10];


#define UNI_BUFFER_SIZE 36

static union {
	uint8_t  w8[UNI_BUFFER_SIZE];
	uint16_t w16[UNI_BUFFER_SIZE/2];
	uint32_t w32[UNI_BUFFER_SIZE/4];
	void*    ptr[UNI_BUFFER_SIZE/sizeof(void*)];
} uni_buffer;

static uint8_t uni_buffer_fill;
static uint8_t current_command;

typedef struct {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} keyboard_report_t;

#define STATE_WAIT 0
#define STATE_SEND_KEY 1
#define STATE_RELEASE_KEY 2
#define STATE_NEXT 3


static keyboard_report_t keyboard_report; // sent to PC
static uchar idleRate;           /* in 4 ms units */
static uchar key_state = STATE_WAIT;
volatile static uchar LED_state = 0xff; // received from PC
/* ------------------------------------------------------------------------- */

void memory_clean(void) {
    memset(secret, 0, 32);
    secret_length_b = 0;
}

uint8_t secret_set(void){
    uint8_t r;
    union {
        uint8_t w8[32];
        uint16_t w16[16];
    } read_back;
    const uint8_t length_B = (secret_length_b + 7) / 8;

    eeprom_busy_wait();
    eeprom_write_block(secret, secret_ee, length_B);
    eeprom_busy_wait();
    eeprom_read_block(read_back.w8, secret_ee, length_B);
    r = memcmp(secret, read_back.w8, length_B);
    memory_clean();
    memset(read_back.w8, 0, 32);
    if (r) {
        return 1;
    }
    eeprom_busy_wait();
    eeprom_write_word(&secret_length_ee, secret_length_b);
    eeprom_busy_wait();
    r = eeprom_read_word(&secret_length_ee) == secret_length_b;
    memory_clean();
    *read_back.w16 = 0;
    if (!r) {
        return 1;
    }
    return 0;
}

void token_generate(void) {
    percnt_inc(0);
    eeprom_busy_wait();
    eeprom_read_block(secret, secret_ee, 32);
    eeprom_busy_wait();
    hotp(token, secret, eeprom_read_word(&secret_length_ee), percnt_get(0), eeprom_read_byte(&digits_ee));
    memory_clean();
}

void counter_reset(void) {
    uint8_t reset_counter;
    eeprom_busy_wait();
    reset_counter = eeprom_read_byte(&reset_counter_ee);
    percnt_reset(0);
    eeprom_busy_wait();
    eeprom_write_byte(&reset_counter_ee, reset_counter + 1);
}

void counter_init(void) {
    eeprom_busy_wait();
    if (eeprom_read_byte(&reset_counter_ee) == 0) {
        counter_reset();
    }
    percnt_init(0);
}

void buildReport(uchar send_key) {
    keyboard_report.modifier = 0;

    switch (send_key) {
    case 'A' ... 'Z':
        keyboard_report.modifier = MOD_SHIFT_LEFT;
        keyboard_report.keycode[0] = KEY_A + (send_key-'A');
        break;
    case 'a' ... 'z':
        keyboard_report.keycode[0] = KEY_A + (send_key-'a');
        break;
    case '1' ... '9':
        keyboard_report.keycode[0] = KEY_1 + (send_key-'1');
        break;
    case '0':
        keyboard_report.keycode[0] = KEY_0;
        break;
    default:
        keyboard_report.keycode[0] = 0;
    }
}

uint8_t read_button(void){
	uint8_t t,v=0;
	t = DDRB;
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

void init_temperature_sensor(void){
	ADMUX = 0x8F;
	ADCSRA = 0x87;
}

uint16_t read_temperture_sensor(void){
	ADCSRA |= 0x40;
	while(ADCSRA & 0x40)
		;
	return ADC;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (usbRequest_t *)data;
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {    /* class request type */
	    switch(rq->bRequest) {
        case USBRQ_HID_GET_REPORT: // send "no keys pressed" if asked here
            // wValue: ReportType (highbyte), ReportID (lowbyte)
            usbMsgPtr = (void *)&keyboard_report; // we only have this one
            keyboard_report.modifier = 0;
            keyboard_report.keycode[0] = 0;
            return sizeof(keyboard_report);
        case USBRQ_HID_SET_REPORT: // if wLength == 1, should be LED state
            if (rq->wLength.word == 1) {
                current_command = LED_WRITE;
                return USB_NO_MSG;
            }
            return 0;
        case USBRQ_HID_GET_IDLE: // send idle rate to PC as required by spec
            usbMsgPtr = &idleRate;
            return 1;
        case USBRQ_HID_SET_IDLE: // save idle rate as required by spec
            idleRate = rq->wValue.bytes[1];
            return 0;
        }
    }
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR) {
		current_command = rq->bRequest;
    	switch(rq->bRequest)
		{
    	case CUSTOM_RQ_SET_SECRET:
    	    secret_length_b = rq->wValue.word;
    	    if (secret_length_b > 256) {
    	        secret_length_b = 256;
    	    }
    	    uni_buffer.w8[0] = 0;
    	    return USB_NO_MSG;
    	case CUSTOM_RQ_INC_COUNTER:
    	    percnt_inc(0);
    	    return 0;
    	case CUSTOM_RQ_GET_COUNTER:
    	    uni_buffer.w32[0] = percnt_get(0);
    	    usbMsgPtr = (usbMsgPtr_t)uni_buffer.w32;
    	    return 4;
    	case CUSTOM_RQ_RESET_COUNTER:
    	    counter_reset();
    	    return 0;
        case CUSTOM_RQ_GET_RESET_COUNTER:
    	    eeprom_busy_wait();
            uni_buffer.w8[0] = eeprom_read_byte(&reset_counter_ee);
            usbMsgPtr = uni_buffer.w8;
            return 1;
    	case CUSTOM_RQ_SET_DIGITS:
    	    if (rq->wValue.bytes[0] > 9) {
    	        rq->wValue.bytes[0] = 9;
    	    }
    	    eeprom_busy_wait();
    	    eeprom_write_byte(&digits_ee, rq->wValue.bytes[0]);
    	    return 0;
    	case CUSTOM_RQ_GET_DIGITS:
    	    eeprom_busy_wait();
            uni_buffer.w8[0] = eeprom_read_byte(&digits_ee);
            usbMsgPtr = uni_buffer.w8;
            return 1;
    	case CUSTOM_RQ_GET_TOKEN:
    	    token_generate();
    	    usbMsgPtr = token;
    	    return strlen(token);

    	case CUSTOM_RQ_PRESS_BUTTON:
    	    key_state = STATE_SEND_KEY;
    	    return 0;
    	case CUSTOM_RQ_CLR_DBG:
            memset(dbg_buffer, 0, sizeof(dbg_buffer));
            return 0;
		case CUSTOM_RQ_SET_DBG:
			return USB_NO_MSG;
		case CUSTOM_RQ_GET_DBG:{
			usbMsgLen_t len = 8;
			if(len > rq->wLength.word){
				len = rq->wLength.word;
			}
			usbMsgPtr = dbg_buffer;
			return len;
		}
		case CUSTOM_RQ_READ_MEM:
			usbMsgPtr = (uchar*)rq->wValue.word;
			return rq->wLength.word;
		case CUSTOM_RQ_WRITE_MEM:
		case CUSTOM_RQ_EXEC_SPM:
/*			uni_buffer_fill = 4;
			uni_buffer.w16[0] = rq->wValue.word;
			uni_buffer.w16[1] = rq->wLength.word;
			return USB_NO_MSG;
*/		case CUSTOM_RQ_READ_FLASH:
			uni_buffer.w16[0] = rq->wValue.word;
			uni_buffer.w16[1] = rq->wLength.word;
            uni_buffer_fill = 4;
			return USB_NO_MSG;
		case CUSTOM_RQ_RESET:
			soft_reset((uint8_t)(rq->wValue.word));
			break;
		case CUSTOM_RQ_READ_BUTTON:
			uni_buffer.w8[0] = read_button();
			usbMsgPtr = uni_buffer.w8;
			return 1;
		case CUSTOM_RQ_READ_TMPSENS:
			uni_buffer.w16[0] = read_temperture_sensor();
			usbMsgPtr = uni_buffer.w8;
			return 2;
		}
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}


uchar usbFunctionWrite(uchar *data, uchar len)
{
	switch(current_command){

	case LED_WRITE:
	    if (data[0] != LED_state)
	        LED_state = data[0];
	    return 1; // Data read, not expecting more
	case CUSTOM_RQ_SET_SECRET:
        {
            if (uni_buffer.w8[0] < (secret_length_b + 7) / 8) {
                memcpy(&secret[uni_buffer.w8[0]], data, len);
                uni_buffer.w8[0] += len;
            }
            if (uni_buffer.w8[0] >= (secret_length_b + 7) / 8) {
                secret_set();
                return 1;
            }
            return 0;
        }
	case CUSTOM_RQ_SET_DBG:
		if(len > sizeof(dbg_buffer)){
			len = sizeof(dbg_buffer);
		}
		memcpy(dbg_buffer, data, len);
		return 1;
	case CUSTOM_RQ_WRITE_MEM:
		memcpy(uni_buffer.ptr[0], data, len);
		uni_buffer.w16[0] += len;
		return !(uni_buffer.w16[1] -= len);
	case CUSTOM_RQ_EXEC_SPM:
		if(uni_buffer_fill < 8){
			uint8_t l = 8 - uni_buffer_fill;
			if(len<l){
				len = l;
			}
			memcpy(&(uni_buffer.w8[uni_buffer_fill]), data, len);
			uni_buffer_fill += len;
			return 0;
		}
		uni_buffer.w16[1] -= len;
		if (uni_buffer.w16[1] > 8) {
			memcpy(uni_buffer.ptr[0], data, len);
			uni_buffer.w16[0] += len;
			return 0;
		} else {
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
	uchar ret = len;
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
    do {
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if(x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    } while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if (x < 0)
            x = -x;
        if (x < optimumDev) {
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

char key_seq[] = "Hello World";

int main(void)
{
	uchar  i;
	size_t idx = 0;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

    init_temperature_sensor();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for ~512 ms */
        wdt_reset();
        _delay_ms(2);
    }
    usbDeviceConnect();
    LED_PORT_DDR |= _BV(R_BIT) | _BV(G_BIT) | _BV(B_BIT);   /* make the LED bit an output */

	
    sei();

    for(;;){                /* main event loop */
        wdt_reset();
        usbPoll();

        if(usbInterruptIsReady() && key_state != STATE_WAIT){
            switch(key_state) {
            case STATE_SEND_KEY:
                buildReport(key_seq[idx]);
                key_state = STATE_RELEASE_KEY; // release next
                break;
            case STATE_RELEASE_KEY:
                buildReport(0);
                ++idx;
                if (key_seq[idx] == '\0') {
                    idx = 0;
                    key_state = STATE_WAIT;
                } else {
                    key_state = STATE_SEND_KEY;
                }
                break;
            default:
                key_state = STATE_WAIT; // should not happen
            }
                        // start sending
            usbSetInterrupt((void *)&keyboard_report, sizeof(keyboard_report));

        }

    }
    return 0;
}

/* ------------------------------------------------------------------------- */
