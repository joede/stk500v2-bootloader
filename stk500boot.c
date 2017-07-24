/*****************************************************************************
Title:     STK500v2 compatible bootloader
Author:    Peter Fleury  <pfleury@gmx.ch> http://tinyurl.com/peterfleury
           Joerg Desch <github@jdesch.de> https://github.com/joede/stk500v2-bootloader
Release:   1.2
File:      stk500boot.c
Compiler:  avr-gcc 4.8.1 / avr-libc 1.8
Hardware:  All AVRs with bootloader support
License:   GNU General Public License Version 3

DESCRIPTION:

    This program allows an AVR with bootloader capabilities to read/write its
    own Flash/EEprom. To enter Programming mode, either an input pin is
    checked (called "pin mode"), or the serial port is checked for a defined
    pattern (called "forced mode"). The bootloader wount leave if there is no
    application code found (flash at address 0x0000 is 0xFF)!

    pin mode: If a defined pin is pulled low, programming mode is entered. If not,
    normal execution is done from $0000 "reset" vector in Application area.

    forced mode: In "forced mode", the bootloader waits up to 3 seconds for a
    sequence of 5+ consecutive '*' at the UART. If this '*' sequence is received,
    the bootloader starts replying '*' and wait for an empty UART buffer. If no
    more data is fetched, the regular bootloader is entered.

    In best case, the size of the bootloader is less than 500 words, so it will
    fit into a 512 word bootloader section!

USAGE, NOTES:
    See README.md.

LICENSE:

    Copyright (C) 2006 Peter Fleury

    Modifications (C) 2016, 2017 by Jörg Desch and Contributors

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#include <inttypes.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "command.h"
#include CONFIG_FILE


/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           2
#define CONFIG_PARAM_SW_MINOR           0x0A


/* Calculate the address where the application section ends from FLASHEND and
 * from start address of bootloader. The address where the bootloader starts is
 * passed by Makefile using -DBOOTLOADER_ADDRESS
 */
#define APP_END (BOOTLOADER_ADDRESS - 1)


/*
 *  Defines for the various USART registers
 */
#ifdef USE_USART0

#ifdef UBRRL
#define UART_BAUD_RATE_LOW       UBRRL
#else
#ifdef UBRR0L
#define UART_BAUD_RATE_LOW       UBRR0L
#endif
#endif

#ifdef UBRR0H
#define UART_BAUD_RATE_HIGH      UBRR0H
#endif

#ifdef UCSRA
#define UART_STATUS_REG          UCSRA
#else
#ifdef UCSR0A
#define UART_STATUS_REG          UCSR0A
#endif
#endif

#ifdef UCSRB
#define UART_CONTROL_REG         UCSRB
#else
#ifdef UCSR0B
#define UART_CONTROL_REG         UCSR0B
#endif
#endif

#ifdef TXEN
#define UART_ENABLE_TRANSMITTER  TXEN
#else
#ifdef TXEN0
#define UART_ENABLE_TRANSMITTER  TXEN0
#endif
#endif

#ifdef RXEN
#define UART_ENABLE_RECEIVER     RXEN
#else
#ifdef RXEN0
#define UART_ENABLE_RECEIVER     RXEN0
#endif
#endif

#ifdef TXC
#define UART_TRANSMIT_COMPLETE   TXC
#else
#ifdef TXC0
#define UART_TRANSMIT_COMPLETE   TXC0
#endif
#endif

#ifdef RXC
#define UART_RECEIVE_COMPLETE    RXC
#else
#ifdef RXC0
#define UART_RECEIVE_COMPLETE    RXC0
#endif
#endif

#ifdef UDR
#define UART_DATA_REG            UDR
#else
#ifdef UDR0
#define UART_DATA_REG            UDR0
#endif
#endif

#ifdef U2X
#define UART_DOUBLE_SPEED        U2X
#else
#ifdef U2X0
#define UART_DOUBLE_SPEED        U2X0
#endif
#endif

#ifdef FE
#define UART_FRAME_ERROR         FE
#else
#ifdef FE0
#define UART_FRAME_ERROR         FE0
#endif
#endif

#ifdef DOR
#define UART_DATA_OVERRUN        DOR
#else
#ifdef DOR0
#define UART_DATA_OVERRUN        DOR0
#endif
#endif

/* ATMega with two USART, select second USART for bootloader using USE_USART1 define */
#else
#ifdef USE_USART1

#ifdef UBRR1L
#define UART_BAUD_RATE_LOW       UBRR1L
#endif

#ifdef UBRR1H
#define UART_BAUD_RATE_HIGH      UBRR1H
#endif

#ifdef UCSR1A
#define UART_STATUS_REG          UCSR1A
#endif

#ifdef UCSR1B
#define UART_CONTROL_REG         UCSR1B
#endif

#ifdef TXEN1
#define UART_ENABLE_TRANSMITTER  TXEN1
#endif

#ifdef RXEN1
#define UART_ENABLE_RECEIVER     RXEN1
#endif

#ifdef TXC1
#define UART_TRANSMIT_COMPLETE   TXC1
#endif

#ifdef RXC1
#define UART_RECEIVE_COMPLETE    RXC1
#endif

#ifdef UDR1
#define UART_DATA_REG            UDR1
#endif

#ifdef U2X1
#define UART_DOUBLE_SPEED        U2X1
#endif

#ifdef FE
#define UART_FRAME_ERROR         FE
#else
#ifdef FE1
#define UART_FRAME_ERROR         FE1
#endif
#endif

#ifdef DOR
#define UART_DATA_OVERRUN        DOR
#else
#ifdef DOR1
#define UART_DATA_OVERRUN        DOR1
#endif
#endif

#else
#error "USE_USART0 / USE_USART1 undefined  !"
#endif
#endif

#if defined(UART_BAUD_RATE_LOW) && defined(UART_STATUS_REG) && defined(UART_CONTROL_REG) && defined(UART_ENABLE_TRANSMITTER) && defined(UART_ENABLE_RECEIVER) && defined(UART_TRANSMIT_COMPLETE) && defined(UART_RECEIVE_COMPLETE) && defined(UART_DATA_REG) && defined(UART_DOUBLE_SPEED)
#else
#error "no UART definition for MCU available"
#endif


/*
 * Macros to map the new ATmega88/168 EEPROM bits
 */
#ifdef EEMPE
#define EEMWE EEMPE
#define EEWE  EEPE
#endif


/*
 * Macro to calculate UBBR from XTAL and baudrate
 */
#if UART_BAUDRATE_DOUBLE_SPEED
#  define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
#else
#  define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*16.0)-1.0+0.5)
#endif

/*
 * The mega32, mega128 and the mega162 use MCUCSR instead of MCUSR
 */
#ifdef MCUCSR
  #define _MCUSR MCUCSR
#else
  #define _MCUSR MCUSR
#endif

/*
 * States used in the receive state machine
 */
#define ST_START        0
#define ST_GET_SEQ_NUM  1
#define ST_MSG_SIZE_1   2
#define ST_MSG_SIZE_2   3
#define ST_GET_TOKEN    4
#define ST_GET_DATA     5
#define ST_GET_CHECK    6
#define ST_PROCESS      7


/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
typedef uint32_t address_t;
#else
typedef uint16_t address_t;
#endif


/*
 * function prototypes
 */
static inline void wdt_maybe_ping(void);
static inline void wdt_maybe_ping_constant_time(void);
static void sendchar(char c);
static unsigned char recchar(void);
static void leave_bootloader(void);
#if defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_FORCED_MODE_ENTER)
static char haveChar(char expected);
static char check_forced_enter(void);
#endif
static void flash_led(uint8_t count);
static void init_ports ( void );


/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain     (void) __attribute__ ((naked)) __attribute__ ((section (".init9")));

void __jumpMain(void)
{
    asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );       // init stack
    asm volatile ( "clr __zero_reg__" );                       // GCC depends on register r1 set to 0
    asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );  // set SREG to 0
    wdt_maybe_ping();
#if !defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_PROG_PIN_PULLUP)
    PROG_PORT |= (1<<PROG_PIN);                                // Enable internal pullup
#endif
    asm volatile ( "rjmp main");                               // jump to main()
}

static inline void wdt_maybe_ping(void)
{
#ifndef REMOVE_WDT_PING
    wdt_reset();
#endif
}

static inline void wdt_maybe_ping_constant_time(void)
{
/* hope WDT reset and NOP insns take the same time in every AVR uC */
#ifndef REMOVE_WDT_PING
    wdt_reset();
#else
    _NOP();
#endif
}

/*
 * send single byte to USART, wait until transmission is completed
 */
static void sendchar(char c)
{
    UART_DATA_REG = c;                                         // prepare transmission
    wdt_maybe_ping();
    while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE)));// wait until byte sent
    wdt_maybe_ping();                                          // TX should take less than WDT period
    UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);          // delete TXCflag
}

/*
 * Read single byte from USART, block if no data available
 */
static unsigned char recchar(void)
{
    do
	wdt_maybe_ping();
    while(!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE)));  // wait for data

    //~ if ( ((UART_STATUS_REG&(1<<UART_FRAME_ERROR)) | (UART_STATUS_REG&(1<<UART_DATA_OVERRUN))) )
	//~ return 0;
    return UART_DATA_REG;
}

/* Try to read a byte from the UART. If there is a byte, compare it with the
 * passed byte \c expected. The functions returns \c true, it a received
 * byte equals to \c expected. In all other cases \c false is returned.
 *
 * If \c expected is 0x00, than we just wait for a character and do not
 * compare it. In this case a returned \x true means that we have received
 * a character. Even the overrun error and the frame error will be ignored.
 */
#if defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_FORCED_MODE_ENTER)
static char haveChar ( char expected )
{
    uint8_t d, s;
    char failed = 0;

    if ( !(UART_STATUS_REG&(1<<UART_RECEIVE_COMPLETE)) )
	return 0;
    s = UART_STATUS_REG; d = UART_DATA_REG;
    if ( ((s&(1<<UART_FRAME_ERROR)) | (s&(1<<UART_DATA_OVERRUN))) )
	failed = 1;

    if ( (expected!=0x00) && failed )
	return 0;
    return ((expected==0x00)||(expected==d))?1:0;
}
#endif

/* Leave the bootloader if application code is found.
 */
static void leave_bootloader (void)
{

    if ( pgm_read_byte_near(0x0000) == 0xFF )
    {
	// no application we can "leave to"
	return;
    }

#ifndef REMOVE_BOOTLOADER_LED
    PROGLED_PORT |= (1<<PROGLED_PIN);
    PROGLED_DDR  &= ~(1<<PROGLED_PIN);   // set to default
#endif

    /* Now leave bootloader
     */
#if !defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_PROG_PIN_PULLUP)
    PROG_PORT &= ~(1<<PROG_PIN);    // set to default
#endif
    boot_rww_enable();              // enable application section

    wdt_maybe_ping();

    // Jump to Reset vector in Application Section. The address is always 22bit (three bytes), even if the MCU
    // hasn't more than 128 kB flash. The wrong stack is resetted by the startup code of the runtime of the
    // application. This way we don't need ugly ifdefs.
    // (clear register, push this register to the stack three times = adress 0x000000, and return to this address)
    asm volatile (
        "clr r1" "\n\t"
        "push r1" "\n\t"
        "push r1" "\n\t"
        "push r1" "\n\t"
        "ret"     "\n\t"
	::);
}

/* Check for "forced mode" to enter the bootloader. In the case there is no
 * bootloader pin, the bootloader can be configured (REMOVE_FORCED_MODE_ENTER)
 * to wait 3 seconds for a sequence of 5+ consecutive '*' characters at the UART.
 *
 * If this '*' sequence is received, the bootloader starts sending '*' and waits
 * for an empty UART buffer. If no more data is fetched, the regular bootloader
 * can be entered.
 *
 * This functions returns \c true, if the bootloader can be enteres or not.
 */
#if defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_FORCED_MODE_ENTER)
static char check_forced_enter ( void )
{
    volatile uint32_t l;		/* fool the optimizer */
    uint8_t matches = 0;
    uint8_t count = 0;

    /* wait for the "forced enter" sequence of 5x '*' or timeout after 3sec
     */
    do
    {
	if ( count++ )
	    for (l=0; l<(F_CPU/200); ++l)
		wdt_maybe_ping_constant_time();
	if ( haveChar('*') )
	{
	    ++matches;
	}
    } while ( count<20 && matches<5 );
    if ( matches<5 )
    {
	return 0;
    }

    /* wait for "silence" on the UART. To indicate we are ready to enter
     * the bootloader, we now send '*'
     */
    matches = 0;
    count = 0;
    do
    {
	sendchar('*');
	if ( count++ )
	    for (l=0; l<(F_CPU/200); ++l)
		wdt_maybe_ping_constant_time();
	if ( !haveChar(0x00) )
	    ++matches;
	else
	    matches = 0;
    } while ( matches<8 );

    sendchar('-'); sendchar('-');
    sendchar('e');sendchar('n');sendchar('t');sendchar('e');sendchar('r');sendchar('e');sendchar('d');
    sendchar('-'); sendchar('-');  sendchar('\r'); sendchar('\n');
    return 1;
}
#endif

/* flash onboard LED three times to signal entering of bootloader
 */
static void flash_led ( uint8_t count )
{
#ifndef REMOVE_BOOTLOADER_LED
    volatile uint32_t l;		/* fool the optimizer */
    PROGLED_DDR  |= (1<<PROGLED_PIN);
    if (count == 0)
      count = 3;
    do
    {
	PROGLED_PORT &= ~(1<<PROGLED_PIN);
	for ( l=0; l < (F_CPU/500); ++l)
            wdt_maybe_ping_constant_time();

	PROGLED_PORT |= (1<<PROGLED_PIN);
	for ( l=0; l < (F_CPU/800); ++l)
            wdt_maybe_ping_constant_time();
    } while (--count);
    for ( l=0; l < (F_CPU/100); ++l )
	wdt_maybe_ping_constant_time();
    // the LED indicates an active loader
    PROGLED_PORT &= ~(1<<PROGLED_PIN);
#endif
}

/* Initialize the MCU ports
 */
static void init_ports ( void )
{
#if defined(VAL_PORT_A) && defined(PORTA)
    PORTA = VAL_PORT_A;  DDRA = DIR_PORT_A;
#endif
#if defined(VAL_PORT_B) && defined(PORTB)
    PORTB = VAL_PORT_B;  DDRB = DIR_PORT_B;
#endif
#if defined(VAL_PORT_C) && defined(PORTC)
    PORTC = VAL_PORT_C;  DDRC = DIR_PORT_C;
#endif
#if defined(VAL_PORT_D) && defined(PORTD)
    PORTD = VAL_PORT_D;  DDRD = DIR_PORT_D;
#endif
#if defined(VAL_PORT_E) && defined(PORTE)
    PORTE = VAL_PORT_E;  DDRE = DIR_PORT_E;
#endif
#if defined(VAL_PORT_F) && defined(PORTF)
    PORTF = VAL_PORT_F;  DDRF = DIR_PORT_F;
#endif
#if defined(VAL_PORT_G) && defined(PORTG)
    PORTG = VAL_PORT_G;  DDRG = DIR_PORT_G;
#endif
#if defined(VAL_PORT_H) && defined(PORTH)
    PORTH = VAL_PORT_H;  DDRH = DIR_PORT_H;
#endif
#if defined(VAL_PORT_J) && defined(PORTJ)
    PORTJ = VAL_PORT_J;  DDRJ = DIR_PORT_J;
#endif
#if defined(VAL_PORT_K) && defined(PORTK)
    PORTK = VAL_PORT_K;  DDRK = DIR_PORT_K;
#endif
#if defined(VAL_PORT_L) && defined(PORTL)
    PORTL = VAL_PORT_L;  DDRL = DIR_PORT_L;
#endif
}



int main(void) __attribute__ ((OS_main));
int main(void)
{
    address_t       address = 0;
    address_t       eraseAddress = 0;
    unsigned char   msgParseState;
    unsigned int    i = 0;
    unsigned char   checksum = 0;
    unsigned char   seqNum = 0;
    unsigned int    msgLength = 0;
    unsigned char   msgBuffer[285];
    unsigned char   c, *p;
    unsigned char   isLeave = 0;

#ifndef REMOVE_WDT_RESET
    if ( _MCUSR & _BV(WDRF) )
	leave_bootloader();
#endif

    wdt_maybe_ping();

    /* optionally setup the CPU ports
     */
    init_ports();

    flash_led(3);

    /* Branch to bootloader or application code ?
     */
#ifndef REMOVE_PROG_PIN_ENTER
    if ( PROG_IN&(1<<PROG_PIN) )
    {
	leave_bootloader();
    }
#endif

    /*
     * Init UART
     * set baudrate and enable USART receiver and transmiter without interrupts
     */
#if UART_BAUDRATE_DOUBLE_SPEED
    UART_STATUS_REG   |=  (1 <<UART_DOUBLE_SPEED);
#endif

#ifdef UART_BAUD_RATE_HIGH
    UART_BAUD_RATE_HIGH = 0;
#endif
    UART_BAUD_RATE_LOW = UART_BAUD_SELECT(BAUDRATE,F_CPU);
    UART_CONTROL_REG   = (1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

#if defined(REMOVE_PROG_PIN_ENTER) && !defined(REMOVE_FORCED_MODE_ENTER)
    if ( pgm_read_byte_near(0x0000) != 0xFF )
    {
	if ( !check_forced_enter() )
	{
          leave_bootloader();
	}
    }
#endif

    /* indicate with LED that bootloader is active */
    flash_led(2);

    /* main loop */
    while(!isLeave)
    {
	/*
	 * Collect received bytes to a complete message
	 */
	msgParseState = ST_START;
	while ( msgParseState != ST_PROCESS )
	{
	    /* recchar() pings WDT */
	    c = recchar();
	    switch (msgParseState)
	    {
	    case ST_START:
		if( c == MESSAGE_START )
		{
		    msgParseState = ST_GET_SEQ_NUM;
		    checksum = MESSAGE_START^0;
		}
		break;

	    case ST_GET_SEQ_NUM:
		if ( (c == 1) || (c == seqNum) )
		{
		    seqNum = c;
		    msgParseState = ST_MSG_SIZE_1;
		    checksum ^= c;
		}
		else
		{
		    msgParseState = ST_START;
		}
		break;

	    case ST_MSG_SIZE_1:
		msgLength = (unsigned int)c<<8;
		msgParseState = ST_MSG_SIZE_2;
		checksum ^= c;
		break;

	    case ST_MSG_SIZE_2:
		msgLength |= c;
		msgParseState = ST_GET_TOKEN;
		checksum ^= c;
		break;

	    case ST_GET_TOKEN:
		if ( c == TOKEN )
		{
		    msgParseState = ST_GET_DATA;
		    checksum ^= c;
		    i = 0;
		}
		else
		{
		    msgParseState = ST_START;
		}
		break;

	    case ST_GET_DATA:
		msgBuffer[i++] = c;
		checksum ^= c;
		if ( i == msgLength )
		{
		    msgParseState = ST_GET_CHECK;
		}
		break;

	    case ST_GET_CHECK:
		if( c == checksum )
		{
		    msgParseState = ST_PROCESS;
		}
		else
		{
		    msgParseState = ST_START;
		}
		break;
	    }//switch
	}//while(msgParseState)

	/*
	 * Now process the STK500 commands, see Atmel Appnote AVR068
	 */

	switch (msgBuffer[0])
	{
#ifndef REMOVE_CMD_SPI_MULTI
	case CMD_SPI_MULTI:
	    {
		unsigned char answerByte = 0;

		// only Read Signature Bytes implemented, return dummy value for other instructions
		if ( msgBuffer[4]== 0x30 )
		{
		    unsigned char signatureIndex = msgBuffer[6];

		    if ( signatureIndex == 0 )
			answerByte = SIGNATURE_0;
		    else if ( signatureIndex == 1 )
			answerByte = SIGNATURE_1;
		    else
			answerByte = SIGNATURE_2;
		}
		msgLength = 7;
		msgBuffer[1] = STATUS_CMD_OK;
		msgBuffer[2] = 0;
		msgBuffer[3] = msgBuffer[4];  // Instruction Byte 1
		msgBuffer[4] = msgBuffer[5];  // Instruction Byte 2
		msgBuffer[5] = answerByte;
		msgBuffer[6] = STATUS_CMD_OK;
	    }
	    break;
#endif
	case CMD_SIGN_ON:
	    msgLength = 11;
	    msgBuffer[1]  = STATUS_CMD_OK;
	    msgBuffer[2]  = 8;
	    msgBuffer[3]  = 'A';
	    msgBuffer[4]  = 'V';
	    msgBuffer[5]  = 'R';
	    msgBuffer[6]  = 'I';
	    msgBuffer[7]  = 'S';
	    msgBuffer[8]  = 'P';
	    msgBuffer[9]  = '_';
	    msgBuffer[10] = '2';
	    break;

	case CMD_GET_PARAMETER:
	    switch(msgBuffer[1])
	    {
	    case PARAM_BUILD_NUMBER_LOW:
		msgBuffer[2] = CONFIG_PARAM_BUILD_NUMBER_LOW;
		break;
	    case PARAM_BUILD_NUMBER_HIGH:
		msgBuffer[2] = CONFIG_PARAM_BUILD_NUMBER_HIGH;
		break;
	    case PARAM_HW_VER:
		msgBuffer[2] = CONFIG_PARAM_HW_VER;
		break;
	    case PARAM_SW_MAJOR:
		msgBuffer[2] = CONFIG_PARAM_SW_MAJOR;
		break;
	    case PARAM_SW_MINOR:
		msgBuffer[2] = CONFIG_PARAM_SW_MINOR;
		break;
	    default:
		msgBuffer[2] = 0;
		break;
	    }
	    msgLength = 3;
	    msgBuffer[1] = STATUS_CMD_OK;
	    break;

	case CMD_LEAVE_PROGMODE_ISP:
#ifdef ENABLE_LEAVE_BOOTLADER
	    isLeave = 1;
#endif
	case CMD_ENTER_PROGMODE_ISP:
	case CMD_SET_PARAMETER:
	    msgLength = 2;
	    msgBuffer[1] = STATUS_CMD_OK;
	    break;

	case CMD_READ_SIGNATURE_ISP:
	    {
		unsigned char signatureIndex = msgBuffer[4];
		unsigned char signature;

		if ( signatureIndex == 0 )
		    signature = SIGNATURE_0;
		else if ( signatureIndex == 1 )
		    signature = SIGNATURE_1;
		else
		    signature = SIGNATURE_2;

		msgLength = 4;
		msgBuffer[1] = STATUS_CMD_OK;
		msgBuffer[2] = signature;
		msgBuffer[3] = STATUS_CMD_OK;
	    }
	    break;

#ifndef REMOVE_READ_LOCK_FUSE_BIT_SUPPORT
	case CMD_READ_LOCK_ISP:
	    msgLength = 4;
	    msgBuffer[1] = STATUS_CMD_OK;
	    msgBuffer[2] = boot_lock_fuse_bits_get( GET_LOCK_BITS );
	    msgBuffer[3] = STATUS_CMD_OK;
	    break;

	case CMD_READ_FUSE_ISP:
	    {
		unsigned char fuseBits;

		if ( msgBuffer[2] == 0x50 )
		{
		    if ( msgBuffer[3] == 0x08 )
			fuseBits = boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
		    else
			fuseBits = boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
		}
		else
		{
		    fuseBits = boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
		}
		msgLength = 4;
		msgBuffer[1] = STATUS_CMD_OK;
		msgBuffer[2] = fuseBits;
		msgBuffer[3] = STATUS_CMD_OK;
	    }
	    break;
#endif
#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
	case CMD_PROGRAM_LOCK_ISP:
	    {
		unsigned char lockBits = msgBuffer[4];

		lockBits = (~lockBits) & 0x3C;  // mask BLBxx bits
		wdt_maybe_ping();
		boot_lock_bits_set(lockBits);   // and program it
		while (boot_spm_busy())
		    wdt_maybe_ping();

		msgLength = 3;
		msgBuffer[1] = STATUS_CMD_OK;
		msgBuffer[2] = STATUS_CMD_OK;
	    }
	    break;
#endif
	case CMD_CHIP_ERASE_ISP:
	    eraseAddress = 0;
	    msgLength = 2;
	    msgBuffer[1] = STATUS_CMD_OK;
	    break;

	case CMD_LOAD_ADDRESS:
#if defined(RAMPZ)
	    address = ((address_t)(msgBuffer[1]) << 24) |
		    ((address_t)(msgBuffer[2]) << 16) |
		    ((address_t)(msgBuffer[3]) << 8) |
		    (msgBuffer[4]);

	    /* unset address bit 31 which is a request to execute a */
	    /* "load extended address" command by hardware programmer */
	    address &= ~((address_t)1 << 31);
#else
	    address = ((msgBuffer[3]) << 8) | (msgBuffer[4]);
#endif
	    msgLength = 2;
	    msgBuffer[1] = STATUS_CMD_OK;
	    break;

	case CMD_PROGRAM_FLASH_ISP:
	case CMD_PROGRAM_EEPROM_ISP:
	    {
		unsigned int  size = (((unsigned int)msgBuffer[1])<<8) | msgBuffer[2];
		unsigned char *p = msgBuffer+10;
		unsigned int  data;
		unsigned char highByte, lowByte;

		if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
		{
		    address_t pageaddress = address;

		    // erase only main section (bootloader protection)
		    if  (  eraseAddress < APP_END )
		    {
			wdt_maybe_ping();
			boot_page_erase(eraseAddress);  // Perform page erase
			while (boot_spm_busy())         // Wait until the memory is erased.
			    wdt_maybe_ping();
			eraseAddress += SPM_PAGESIZE;    // point to next page to be erase
		    }

		    /* Write FLASH */
		    do {
			wdt_maybe_ping();

			lowByte   = *p++;
			highByte  = *p++;

			data =  (highByte << 8) | lowByte;
			boot_page_fill(address << 1, data);

			address++;          // Select next word in memory
			size -= 2;          // Reduce number of bytes to write by two
		    } while(size);          // Loop until all bytes written

		    wdt_maybe_ping();
		    boot_page_write(pageaddress << 1);
		    while (boot_spm_busy())
			wdt_maybe_ping();
		    boot_rww_enable();              // Re-enable the RWW section
		}
		else
		{
		    /* write EEPROM */
		    do {
			EEARL = address;            // Setup EEPROM address
			EEARH = (address >> 8);
			address++;                  // Select next EEPROM byte

			EEDR= *p++;                 // get byte from buffer
			EECR |= (1<<EEMWE);         // Write data into EEPROM
			EECR |= (1<<EEWE);

			do
		            wdt_maybe_ping();
			while (EECR & (1<<EEWE));   // Wait for write operation to finish
			size--;                     // Decrease number of bytes to write
		    } while(size);                  // Loop until all bytes written
		}
		msgLength = 2;
		msgBuffer[1] = STATUS_CMD_OK;
	    }
	    break;

	case CMD_READ_FLASH_ISP:
	case CMD_READ_EEPROM_ISP:
	    {
		unsigned int  size = (((unsigned int)msgBuffer[1])<<8) | msgBuffer[2];
		unsigned char *p = msgBuffer+1;
		msgLength = size+3;

		*p++ = STATUS_CMD_OK;
		if (msgBuffer[0] == CMD_READ_FLASH_ISP )
		{
		    unsigned int data;

		    // Read FLASH
		    do {
#if defined(RAMPZ)
			data = pgm_read_word_far(address << 1);
#else
			data = pgm_read_word_near(address << 1);
#endif
			*p++ = (unsigned char)data;         //LSB
			*p++ = (unsigned char)(data >> 8);  //MSB
			address++; // Select next word in memory
			size -= 2;
		    }while (size);
		}
		else
		{
		    /* Read EEPROM */
		    do {
			EEARL = address;            // Setup EEPROM address
			EEARH = ((address >> 8));
			address++;                  // Select next EEPROM byte
			EECR |= (1<<EERE);          // Read EEPROM
			*p++ = EEDR;                // Send EEPROM data
			size--;
		    }while(size);
		}
		*p++ = STATUS_CMD_OK;
	    }
	    break;

	default:
	    msgLength = 2;
	    msgBuffer[1] = STATUS_CMD_FAILED;
	    break;
	}

	/*
	 * Now send answer message back
	 */
	sendchar(MESSAGE_START);
	checksum = MESSAGE_START^0;

	sendchar(seqNum);
	checksum ^= seqNum;

	c = ((msgLength>>8)&0xFF);
	sendchar(c);
	checksum ^= c;

	c = msgLength&0x00FF;
	sendchar(c);
	checksum ^= c;

	sendchar(TOKEN);
	checksum ^= TOKEN;

	p = msgBuffer;
	while ( msgLength )
	{
	   c = *p++;
	   sendchar(c);
	   checksum ^=c;
	   msgLength--;
	}
	sendchar(checksum);
	seqNum++;

    }//for

    leave_bootloader();

    /* Never return to stop GCC to generate exit return code
     * Actually we will never reach this point, but the compiler doesn't
     * understand this
     */
    for (;;)
        ;
}
