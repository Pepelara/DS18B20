/* ds18b20.c - a part of avr-ds18b20 library
 *
 * Copyright (C) 2016 Jacek Wieczorek
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.	See the LICENSE file for details.
 */

/**
	\file
	\brief Implements functions for controlling the DS18B20 sensors
*/

#include <stddef.h>
#include <string.h>
#include "ds18b20.h"

#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1

static void ds18b20_dummy_pinMode(void *port, int pin, int mode);
static void ds18b20_dummy_writePin(void *port, int pin, int out);
static void ds18b20_dummy_portDelay(int delay);
static void ds18b20_dummy_readPin(void *port, int pin);
static void irq_disable_dummy(void);

static onewire_lib_t private_onewire_lib = {
    .user_pinMode = ds18b20_dummy_pinMode,
    .user_writePin = ds18b20_dummy_writePin,
    .user_portDelay = ds18b20_dummy_portDelay,
    .user_readPin = ds18b20_dummy_readPin,
    .disable_irq = irq_disable_dummy,
	.enable_irq = irq_disable_dummy,
};

void onewire_lib_init(onewire_lib_t *lib)
{
    private_onewire_lib.user_pinMode = lib->user_pinMode;
    private_onewire_lib.user_writePin = lib->user_writePin;
    private_onewire_lib.user_portDelay = lib->user_portDelay;
    private_onewire_lib.user_readPin = lib->user_readPin;
	private_onewire_lib.disable_irq = lib->disable_irq;
	private_onewire_lib.enable_irq = lib->enable_irq;
}


//! Calculate CRC of provided data
uint8_t ds18_crc8( uint8_t *data, uint8_t length )
{
    //Generate 8bit CRC for given data (Maxim/Dallas)

    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t mix = 0;
    uint8_t crc = 0;
    uint8_t byte = 0;

    for ( i = 0; i < length; i++ )
    {
        byte = data[i];

        for( j = 0; j < 8; j++ )
        {
            mix = ( crc ^ byte ) & 0x01;
            crc >>= 1;
            if ( mix ) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

//! Perform ROM matching
void ds18_match( onewire_bus_t *owbus, ds18_dev_t *dev )
{
    //Perform ROM match operation on DS18B20 devices
    //Or skip ROM matching if ptr is NULL
    uint8_t i = 0;
	uint8_t *rom = &(dev->rom);
	
    //If rom pointer is NULL then read temperature without matching.
    if ( dev == NULL || dev->rom == 0)
    {
        //Skip ROM
        ds18_onewireWrite( owbus, DS18B20_COMMAND_SKIP_ROM );
    }
    else
    {
        //Match ROM
        ds18_onewireWrite( owbus, DS18B20_COMMAND_MATCH_ROM );
        for ( i = 0; i < 8; i++ )
            ds18_onewireWrite( owbus, rom[i] );
    }
}


//! Request temperature conversion
uint8_t ds18_convert( onewire_bus_t *owbus, ds18_dev_t *dev )
{
    //Send conversion request to DS18B20 on one wire bus

    //Communication check
    if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
        return DS18B20_ERROR_COMM;

    //ROM match (or not)
    ds18_match( owbus, dev );

    //Convert temperature
    ds18_onewireWrite( owbus, DS18B20_COMMAND_CONVERT );

    return DS18B20_ERROR_OK;
}


//! Read sensor scratchpad contents
uint8_t ds18_rsp( onewire_bus_t *owbus, ds18_dev_t *dev)
{
    //Read DS18B20 scratchpad

    uint8_t i = 0;
    uint8_t crc;
    uint8_t *sp = (uint8_t *)dev;

    //Communication check
    if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
        return DS18B20_ERROR_COMM;

    //Match (or not) ROM
    ds18_match( owbus, dev );

    //Read scratchpad
    ds18_onewireWrite( owbus, DS18B20_COMMAND_READ_SP );
    for ( i = 0; i < 8; i++ )
        sp[i] = ds18_onewireRead( owbus );
    crc = ds18_onewireRead( owbus );

    //Check pull-up
    if ( ( sp[0] | sp[1] | sp[2] | sp[3] | sp[4] | sp[5] | sp[6] | sp[7] ) == 0 )
        return DS18B20_ERROR_PULL;

    //CRC check

    if ( ds18_crc8( sp, 8 ) != crc )
        return DS18B20_ERROR_CRC;
    return DS18B20_ERROR_OK;
}


//! Write sensor scratchpad
uint8_t ds18_wsp( onewire_bus_t *owbus, ds18_dev_t *dev, uint8_t th, uint8_t tl, uint8_t conf )
{
    //Writes DS18B20 scratchpad
    //th - thermostat high temperature
    //tl - thermostat low temperature
    //conf - configuration byte

    //Communication check
    if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
        return DS18B20_ERROR_COMM;

    //ROM match (or not)
    ds18_match( owbus, dev );

    //Write scratchpad
    ds18_onewireWrite( owbus, DS18B20_COMMAND_WRITE_SP );
    ds18_onewireWrite( owbus, th );
    ds18_onewireWrite( owbus, tl );
    ds18_onewireWrite( owbus, conf );

    return DS18B20_ERROR_OK;
}


//! Copy scratchpad to EEPROM
uint8_t ds18_csp( onewire_bus_t *owbus, ds18_dev_t *dev )
{
    //Copies DS18B20 scratchpad contents to its EEPROM

    //Communication check
    if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
        return DS18B20_ERROR_COMM;

    //ROM match (or not)
    ds18_match( owbus, dev );

    //Copy scratchpad
    ds18_onewireWrite( dev, DS18B20_COMMAND_COPY_SP );

    //Set pin high
    //Poor DS18B20 feels better then...
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1);
    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, OUTPUT);

    return DS18B20_ERROR_OK;
}


//! Read temperature
uint8_t ds18_read( onewire_bus_t *owbus, ds18_dev_t *dev, int16_t *temp)
{
    //Read temperature from DS18B20
    //Note: returns actual temperature * 16

    uint8_t *sp = &(dev->sp);
    uint8_t ec = 0;

    //Communication, pull-up, CRC checks happen here
    ec = ds18_rsp( owbus, dev );

    if ( ec != DS18B20_ERROR_OK )
    {
        *temp = 0;
        return ec;
    }

    //Get temperature from received data
    *temp = (int16_t)( sp[1] << 8 ) + sp[0]; 

    return DS18B20_ERROR_OK;
}

//! Read ROM address
uint8_t ds18_rr( onewire_bus_t *owbus, ds18_dev_t *dev )
{
    //Read DS18B20 rom
    uint8_t *rom = &(dev->sp);
    unsigned char i = 0;

    if ( rom == NULL ) return DS18B20_ERROR_OTHER;

    //Communication check
    if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
        return DS18B20_ERROR_COMM;

    //Read ROM
    ds18_onewireWrite( owbus, DS18B20_COMMAND_READ_ROM );
    for ( i = 0; i < 8; i++ )
        rom[i] = ds18_onewireRead( owbus );

    //Pull-up check
    if ( ( rom[0] | rom[1] | rom[2] | rom[3] | rom[4] | rom[5] | rom[6] | rom[7] ) == 0 )
        return DS18B20_ERROR_PULL;

    //Check CRC
    if ( ds18_crc8( rom, 7 ) != rom[7] )
    {
        for ( i = 0; i < 8; i++ )
            rom[i] = 0;
        return DS18B20_ERROR_CRC;
    }

    return DS18B20_ERROR_OK;
}

uint8_t ds18_search( onewire_bus_t *owbus, ds18_dev_t *devs, uint8_t *romcnt, uint16_t buflen )
{
	uint8_t bit, currom = 0;
	uint64_t i, junction = 0, rom;
		
	private_onewire_lib.disable_irq();
	// 1 loop - 1 thermometer discovered
	do
	{
		// Reset current ROM buffer
		rom = 0;

		// Initiate ROM search
		if ( ds18_onewireInit( owbus ) == ONEWIRE_ERROR_COMM )
		{
			*romcnt = 0;
			private_onewire_lib.enable_irq();
			return DS18B20_ERROR_COMM;
		}
		ds18_onewireWrite( owbus, DS18B20_COMMAND_SEARCH_ROM );

		// Access 64 bits of ROM
		for ( i = 1; i; i <<= 1 )
		{
			//Request two complementary bits from sensors
			bit = 0;
			bit |= ds18_onewireReadBit( owbus );
			bit |= ds18_onewireReadBit( owbus ) << 1;

			switch ( bit )
			{
				//Received 11 - no sensors connected
				case 3:
					*romcnt = 0; //Null pointer check is at the begining
					private_onewire_lib.enable_irq();
					return DS18B20_ERROR_COMM;
					break;

				//Received 10 or 01 - ROM bits match
				case 1:
				case 2:
					bit &= 1;
					break;

				//Received 00 - ROM bits differ
				case 0:
					// Check if there are older junction bits set
					// If there are older bits set, junction value
					// shall be at least equal to i*2
					if ( junction >= ( i << 1 ) )
					// if ( junction >= ( 2 << i ) )
					{
						// Send complement of junction bit
						bit = !( junction & i );
					}
					else
					{
						// Send value of junction bit and toggle it afterwards
						bit = ( junction & i ) != 0;
						junction ^= i;
					}
					break;
			}

			// Send response bit depending on junction status
			// At this point bit is either 0 or 1 and corresponds
			// to the discovered ROM value
			ds18_onewireWriteBit( owbus, bit );
			
			// Set bit in temporary ROM buffer
			rom |= bit ? i : 0;
		}

		// Copy prepared ROM to its destination
		if (( currom + 1 ) <= buflen )
			memcpy(&devs[currom].rom , &rom, 8 );
	}
	while ( ++currom && junction ); // As long as there are junction bits set
	private_onewire_lib.enable_irq();
	
	*romcnt = currom;
	if ( currom == 0 ){
		return DS18B20_ERROR_COMM; // Exit because of currom overflow (junction broken?)
	}
	
	return DS18B20_ERROR_OK;
}

//! Initializes 1wire bus before transmission
uint8_t ds18_onewireInit( onewire_bus_t *owbus )
{
    uint8_t response = 0;

    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1); //Write 1 to output
    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, OUTPUT);
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 0); //Write 0 to output

    private_onewire_lib.user_portDelay( 600 );

    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, INPUT);

    private_onewire_lib.user_portDelay( 70 );

    response = private_onewire_lib.user_readPin(owbus->GPIOx, owbus->pin); //Read input

    private_onewire_lib.user_portDelay( 200 );

    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1); //Write 1 to output
    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, OUTPUT); //Set port to output

    private_onewire_lib.user_portDelay( 600 );

    return response != 0 ? ONEWIRE_ERROR_COMM : ONEWIRE_ERROR_OK;
}

//! Sends a single bit over the 1wire bus
uint8_t ds18_onewireWriteBit(onewire_bus_t *owbus, uint8_t bit )
{
	private_onewire_lib.disable_irq();
	
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1); //Write 1 to output
    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, OUTPUT);
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 0); //Write 0 to output

    if ( bit != 0 ) private_onewire_lib.user_portDelay( 8 );
    else private_onewire_lib.user_portDelay( 80 );

    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1);

    if ( bit != 0 ) private_onewire_lib.user_portDelay( 80 );
    else private_onewire_lib.user_portDelay( 2 );
    
    private_onewire_lib.enable_irq();
    return bit != 0;
}

//! Transmits a byte over 1wire bus
void ds18_onewireWrite( onewire_bus_t *owbus, uint8_t data )
{
    uint8_t i = 0;

    for ( i = 1; i != 0; i <<= 1 ) //Write byte in 8 single bit writes
        ds18_onewireWriteBit( owbus, data & i );

}

//! Reads a bit from the 1wire bus
uint8_t ds18_onewireReadBit( onewire_bus_t *owbus )
{
    uint8_t bit = 0;
	private_onewire_lib.disable_irq();
	
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 1); //Write 1 to output
    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, OUTPUT);
    private_onewire_lib.user_writePin(owbus->GPIOx, owbus->pin, 0); //Write 0 to output

    private_onewire_lib.user_portDelay( 3 );

    private_onewire_lib.user_pinMode(owbus->GPIOx, owbus->pin, INPUT); //Set port to input

    private_onewire_lib.user_portDelay( 7 );

    bit = private_onewire_lib.user_readPin(owbus->GPIOx, owbus->pin); //Read input

    private_onewire_lib.user_portDelay( 60 );
	private_onewire_lib.enable_irq();
	
    return bit;
}

//! Reads a byte from the 1wire bus
uint8_t ds18_onewireRead( onewire_bus_t *owbus )
{
    uint8_t data = 0;
    uint8_t i = 0;


    for ( i = 0; i < 8; i ++ ) //Read byte in 8 single bit reads
        data |= ds18_onewireReadBit( owbus ) << i;

    return data;
}

static void irq_disable_dummy(void){
	return;
}
static void ds18b20_dummy_pinMode(void *port, int pin, int mode) {
    return;
}
static void ds18b20_dummy_writePin(void *port, int pin, int out) {
    return;
}
static void ds18b20_dummy_portDelay(int delay) {
    return;
}
static void ds18b20_dummy_readPin(void *port, int pin) {
    return;
}
