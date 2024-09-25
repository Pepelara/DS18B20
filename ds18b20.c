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
#include "ds18b20.h"

#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1

static void ds18b20_dummy_pinMode(void *port, int pin, int mode);
static void ds18b20_dummy_writePin(void *port, int pin, int out);
static void ds18b20_dummy_portDelay(int delay);
static void ds18b20_dummy_readPin(void *port, int pin);


static ds18_lib_t ds18_lib = {
    .user_pinMode = ds18b20_dummy_pinMode, 
    .user_writePin = ds18b20_dummy_writePin, 
    .user_portDelay = ds18b20_dummy_portDelay,
    .user_readPin = ds18b20_dummy_readPin
};

 void ds18_lib_init(ds18_lib_t *lib)
{
    ds18_lib.user_pinMode = lib->user_pinMode;
    ds18_lib.user_writePin = lib->user_writePin;
    ds18_lib.user_portDelay = lib->user_portDelay;
    ds18_lib.user_readPin = lib->user_readPin;
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
void ds18_match( ds18_dev_t *dev )
{
	//Perform ROM match operation on DS18B20 devices
	//Or skip ROM matching if ptr is NULL
	uint8_t i = 0;

	//If rom pointer is NULL then read temperature without matching.
	if ( dev->rom == NULL )
	{
		//Skip ROM
		ds18_onewireWrite( dev, DS18B20_COMMAND_SKIP_ROM );
	}
	else
	{
		//Match ROM
		ds18_onewireWrite( dev, DS18B20_COMMAND_MATCH_ROM );
		for ( i = 0; i < 8; i++ )
			ds18_onewireWrite( dev, dev->rom[i] );
	}
}


//! Request temperature conversion
uint8_t ds18_convert( ds18_dev_t *dev )
{
	//Send conversion request to DS18B20 on one wire bus

	//Communication check
	if ( ds18_onewireInit( dev ) == ONEWIRE_ERROR_COMM )
		return DS18B20_ERROR_COMM;

	//ROM match (or not)
	ds18_match( dev );

	//Convert temperature
	ds18_onewireWrite( dev, DS18B20_COMMAND_CONVERT );

	return DS18B20_ERROR_OK;
}


//! Read sensor scratchpad contents
uint8_t ds18_rsp( ds18_dev_t *dev )
{
	//Read DS18B20 scratchpad

	uint8_t i = 0;
	uint8_t *sp = dev->sp; 

	//Communication check
	if ( ds18_onewireInit( dev ) == ONEWIRE_ERROR_COMM )
		return DS18B20_ERROR_COMM;
	
	//Match (or not) ROM
	ds18_match( dev );

	//Read scratchpad
	ds18_onewireWrite( dev, DS18B20_COMMAND_READ_SP );
	for ( i = 0; i < 9; i++ )
		sp[i] = ds18_onewireRead( dev );

	//Check pull-up
	if ( ( sp[0] | sp[1] | sp[2] | sp[3] | sp[4] | sp[5] | sp[6] | sp[7] ) == 0 )
		return DS18B20_ERROR_PULL;

	//CRC check
	
	if ( ds18_crc8( sp, 8 ) != sp[8] )
		return DS18B20_ERROR_CRC;
	return DS18B20_ERROR_OK;
}


//! Write sensor scratchpad
uint8_t ds18_wsp( ds18_dev_t *dev, uint8_t th, uint8_t tl, uint8_t conf )
{
	//Writes DS18B20 scratchpad
	//th - thermostat high temperature
	//tl - thermostat low temperature
	//conf - configuration byte

	//Communication check
	if ( ds18_onewireInit( dev ) == ONEWIRE_ERROR_COMM )
		return DS18B20_ERROR_COMM;

	//ROM match (or not)
	ds18_match( dev );

	//Write scratchpad
	ds18_onewireWrite( dev, DS18B20_COMMAND_WRITE_SP );
	ds18_onewireWrite( dev, th );
	ds18_onewireWrite( dev, tl );
	ds18_onewireWrite( dev, conf );

	return DS18B20_ERROR_OK;
}


//! Copy scratchpad to EEPROM
uint8_t ds18_csp( ds18_dev_t *dev )
{
	//Copies DS18B20 scratchpad contents to its EEPROM

	//Communication check
	if ( ds18_onewireInit( dev ) == ONEWIRE_ERROR_COMM )
		return DS18B20_ERROR_COMM;

	//ROM match (or not)
	ds18_match( dev );

	//Copy scratchpad
	ds18_onewireWrite( dev, DS18B20_COMMAND_COPY_SP );

	//Set pin high
	//Poor DS18B20 feels better then...
	ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1);
    ds18_lib.user_pinMode(dev->GPIOx, dev->pin, OUTPUT);

	return DS18B20_ERROR_OK;
}


//! Read temperature
uint8_t ds18_read( ds18_dev_t *dev, int32_t *temp)
{
	//Read temperature from DS18B20
	//Note: returns actual temperature * 16

	uint8_t *sp = dev->sp;
	uint32_t ec = 0;

	//Communication, pull-up, CRC checks happen here
	ec = ds18_rsp( dev );

	if ( ec != DS18B20_ERROR_OK )
	{
		*temp = 0;
		return ec;
	}

	//Get temperature from received data
	*temp = (int32_t)( sp[1] << 8 ) | sp[0];

	return DS18B20_ERROR_OK;
}

//! Read ROM address
uint8_t ds18_rr( ds18_dev_t *dev )
{
	//Read DS18B20 rom
	uint8_t *rom = dev->rom; 
	unsigned char i = 0;
	
	if ( rom == NULL ) return DS18B20_ERROR_OTHER;

	//Communication check
	if ( ds18_onewireInit( dev ) == ONEWIRE_ERROR_COMM )
		return DS18B20_ERROR_COMM;

	//Read ROM
	ds18_onewireWrite( dev, DS18B20_COMMAND_READ_ROM );
	for ( i = 0; i < 8; i++ )
		rom[i] = ds18_onewireRead( dev );

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

//! Initializes 1wire bus before transmission
uint8_t ds18_onewireInit( ds18_dev_t *dev )
{
	uint8_t response = 0;

    ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1); //Write 1 to output
    ds18_lib.user_pinMode(dev->GPIOx, dev->pin, OUTPUT);
    ds18_lib.user_writePin(dev->GPIOx, dev->pin, 0); //Write 0 to output
    
	ds18_lib.user_portDelay( 600 );

	ds18_lib.user_pinMode(dev->GPIOx, dev->pin, INPUT);

	ds18_lib.user_portDelay( 70 );

	response = ds18_lib.user_readPin(dev->GPIOx, dev->pin); //Read input

	ds18_lib.user_portDelay( 200 );

	ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1); //Write 1 to output
	ds18_lib.user_pinMode(dev->GPIOx, dev->pin, OUTPUT); //Set port to output

	ds18_lib.user_portDelay( 600 );

	return response != 0 ? ONEWIRE_ERROR_COMM : ONEWIRE_ERROR_OK;
}

//! Sends a single bit over the 1wire bus
uint8_t ds18_onewireWriteBit(ds18_dev_t *dev, uint8_t bit )
{

	ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1); //Write 1 to output
    ds18_lib.user_pinMode(dev->GPIOx, dev->pin, OUTPUT);
    ds18_lib.user_writePin(dev->GPIOx, dev->pin, 0); //Write 0 to output

	if ( bit != 0 ) ds18_lib.user_portDelay( 8 );
	else ds18_lib.user_portDelay( 80 );

	ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1);

	if ( bit != 0 ) ds18_lib.user_portDelay( 80 );
	else ds18_lib.user_portDelay( 2 );

	return bit != 0;
}

//! Transmits a byte over 1wire bus
void ds18_onewireWrite( ds18_dev_t *dev, uint8_t data )
{
	uint8_t i = 0;
	
	for ( i = 1; i != 0; i <<= 1 ) //Write byte in 8 single bit writes
		ds18_onewireWriteBit( dev, data & i );

}

//! Reads a bit from the 1wire bus
uint8_t ds18_onewireReadBit( ds18_dev_t *dev )
{
	uint8_t bit = 0;
	
	ds18_lib.user_writePin(dev->GPIOx, dev->pin, 1); //Write 1 to output
    ds18_lib.user_pinMode(dev->GPIOx, dev->pin, OUTPUT);
    ds18_lib.user_writePin(dev->GPIOx, dev->pin, 0); //Write 0 to output
	
	ds18_lib.user_portDelay( 3 );
	
	ds18_lib.user_pinMode(dev->GPIOx, dev->pin, INPUT); //Set port to input
	
	ds18_lib.user_portDelay( 7 );
	
	bit = ds18_lib.user_readPin(dev->GPIOx, dev->pin); //Read input
	
	ds18_lib.user_portDelay( 60 );

	return bit;
}

//! Reads a byte from the 1wire bus
uint8_t ds18_onewireRead( ds18_dev_t *dev )
{
	uint8_t data = 0;
	uint8_t i = 0;


	for ( i = 0; i < 8; i ++ ) //Read byte in 8 single bit reads
		data |= ds18_onewireReadBit( dev ) << i;

	return data;
}


static void ds18b20_dummy_pinMode(void *port, int pin, int mode){
	return;
}
static void ds18b20_dummy_writePin(void *port, int pin, int out){
	return;
}
static void ds18b20_dummy_portDelay(int delay){
	return;
}
static void ds18b20_dummy_readPin(void *port, int pin){
	return;
}
