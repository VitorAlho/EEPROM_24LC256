/* 
 * 
 * File: 24LC256.H
 * 
 * Author: Eng. Vitor Alho
 * 
 * Comments: Biblioteca com alto n�vel de abstra��o para mem�ria EEPROM 24LC256
 * 
 * Data: Dezembro de 2020
 * 
 * Mais informa��es no arquivo 24LC256.H
 * 
 */

#include "24LC256.h"

#define EEPROM_24LC256_MAX_TIMEOUT 65

#define EEPROM_24LC256_MAX_ADDRESS 7
#define EEPROM_24LC256_MIN_ADDRESS 0

#define EEPROM_24LC256_MIN_DELAY_BETWEEN_OPERATIONS 1

EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_start) (int16_t timeout);
EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_stop) (int16_t timeout);
EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_restart) (int16_t timeout);
EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_getRxData) (uint8_t *data, int16_t timeout);
EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_transmitTxData) (uint8_t data, int16_t timeout);
EEPROM_24LC256_stats (*EEPROM_24LC256_I2C_wrapper_sendAck) (int16_t timeout);

void (*EEPROM_24LC256_wrapper_delay_ms) (uint16_t ms_delay);

EEPROM_24LC256_stats EEPROM_24LC256_I2C_write_uchar ( uint8_t i2c_address, uint8_t mem_address, uint8_t data ){
    
    EEPROM_24LC256_stats status = EEPROM_24LC256_ERROR;
    
    if( i2c_address > EEPROM_24LC256_MAX_ADDRESS || i2c_address < EEPROM_24LC256_MIN_ADDRESS ) { 
        
        return EEPROM_24LC256_ADDRESS_OUT_OF_RANGE;
        
    }
   
    status = EEPROM_24LC256_I2C_start( EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( 0xA0 + ((i2c_address & 0x07) << 1), EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( ( mem_address >> 8 ), EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( mem_address & 0x00FF, EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( data, EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_stop( EEPROM_24LC256_MAX_TIMEOUT );
    
    if( status != EEPROM_24LC256_OK ) return status;
    
    EEPROM_24LC256_delay_ms(EEPROM_24LC256_MIN_DELAY_BETWEEN_OPERATIONS);
    
    return EEPROM_24LC256_OK;

}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_read_uchar ( uint8_t i2c_address, uint8_t address, uint8_t *data ){

    EEPROM_24LC256_stats status = EEPROM_24LC256_ERROR;
    
    if( i2c_address > EEPROM_24LC256_MAX_ADDRESS || i2c_address < EEPROM_24LC256_MIN_ADDRESS ) { 
        
        return EEPROM_24LC256_ADDRESS_OUT_OF_RANGE;
        
    }
    
    status = EEPROM_24LC256_I2C_start( EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( 0xA0 + ((i2c_address & 0x07) << 1),EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( ( address >> 8 ),EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( address & 0x00FF,EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_restart( EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_sendTxData( 0xA1 + ((i2c_address & 0x07) << 1),EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_getRxData( data, EEPROM_24LC256_MAX_TIMEOUT );

    if( status != EEPROM_24LC256_OK ) return status;
    
    status = EEPROM_24LC256_I2C_stop( EEPROM_24LC256_MAX_TIMEOUT );
    
    if( status != EEPROM_24LC256_OK ) return status;
    
    EEPROM_24LC256_delay_ms(EEPROM_24LC256_MIN_DELAY_BETWEEN_OPERATIONS);
    
    return EEPROM_24LC256_OK;
    
}

void EEPROM_24LC256_load_callbacks(void* I2C_start,
                                   void* I2C_restart,
                                   void* I2C_stop,
                                   void* I2C_sendData_uchar,
                                   void* I2C_receiveData_uchar,
                                   void* I2C_sendAck,
                                   void* EEPROM_24LC256_delay_ms) {
    
    EEPROM_24LC256_I2C_wrapper_start          = (EEPROM_24LC256_stats (*)  (int16_t))           I2C_start;
    EEPROM_24LC256_I2C_wrapper_stop           = (EEPROM_24LC256_stats (*)  (int16_t))           I2C_stop;
    EEPROM_24LC256_I2C_wrapper_restart        = (EEPROM_24LC256_stats (*)  (int16_t))           I2C_restart;
    EEPROM_24LC256_I2C_wrapper_getRxData      = (EEPROM_24LC256_stats (*)  (uint8_t*, int16_t)) I2C_receiveData_uchar;
    EEPROM_24LC256_I2C_wrapper_transmitTxData = (EEPROM_24LC256_stats (*)  (uint8_t, int16_t))  I2C_sendData_uchar;
    EEPROM_24LC256_I2C_wrapper_sendAck        = (EEPROM_24LC256_stats (*)  (int16_t))           I2C_sendAck;
    
    EEPROM_24LC256_wrapper_delay_ms    = (void (*) (uint16_t)) EEPROM_24LC256_delay_ms;
    
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// FUN��ES USADAS INTERNAMENTE ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void EEPROM_24LC256_delay_ms ( uint16_t ms_delay ) {
    
    (*EEPROM_24LC256_wrapper_delay_ms)(ms_delay);
    
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_start(int16_t timeout) {
    switch( (*EEPROM_24LC256_I2C_wrapper_start)(timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_restart(int16_t timeout) {
    switch( (*EEPROM_24LC256_I2C_wrapper_restart)(timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_stop(int16_t timeout) {
    switch( (*EEPROM_24LC256_I2C_wrapper_stop)(timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_getRxData(uint8_t *data, int16_t timeout) {
    switch( (*EEPROM_24LC256_I2C_wrapper_getRxData)(data, timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_sendTxData(uint8_t data, int16_t timeout) {
    switch( (*EEPROM_24LC256_I2C_wrapper_transmitTxData)(data,timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
}

EEPROM_24LC256_stats EEPROM_24LC256_I2C_sendAck(uint16_t timeout) {
    
    switch( (*EEPROM_24LC256_I2C_wrapper_sendAck)(timeout) ) {
        case EEPROM_24LC256_TIMEOUT:
            return EEPROM_24LC256_TIMEOUT;
        break;
        case EEPROM_24LC256_OK:
            return EEPROM_24LC256_OK;
        break;
        default:
            return EEPROM_24LC256_ERROR;
        break;
    }
    
}