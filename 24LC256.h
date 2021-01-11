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
 * Utilize somente as fun��es listadas abaixo. Demais s�o utilizadas apenas internamente.
 * 
 * Exemplo de utiliza��o da biblioteca no MPLAB X IDE para o PIC24FJ256DA210
 * 
    int main(void)
    {    
        uint8_t dado;

        EEPROM_24LC256_stats status;

        SYSTEM_Initialize();

        i2c3_driver_init(100000); // 100 KHz

        EEPROM_24LC256_load_callbacks(i2c3_driver_start,
                                      i2c3_driver_restart,
                                      i2c3_driver_stop,
                                      i2c3_driver_TXData,
                                      i2c3_driver_getRXData,
                                      i2c3_driver_sendAck,
                                      delay_ms);

        status = EEPROM_24LC256_I2C_write_uchar(0,100,0xF7);

        status = EEPROM_24LC256_I2C_read_uchar(0,100,&dado);   

        if( dado == 0xF7 ) {

            dado = dado; // Insert a breakpoint here. TEST HAS PASSED

        }

        while (1)
        {

        }

        return 1;

    }
 * 
 */

#ifndef _24LC256_H_
#define	_24LC256_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/* 
 * Enumerador com os status da comunica��o I2C retornado por todas
 * as fun��es.
 */
typedef enum {    
    EEPROM_24LC256_TIMEOUT = -1,   
    EEPROM_24LC256_OK,    
    EEPROM_24LC256_ERROR,
    EEPROM_24LC256_ADDRESS_OUT_OF_RANGE
} EEPROM_24LC256_stats;


void EEPROM_24LC256_delay_ms ( uint16_t ms_delay );

/* 
 * Fun��o que recebe o endere�o de mem�ria das fun��es 
 * do protocolo I2C e carrega dentro da biblioteca para 
 * comunicar com a mem�ria EEPROM
 */
void EEPROM_24LC256_load_callbacks(void* I2C_start,
                                   void* I2C_restart,
                                   void* I2C_stop,
                                   void* I2C_sendData_uchar,
                                   void* I2C_receiveData_uchar,
                                   void* I2C_sendAck,
                                   void* EEPROM_24LC256_delay_ms);

/* 
 * Fun��o que realiza a LEITURA de um dado, em um determinado endere�o da mem�ria
 * EEPROM.
 */
EEPROM_24LC256_stats EEPROM_24LC256_I2C_read_uchar ( uint8_t i2c_address, uint8_t mem_address, uint8_t *data );

/* 
 * Fun��o que realiza a ESCRITA de um dado em um determinado endere�o dA mem�ria
 * EEPROM.
 */
EEPROM_24LC256_stats EEPROM_24LC256_I2C_write_uchar (uint8_t i2c_address ,uint8_t mem_address, uint8_t data);



///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// FUN��ES USADAS INTERNAMENTE ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

/* 
 * Fun��o que sinaliza o in�cio da transmiss�o MASTER -> SLAVE no barramento
 */
EEPROM_24LC256_stats  EEPROM_24LC256_I2C_start(int16_t timeout);

/* 
 * Fun��o que sinaliza o restart da comunica��o MASTER -> SLAVE no barramento
 */
EEPROM_24LC256_stats  EEPROM_24LC256_I2C_restart(int16_t timeout);

/* 
 * Fun��o que sinaliza o fim (stop) da comunica��o MASTER -> SLAVE no barramento
 */
EEPROM_24LC256_stats  EEPROM_24LC256_I2C_stop(int16_t timeout);

/* 
 * Fun��o que recebe um dado de 8 bits enviado pelo SLAVE no barramento
 */
EEPROM_24LC256_stats EEPROM_24LC256_I2C_getRxData(uint8_t *data, int16_t timeout);

/* 
 * Fun��o que envia um dado de 8 bits para o SLAVE no barramento
 */
EEPROM_24LC256_stats EEPROM_24LC256_I2C_sendTxData(uint8_t data, int16_t timeout);

EEPROM_24LC256_stats EEPROM_24LC256_I2C_sendAck(uint16_t timeout);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

