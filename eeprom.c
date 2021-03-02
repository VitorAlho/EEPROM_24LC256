#include <stdint.h>
#include "eeprom.h"
#include "24LC256.h"

int8_t EscreverNaEEprom (int8_t endereco, uint8_t dado){
    
    return EEPROM_24LC256_I2C_write_uchar(0,endereco,dado);
    
}

int8_t LerDadosDaEEprom (int8_t endereco, uint8_t *dado){
    
    return EEPROM_24LC256_I2C_read_uchar(0,endereco,dado);
    
}
