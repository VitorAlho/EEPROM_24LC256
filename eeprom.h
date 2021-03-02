#ifndef _EEPROM_H_
#define _EEPROM_H_

int8_t EscreverNaEEprom (int8_t endereco, uint8_t dado);

int8_t LerDadosDaEEprom (int8_t endereco, uint8_t *dado);

#endif