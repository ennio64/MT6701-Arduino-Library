/* 
 * Classe per Arduino IDE che implementa molti metodi
 * di interazione con il sensore di posizione senza contatto
 * MT6701 della società MagnTek http://www.magntek.com.cn/en/index.htm
 * 
 * Documentazione del sensore:
 ** http://www.magntek.com.cn/en/list/177/559.htm
 ** http://www.magntek.com.cn/upload/MT6701_Rev.1.5.pdf
 * 
 * Contatti:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.2 / Licenza MIT / Roman Sklyar S-LAB
 */

#include "MT6701_I2C.h"

// ########## COSTRUTTORE ##########
/*
 * @brief: utilizzare solo l'interfaccia I2C
 * @param _twi: accesso ai metodi dell'oggetto Wire
 */
MT6701I2C::MT6701I2C(TwoWire* _twi) : _wire_(_twi ? _twi : &Wire) {
  // Nulla
}
/* 
 * @brief: richiedere un byte di dati dal buffer
 * @param _reg_addr: 1 byte dell'indirizzo del registro
 * @return: valore del byte dal registro che è stato richiesto
 * @note: utilizzare per un singolo registro, ad esempio 0x29
 */
uint8_t MT6701I2C::MT_RequestSingleRegister(uint8_t _reg_addr) {
  uint8_t single_byte = 0;

  // Inizia la trasmissione all'indirizzo
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Invia il byte del registro
  _wire_->write(_reg_addr);
  // Termina la connessione
  _wire_->endTransmission();
  
  // Richiedi un byte di dati all'indirizzo
  _wire_->requestFrom(MT6701_I2C_ADDRESS, (uint8_t)1);
  // Leggi i dati dal buffer
  if (_wire_->available() >= 1 ) {
    single_byte = _wire_->read();
  }
  // Termina la connessione
  _wire_->endTransmission();

  return single_byte;
}
/*
 * @brief: scrivere un valore di 1 byte in un registro arbitrario di 1 byte
 * @param _reg_addr: 1 byte dell'indirizzo del registro
 * @param _payload: 1 byte di dati utili
 */
void MT6701I2C::MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload) {
  // Inizia la trasmissione all'indirizzo per inviare un byte di dati al registro
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  _wire_->write(_reg_addr);
  _wire_->write(_payload);
  // Termina la connessione
  _wire_->endTransmission();
}

// ########## PUBBLICO ##########
/* 
 * @brief: chiamata al metodo Wire.begin()
 * @note: utilizzare se l'azione non è stata eseguita in precedenza
 */
void MT6701I2C::begin(void) {
  _wire_->begin();
}
/* 
 * @brief: chiamata al metodo Wire.begin(SDA, SCL) specificando i pin
 * @param _sda_pin: pin SDA personalizzato
 * @param _scl_pin: pin SCL personalizzato
 * @note: utilizzare se l'azione non è stata eseguita in precedenza.
 *   Applicabile per piattaforme basate su ESP8266 e ESP32
 */
#if defined(ESP8266) || defined(ESP32)
void MT6701I2C::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->begin(_sda_pin, _scl_pin);
}
#endif

#if defined(ARDUINO_ARCH_STM32)
void MT6701I2C::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->setSDA(_sda_pin);
  _wire_->setSCL(_scl_pin);
  _wire_->begin();
}
#endif
/* 
 * @brief: impostazione di una frequenza arbitraria del bus I2C (default 400kHz)
 * @note: utilizzare se la frequenza del bus cambia a causa di diversi dispositivi
 */
void MT6701I2C::setClock(uint32_t _clock) {
  _wire_->setClock(_clock);
}
/*
 * @brief: salva i dati nella memoria EEPROM del sensore
 * @note: lo scopo di ogni comando non è descritto nella documentazione, l'ordine dei comandi è descritto in 7.2 EEPROM Programming
 *  si raccomanda di eseguire questa operazione con una tensione di alimentazione tra 4.5V e 5.5V
 */
void MT6701I2C::saveNewValues(void) {
  // Inizia la trasmissione all'indirizzo
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Invia 0x09
  _wire_->write(MT6701_I2C_EEPROM_PROG_KEY_REG);
  // Invia 0xB3
  _wire_->write(MT6701_I2C_EEPROM_PROG_KEY_VALUE);
  // Termina la connessione
  _wire_->endTransmission();
  
  // Inizia la trasmissione all'indirizzo
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Invia 0x0A
  _wire_->write(MT6701_I2C_EEPROM_PROG_CMD_REG);
  // Invia 0x05
  _wire_->write(MT6701_I2C_EEPROM_PROG_CMD_VALUE);
  // Termina la connessione
  _wire_->endTransmission();
}
/*
 * @brief: verifica se il sensore è collegato alla linea I2C
 * @note: si utilizza l'algoritmo standard di ricerca dei dispositivi sulla linea I2C
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non collegato
 *  MT6701I2C_DEFAULT_REPORT_OK - collegato
 */
bool MT6701I2C::isConnected(void) {
  // Inizia la trasmissione all'indirizzo
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  return (!_wire_->endTransmission(MT6701_I2C_ADDRESS)) ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: assegna un pin del microcontrollore per controllare la modalità dell'interfaccia
 * @param _pin_mode: pin del microcontrollore a cui è collegato il pin MODE del sensore
 */
void MT6701I2C::attachModePin(byte _pin_mode) {
  _pin_mode_ = _pin_mode;
  pinMode(_pin_mode_, OUTPUT);
}
/*
 * @brief: libera il pin assegnato del microcontrollore per controllare la modalità dell'interfaccia
 */
void MT6701I2C::detachModePin(void) {
  pinMode(_pin_mode_, INPUT);
  _pin_mode_ = -1;
}
/* 
 * @brief: abilita l'interfaccia I2C/SSI
 */
void MT6701I2C::enableI2CorSSI(void) {
  digitalWrite(_pin_mode_, MT6701I2C_MODE_I2C_SSI);
}
/* 
 * @brief: abilita l'interfaccia UVW/ABZ
 */
void MT6701I2C::enableUVWorABZ(void) {
  digitalWrite(_pin_mode_, MT6701I2C_MODE_UVW_ABZ);
}
/* 
 * @brief: ottieni il valore puro dell'angolo da Angle Data Register(13:0)
 * @return:
 *  0 - 16383
 */

word MT6701I2C::getRawAngle(void) {
  uint8_t high_byte = MT_RequestSingleRegister(MT6701_I2C_ANGLE_DATA_REG_H);
  uint8_t low_byte = MT_RequestSingleRegister(MT6701_I2C_ANGLE_DATA_REG_L);
  return (word)(high_byte << 6) | (low_byte >> 2);
}
/* 
 * @brief: ottieni il valore dell'angolo in gradi
 * @return:
 *  0.00 - 359.98
 */
float MT6701I2C::getDegreesAngle(void) {
  return ((float)getRawAngle() * 360) / 16384;
}
/* 
 * @brief: ottieni il valore dell'angolo in radianti
 * @return:
 *  0.00 - 6.28319
 */
float MT6701I2C::getRadiansAngle(void) {
  return (getDegreesAngle() * M_PI) / 180;
}
/* 
 * @brief: ottieni il tipo di configurazione dell'interfaccia di uscita
 * @note: solo per il pacchetto QFN
 * @return:
 *  MT6701I2_OUTPUT_TYPE_A_B_Z
 *  MT6701I2_OUTPUT_TYPE_UVW
 */
MT6701I2CConfigurationOutputType MT6701I2C::getConfigurationOutputType(void) {
  return (MT6701I2CConfigurationOutputType)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG) >> MT6701_I2C_EEPROM_UVW_MUX_BIT) & 0x01);
}
/* 
 * @brief: imposta il tipo di configurazione dell'interfaccia di uscita -A-B-Z
 * @note: solo per il pacchetto QFN
 */
void MT6701I2C::setConfigurationOutputTypeABZ(void) {
  // Leggi il valore corrente del registro
  uint8_t current_value = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  
  // Stampa il valore corrente sulla seriale
  Serial.print("Valore corrente del registro: ");
  Serial.println(current_value, HEX);

  // Modifica il bit specificato nel valore del registro
  uint8_t new_value = current_value | (1 << MT6701_I2C_EEPROM_UVW_MUX_BIT);

  // Scrivi il nuovo valore nel registro
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_MUX_REG, new_value);

  // Leggi il valore aggiornato del registro
  uint8_t updated_value = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  
  // Stampa il valore aggiornato sulla seriale
  Serial.print("Valore aggiornato del registro: ");
  Serial.println(updated_value, HEX);
}
/*
void MT6701I2C::setConfigurationOutputTypeABZ(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_UVW_MUX_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_MUX_REG, bkup);
}
*/
/* 
 * @brief: imposta il tipo di configurazione dell'interfaccia di uscita -A-B-Z con verifica
 * @note: solo per il pacchetto QFN
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setConfigurationOutputTypeABZVerify(void) {
  setConfigurationOutputTypeABZ();
  return getConfigurationOutputType() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}
/* 
 * @brief: imposta il tipo di configurazione dell'interfaccia di uscita UVW
 * @note: solo per il pacchetto QFN
 */
void MT6701I2C::setConfigurationOutputTypeUVW(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_UVW_MUX_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_MUX_REG, bkup);
}
/* 
 * @brief: imposta il tipo di configurazione dell'interfaccia di uscita UVW con verifica
 * @note: solo per il pacchetto QFN
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setConfigurationOutputTypeUVWVerify(void) {
  setConfigurationOutputTypeUVW();
  return getConfigurationOutputType() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore del tipo di interfaccia di uscita
 * @return:
 *  MT6701I2_OUTPUT_TYPE_ABZ
 *  MT6701I2_OUTPUT_TYPE_UVW
 */
MT6701I2COutputType MT6701I2C::getOutputType(void) {
  return (MT6701I2COutputType)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG) >> MT6701_I2C_EEPROM_ABZ_MUX_BIT) & 0x01);
}
/* 
 * @brief: imposta il tipo di interfaccia di uscita ABZ
 */
void MT6701I2C::setOutputTypeABZ(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_ABZ_MUX_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_MUX_REG, bkup);
}
/* 
 * @brief: imposta il tipo di interfaccia di uscita ABZ con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputTypeABZVerify(void) {
  setOutputTypeABZ();
  return getOutputType() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}
/* 
 * @brief: imposta il tipo di interfaccia di uscita UVW
 */
void MT6701I2C::setOutputTypeUVW(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_ABZ_MUX_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_MUX_REG, bkup);
}
/* 
 * @brief: imposta il tipo di interfaccia di uscita UVW con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */

bool MT6701I2C::setOutputTypeUVWVerify(void) {
  setOutputTypeUVW();
  return getOutputType() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore della direzione di rotazione positiva
 * @return:
 *  MT6701I2_DIRECTION_COUNTERCLOCKWISE
 *  MT6701I2_DIRECTION_CLOCKWISE
 */
MT6701I2CDirection MT6701I2C::getOutputRotationDirection(void) {
  return (MT6701I2CDirection)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG) >> MT6701_I2C_EEPROM_DIR_BIT) & 0x01);
}
/* 
 * @brief: imposta la direzione di rotazione positiva in senso antiorario
 */
void MT6701I2C::setOutputRotationDirectionCounterclockwise(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_DIR_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_DIR_REG, bkup);
}
/* 
 * @brief: imposta la direzione di rotazione positiva in senso antiorario con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputRotationDirectionCounterclockwiseVerify(void) {
  setOutputRotationDirectionCounterclockwise();
  return getOutputRotationDirection() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}
/* 
 * @brief: imposta la direzione di rotazione positiva in senso orario
 */
void MT6701I2C::setOutputRotationDirectionClockwise(void) {
  uint8_t bkup =MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_DIR_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_DIR_REG, bkup);
}
/* 
 * @brief: imposta la direzione di rotazione positiva in senso orario con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputRotationDirectionClockwiseVerify(void) {
  setOutputRotationDirectionClockwise();
  return getOutputRotationDirection() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore della risoluzione di uscita in modalità UVW
 * @return:
 *  1 - 16
 */
byte MT6701I2C::getOutputResolutionUVW(void) {
  return ((MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_RES_REG) >> MT6701_I2C_EEPROM_UVW_MUX_BIT_S) & 0x0F) + 1; // 0x0F = 0b00001111, +1 per lo spostamento nell'intervallo 1-16
}
/* 
 * @brief: imposta il valore della risoluzione di uscita in modalità UVW
 * @param _resolution:
 *  1 - 16
 */
void MT6701I2C::setOutputResolutionUVW(byte _resolution) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_RES_REG);
  bkup |= (_resolution - 1) << MT6701_I2C_EEPROM_UVW_MUX_BIT_S; // -1 per lo spostamento nell'intervallo 0-15
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_RES_REG, bkup);
}
/* 
 * @brief: imposta il valore della risoluzione di uscita in modalità UVW con verifica
 * @param _resolution:
 *  1 - 16
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputResolutionUVWVerify(byte _resolution) {
  setOutputResolutionUVW(_resolution);
  return getOutputResolutionUVW() == _resolution ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore della risoluzione di uscita in modalità ABZ
 * @return:
 *  1 - 1024
 */
word MT6701I2C::getOutputResolutionABZ(void) {
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_H) & 0x03; // 0x03 = 0b00000011
  return (word)((reg_h << 8) | MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_L)) + 1; // +1 per lo spostamento nell'intervallo 1-1024
}
/* 
 * @brief: imposta il valore della risoluzione di uscita in modalità ABZ
 * @param _resolution:
 *  1 - 1024
 */

void MT6701I2C::setOutputResolutionABZ(word _resolution) {
  uint8_t reg_l = (_resolution - 1) & 0xFF;
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_H);
  reg_h |= ((_resolution - 1) >> 8) & 0x03;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_RES_REG_H, reg_h);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_RES_REG_L, reg_l);
}
/* 
 * @brief: imposta il valore della risoluzione di uscita in modalità ABZ con verifica
 * @param _resolution:
 *  1 - 1024
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputResolutionABZVerify(word _resolution) {
  setOutputResolutionABZ(_resolution);
  return getOutputResolutionABZ() == _resolution ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}


/* 
 * @brief: ottieni il valore della larghezza dell'impulso Z in modalità ABZ
 * @return:
 *  MT6701I2_Z_PULSE_WIDTH_1LSB
 *  MT6701I2_Z_PULSE_WIDTH_2LSB
 *  MT6701I2_Z_PULSE_WIDTH_4LSB
 *  MT6701I2_Z_PULSE_WIDTH_8LSB
 *  MT6701I2_Z_PULSE_WIDTH_12LSB
 *  MT6701I2_Z_PULSE_WIDTH_16LSB
 *  MT6701I2_Z_PULSE_WIDTH_180DEG
 *  MT6701I2_Z_PULSE_WIDTH_1LSB_2
 */
MT6701I2CZPulseWidth MT6701I2C::getZPulseWidth(void) {
  return (MT6701I2CZPulseWidth)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG) >> MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S) & 0x07); // 0x07 = 0b00000111
}
/*
 * @brief: imposta la larghezza dell'impulso Z 1LSB
 */
void MT6701I2C::setZPulseWidth1LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_1LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x00; // Imposta i bit 6:4 a 000
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 1LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth1LSBVerify(void) {
  setZPulseWidth1LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_1LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 2LSB
 */

void MT6701I2C::setZPulseWidth2LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_2LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x10; // Imposta i bit 6:4 a 001
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 2LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth2LSBVerify(void) {
  setZPulseWidth2LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_2LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 4LSB
 */
void MT6701I2C::setZPulseWidth4LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_4LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x20; // Imposta i bit 6:4 a 010
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 4LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth4LSBVerify(void) {
  setZPulseWidth1LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_4LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 8LSB
 */
void MT6701I2C::setZPulseWidth8LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_8LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x30; // Imposta i bit 6:4 a 011
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 8LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth8LSBVerify(void) {
  setZPulseWidth8LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_8LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 12LSB
 */
void MT6701I2C::setZPulseWidth12LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_12LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x40; // Imposta i bit 6:4 a 100
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 12LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth12LSBVerify(void) {
  setZPulseWidth12LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_12LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 16LSB
 */
void MT6701I2C::setZPulseWidth16LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_16LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x50; // Imposta i bit 6:4 a 101
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 16LSB con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth16LSBVerify(void) {
  setZPulseWidth16LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_16LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: imposta la larghezza dell'impulso Z 180 gradi
 */
void MT6701I2C::setZPulseWidth180DEG(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  //bkup |= MT6701I2_Z_PULSE_WIDTH_180DEG << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  bkup &= 0x0F; // Pulisce i bit 6:4 (imposta a 0)
  bkup |= 0x60; // Imposta i bit 6:4 a 110  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}
/*
 * @brief: imposta la larghezza dell'impulso Z 180 gradi con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZPulseWidth180DEGVerify(void) {
  setZPulseWidth180DEG();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_180DEG ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore della posizione zero
 * @note: GUARDA LA TABELLA NELLA DOCUMENTAZIONE
 * @return:
 *  0 - 4095
 */
word MT6701I2C::getZeroDegreePositionData(void) {
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_H) & 0x0F; // 0x0F = 0b00001111
  return (word)((reg_h << 8) | MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_L));
}
/* 
 * @brief: imposta il valore della posizione zero
 * @note: GUARDA LA TABELLA NELLA DOCUMENTAZIONE
 * @param _zero_position_data:
 *  0 - 4095
 */
void MT6701I2C::setZeroDegreePositionData(word _zero_position_data) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_H) & 0xF0; // 0xF0 = 0b11110000
  uint8_t reg_l = _zero_position_data & 0xFF;
  bkup |= _zero_position_data >> 8;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ZERO_REG_H, bkup);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ZERO_REG_L, reg_l);
}
/* 
 * @brief: imposta il valore della posizione zero con verifica
 * @note: GUARDA LA TABELLA NELLA DOCUMENTAZIONE
 * @param _zero_position_data:
 *  0 - 4095
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setZeroDegreePositionDataVerify(word _zero_position_data) {
  setZeroDegreePositionData(_zero_position_data);
  return getZeroDegreePositionData() == _zero_position_data ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: ottieni il valore della frequenza PWM
 * @return:
 *  MT6701I2_PWM_FREQUENCY_9944
 *  MT6701I2_PWM_FREQUENCY_4972
 */
MT6701I2CFrequencyPWM MT6701I2C::getFrequencyPWM(void) {
  return (MT6701I2CFrequencyPWM)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG) >> MT6701_I2C_EEPROM_PWM_FREQ_BIT) & 0x01);
}
/* 
 * @brief: imposta il valore della frequenza PWM 994.4Hz
 */
void MT6701I2C::setFrequencyPWM9944(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_PWM_FREQ_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_FREQ_REG, bkup);
}
/* 
 * @brief: imposta il valore della frequenza PWM 994.4Hz con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setFrequencyPWM9944Verify(void) {
  setFrequencyPWM9944();
  return getFrequencyPWM() == MT6701I2_PWM_FREQUENCY_9944 ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: imposta il valore della frequenza PWM 497.2Hz
 */
void MT6701I2C::setFrequencyPWM4972(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_PWM_FREQ_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_FREQ_REG, bkup);
}
/* 
 * @brief: imposta il valore della frequenza PWM 497.2Hz con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setFrequencyPWM4972Verify(void) {
  setFrequencyPWM4972();
  return getFrequencyPWM() == MT6701I2_PWM_FREQUENCY_4972 ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: ottieni il valore della polarità PWM
 * @return:
 *  MT6701I2_PWM_POLARITY_HIGH
 *  MT6701I2_PWM_POLARITY_LOW
 */
MT6701I2CPolarityPWM MT6701I2C::getPolarityPWM(void) {
  return (MT6701I2CPolarityPWM)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG) >> MT6701_I2C_EEPROM_PWM_POL_BIT) & 0x01);
}
/* 
 * @brief: imposta il valore della polarità PWM HIGH
 */
void MT6701I2C::setPolarityPWMHigh(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_PWM_POL_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_POL_REG, bkup);
}
/* 
 * @brief: imposta il valore della polarità PWM HIGH con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setPolarityPWMHighVerify(void) {
  setPolarityPWMHigh();
  return getPolarityPWM() == MT6701I2_PWM_POLARITY_HIGH ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: imposta il valore della polarità PWM LOW
 */
void MT6701I2C::setPolarityPWMLow(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_PWM_POL_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_POL_REG, bkup);
}
/* 
 * @brief: imposta il valore della polarità PWM LOW con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setPolarityPWMLowVerify(void) {
  setPolarityPWMLow();
  return getPolarityPWM() == MT6701I2_PWM_POLARITY_LOW ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: ottieni la modalità di uscita
 * @return:
 *  MT6701I2_OUTPUT_MODE_ANALOG
 *  MT6701I2_OUTPUT_MODE_PWM
 */
MT6701I2COutputMode MT6701I2C::getOutputMode(void) {
  return (MT6701I2COutputMode)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG) >> MT6701_I2C_EEPROM_OUT_MODE_BIT) & 0x01);
}
/* 
 * @brief: imposta la modalità di uscita Analogica
 */
void MT6701I2C::setOutputModeAnalog(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_OUT_MODE_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_OUT_MODE_REG, bkup);
}
/* 
 * @brief: imposta la modalità di uscita Analogica con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputModeAnalogVerify(void) {
  setOutputModeAnalog();
  return getOutputMode() == MT6701I2_OUTPUT_MODE_ANALOG ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: imposta la modalità di uscita PWM
 */
void MT6701I2C::setOutputModePWM(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_OUT_MODE_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_OUT_MODE_REG, bkup);
}
/* 
 * @brief: imposta la modalità di uscita PWM con verifica
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - non impostato
 *  MT6701I2C_DEFAULT_REPORT_OK - impostato
 */
bool MT6701I2C::setOutputModePWMVerify(void) {
  setOutputModePWM();
  return getOutputMode() == MT6701I2_OUTPUT_MODE_PWM ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

//-------------------- codice aggiunto ----------------------------------------------------

void MT6701I2C::printRegisterState(uint8_t registerAddress, uint8_t interestBit) {
    uint8_t registerValue = MT_RequestSingleRegister(registerAddress);
    uint8_t interestBitValue = (registerValue >> interestBit) & 0x01;

    Serial.print("Registro ");
    Serial.print(getRegisterName(registerAddress, interestBit));
    Serial.print(" : ");

    if (getRegisterName(registerAddress, interestBit) == "UVW_MUX (Address 0x25, bit 7)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "UVW enabled" : "-A-B-Z enabled"));
    if (getRegisterName(registerAddress, interestBit) == "ABZ_MUX (Address 0x29, bit 6)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "ABZ enabled" : "UVW enabled"));
    if (getRegisterName(registerAddress, interestBit) == "DIR (Address 0x29, bit 1)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "Output Direction CCW" : "Output Direction CW"));
    if (getRegisterName(registerAddress, interestBit) == "UVW_RES (Address 0x30, bit 7:4)") Serial.println("Output Resolutions UVW " + String(getOutputResolutionUVW()));
    if (getRegisterName(registerAddress, interestBit) == "ABZ_RES (Address 0x30, bit 1:0 & Address 0x31, bit 7:0)") Serial.println("Output Resolutions ABZ " + String(getOutputResolutionABZ()));
    if (getRegisterName(registerAddress, interestBit) == "Z_PULSE_WIDTH (Address 0x32, bit 6:4)") Serial.println("(Value = " + String(getZPulseWidth()) + ")  Z Pulse width " + pulseWidthToString(getZPulseWidth()));
    if (getRegisterName(registerAddress, interestBit) == "ZERO (Address 0x32, bit 3:0 & Address 0x33, bit 7:0)") Serial.println(" Zero degree position " + String(getZeroDegreePositionData()));
    if (getRegisterName(registerAddress, interestBit) == "PWM_FREQ (Address 0x38, bit 7)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "994.4 Hz" : "497.2 Hz"));
    if (getRegisterName(registerAddress, interestBit) == "PWM_POL (Address 0x38, bit 6)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "High Level Valid" : "Low Level Valid"));
    if (getRegisterName(registerAddress, interestBit) == "OUT_MODE (Address 0x38, bit 5)") Serial.println("(Value = " + String(interestBitValue) + ") " + (interestBitValue == 0 ? "Analog Output" : "PWM Output"));    

}

void MT6701I2C::printAllRegisterStates() {
  printRegisterState(MT6701_I2C_EEPROM_UVW_MUX_REG, MT6701_I2C_EEPROM_UVW_MUX_BIT);
  printRegisterState(MT6701_I2C_EEPROM_ABZ_MUX_REG, MT6701_I2C_EEPROM_ABZ_MUX_BIT);
  printRegisterState(MT6701_I2C_EEPROM_DIR_REG, MT6701_I2C_EEPROM_DIR_BIT);
  printRegisterState(MT6701_I2C_EEPROM_UVW_RES_REG, MT6701_I2C_EEPROM_UVW_MUX_BIT_S);
  printRegisterState(MT6701_I2C_EEPROM_ABZ_RES_REG_H, MT6701_I2C_EEPROM_ABZ_MUX_BIT_S);
  //printRegisterState(MT6701_I2C_EEPROM_ABZ_RES_REG_L, MT6701_I2C_EEPROM_ABZ_MUX_BIT_S);
  //printRegisterState(MT6701_I2C_EEPROM_HYST_REG_H, 0);
  //printRegisterState(MT6701_I2C_EEPROM_HYST_REG_L, 0);
  printRegisterState(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S);
  printRegisterState(MT6701_I2C_EEPROM_ZERO_REG_H, 0);
  //printRegisterState(MT6701_I2C_EEPROM_ZERO_REG_L, 0);
  printRegisterState(MT6701_I2C_EEPROM_PWM_FREQ_REG, MT6701_I2C_EEPROM_PWM_FREQ_BIT);
  printRegisterState(MT6701_I2C_EEPROM_PWM_POL_REG, MT6701_I2C_EEPROM_PWM_POL_BIT);
  printRegisterState(MT6701_I2C_EEPROM_OUT_MODE_REG, MT6701_I2C_EEPROM_OUT_MODE_BIT);
  //printRegisterState(MT6701_I2C_EEPROM_A_START_REG_H, 0);
  //printRegisterState(MT6701_I2C_EEPROM_A_START_REG_L, 0);
  //printRegisterState(MT6701_I2C_EEPROM_A_STOP_REG_H, MT6701_I2C_EEPROM_A_STOP_BIT_S);
  //printRegisterState(MT6701_I2C_EEPROM_A_STOP_REG_L, MT6701_I2C_EEPROM_A_STOP_BIT_S);
  //printRegisterState(MT6701_I2C_EEPROM_PROG_KEY_REG, 0);
  //printRegisterState(MT6701_I2C_EEPROM_PROG_CMD_REG, 0);
}

const MT6701I2C::RegisterInfo MT6701I2C::registerMap[] = {
  {MT6701_I2C_EEPROM_UVW_MUX_REG, MT6701_I2C_EEPROM_UVW_MUX_BIT, "UVW_MUX (Address 0x25, bit 7)"},
  {MT6701_I2C_EEPROM_ABZ_MUX_REG, MT6701_I2C_EEPROM_ABZ_MUX_BIT, "ABZ_MUX (Address 0x29, bit 6)"},
  {MT6701_I2C_EEPROM_DIR_REG, MT6701_I2C_EEPROM_DIR_BIT, "DIR (Address 0x29, bit 1)"},
  {MT6701_I2C_EEPROM_UVW_RES_REG, MT6701_I2C_EEPROM_UVW_MUX_BIT_S, "UVW_RES (Address 0x30, bit 7:4)"},
  {MT6701_I2C_EEPROM_ABZ_RES_REG_H, MT6701_I2C_EEPROM_ABZ_MUX_BIT_S, "ABZ_RES (Address 0x30, bit 1:0 & Address 0x31, bit 7:0)"},
  //{MT6701_I2C_EEPROM_ABZ_RES_REG_H, MT6701_I2C_EEPROM_ABZ_MUX_BIT_S, "ABZ_RES_REG_H (Address 0x30, bit 1:0 & Address 0x31, bit 7:0)"},
  //{MT6701_I2C_EEPROM_ABZ_RES_REG_L, MT6701_I2C_EEPROM_ABZ_MUX_BIT_S, "ABZ_RES_REG_L (Address 0x30, bit 1:0 & Address 0x31, bit 7:0)"},
  //{MT6701_I2C_EEPROM_HYST_REG_H, 0, "HYST_REG_H (Address 0x32, bit 7 & Address 0x34, bit 7:6)"}, // abilitato in conflitto con reg ZERO
  //{MT6701_I2C_EEPROM_HYST_REG_L, 0, "HYST_REG_L (Address 0x32, bit 7 & Address 0x34, bit 7:6)"},  // abilitato in conflitto con reg ZERO
  {MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S, "Z_PULSE_WIDTH (Address 0x32, bit 6:4)"},
  {MT6701_I2C_EEPROM_ZERO_REG_H, 0, "ZERO (Address 0x32, bit 3:0 & Address 0x33, bit 7:0)"},
  //{MT6701_I2C_EEPROM_ZERO_REG_H, 0, "ZERO_REG_H (Address 0x32, bit 3:0 & Address 0x33, bit 7:0)"},
  //{MT6701_I2C_EEPROM_ZERO_REG_L, 0, "ZERO_REG_L (Address 0x32, bit 3:0 & Address 0x33, bit 7:0)"},
  {MT6701_I2C_EEPROM_PWM_FREQ_REG, MT6701_I2C_EEPROM_PWM_FREQ_BIT, "PWM_FREQ (Address 0x38, bit 7)"},
  {MT6701_I2C_EEPROM_PWM_POL_REG, MT6701_I2C_EEPROM_PWM_POL_BIT, "PWM_POL (Address 0x38, bit 6)"},
  {MT6701_I2C_EEPROM_OUT_MODE_REG, MT6701_I2C_EEPROM_OUT_MODE_BIT, "OUT_MODE (Address 0x38, bit 5)"},
  {MT6701_I2C_EEPROM_A_START_REG_H, 0, "A_START_REG_H (Address 0x3E, bit 3:0 & Address 0x3F, bit 7:0)"},
  {MT6701_I2C_EEPROM_A_START_REG_L, 0, "A_START_REG_L (Address 0x3E, bit 3:0 & Address 0x3F, bit 7:0)"},
  {MT6701_I2C_EEPROM_A_STOP_REG_H, MT6701_I2C_EEPROM_A_STOP_BIT_S, "A_STOP_REG_H (Address 0x3E, bit 7:4 & Address 0x40, bit 7:0)"},
  {MT6701_I2C_EEPROM_A_STOP_REG_L, MT6701_I2C_EEPROM_A_STOP_BIT_S, "A_STOP_REG_L (Address 0x3E, bit 7:4 & Address 0x40, bit 7:0)"},
  {MT6701_I2C_EEPROM_PROG_KEY_REG, 0, "PROG_KEY_REG"},
  {MT6701_I2C_EEPROM_PROG_CMD_REG, 0, "PROG_CMD_REG"},
};

const int MT6701I2C::registerMapSize = sizeof(MT6701I2C::registerMap) / sizeof(MT6701I2C::RegisterInfo);

const char* MT6701I2C::getRegisterName(uint8_t registerAddress, uint8_t interestBit) {
  for (int i = 0; i < registerMapSize; i++) {
    if (registerMap[i].registerAddress == registerAddress && registerMap[i].interestBit == interestBit) {
      return registerMap[i].name;
    }
  }
  return "Unknown Register or Bit";
}

String MT6701I2C::pulseWidthToString(MT6701I2CZPulseWidth pulseWidth) {
  switch(pulseWidth) {
    case MT6701I2_Z_PULSE_WIDTH_1LSB: return "1LSB";
    case MT6701I2_Z_PULSE_WIDTH_2LSB: return "2LSB";
    case MT6701I2_Z_PULSE_WIDTH_4LSB: return "4LSB";
    case MT6701I2_Z_PULSE_WIDTH_8LSB: return "8LSB";
    case MT6701I2_Z_PULSE_WIDTH_12LSB: return "12LSB";
    case MT6701I2_Z_PULSE_WIDTH_16LSB: return "16LSB";
    case MT6701I2_Z_PULSE_WIDTH_180DEG: return "180DEG";
    case MT6701I2_Z_PULSE_WIDTH_1LSB_2: return "1LSB_2";
    default: return "Unknown";
  }
}

void MT6701I2C::printAllRegisterValue() {
  for (uint8_t i = 0x25; i <= 0x40; i++) {
    Serial.print("Register 0x");
    Serial.print(i, HEX);
    Serial.print(": ");
    for (int8_t bit = 7; bit >= 0; bit--) {
      uint8_t bitValue = (MT_RequestSingleRegister(i) >> bit) & 0x01;
      Serial.print(bitValue);
    }
    Serial.println();
  }
}

/*
 * @brief: scrivere un valore di 1 byte in un registro arbitrario di 1 byte
 * @param _reg_addr: indirizzo del registro
 * @param _payload: 1 byte (8 bit) da scrivere
 * esempio: MT_WriteOneRegister(0x40, 0b00000000);
 */
void MT6701I2C::MT_WriteOneRegister(uint8_t _reg_addr, uint8_t _payload) {
  // Inizia la trasmissione all'indirizzo per inviare un byte di dati al registro
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  _wire_->write(_reg_addr);
  _wire_->write(_payload);
  // Termina la connessione
  _wire_->endTransmission();
}
