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
 * Traduzione in italiano /Ennio Sesana /2024
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Impostazioni del bus I2C del sensore ===*/
const uint32_t MT6701_I2C_CLOCK_100KHZ = 100000;
const uint32_t MT6701_I2C_CLOCK_400KHZ = 400000;
const uint32_t MT6701_I2C_CLOCK_1MHZ   = 1000000;
const uint8_t MT6701_I2C_ADDRESS = 0x06;

/*=== Pin MODE su diverse schede (dipende dal core) ===*/
#define STM32_MT6701_MODE_PIN   PC13
#define ESP8266_MT6701_MODE_PIN 2
#define ESP32_MT6701_MODE_PIN   4
#define ARDUINO_MT6701_MODE_PIN 3

/*=== Indirizzi dei registri del sensore ===*/
// Registro dati angolo
const uint8_t MT6701_I2C_ANGLE_DATA_REG_H = 0x03;
const uint8_t MT6701_I2C_ANGLE_DATA_REG_L = 0x04;
// UVW_MUX solo per il pacchetto QFN
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_REG = 0x25;
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_BIT = 7;
// ABZ_MUX
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_REG = 0x29;
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_BIT = 6;
// DIR
const uint8_t MT6701_I2C_EEPROM_DIR_REG = 0x29;
const uint8_t MT6701_I2C_EEPROM_DIR_BIT = 1;
// UVW_RES
const uint8_t MT6701_I2C_EEPROM_UVW_RES_REG   = 0x30;
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_BIT_S = 4;
// ABZ_RES
const uint8_t MT6701_I2C_EEPROM_ABZ_RES_REG_H = 0x30;
const uint8_t MT6701_I2C_EEPROM_ABZ_RES_REG_L = 0x31;
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_BIT_S = 0;
// HYST
const uint8_t MT6701_I2C_EEPROM_HYST_REG_H = 0x32;
const uint8_t MT6701_I2C_EEPROM_HYST_REG_L = 0x34;
// Larghezza dell'impulso Z
const uint8_t MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG   = 0x32;
const uint8_t MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S = 4;
// ZERO
const uint8_t MT6701_I2C_EEPROM_ZERO_REG_H = 0x32;
const uint8_t MT6701_I2C_EEPROM_ZERO_REG_L = 0x33;
// Frequenza PWM
const uint8_t MT6701_I2C_EEPROM_PWM_FREQ_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_PWM_FREQ_BIT = 7;
// Polarità PWM
const uint8_t MT6701_I2C_EEPROM_PWM_POL_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_PWM_POL_BIT = 6;
// Modalità di uscita
const uint8_t MT6701_I2C_EEPROM_OUT_MODE_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_OUT_MODE_BIT = 5;
// A_START
const uint8_t MT6701_I2C_EEPROM_A_START_REG_H = 0x3E;
const uint8_t MT6701_I2C_EEPROM_A_START_REG_L = 0x3F;
// A_STOP
const uint8_t MT6701_I2C_EEPROM_A_STOP_REG_H = 0x3E;
const uint8_t MT6701_I2C_EEPROM_A_STOP_REG_L = 0x40;
const uint8_t MT6701_I2C_EEPROM_A_STOP_BIT_S = 4;
// 7.2 Programmazione EEPROM
const uint8_t MT6701_I2C_EEPROM_PROG_KEY_REG   = 0x09;
const uint8_t MT6701_I2C_EEPROM_PROG_KEY_VALUE = 0xB3;
const uint8_t MT6701_I2C_EEPROM_PROG_CMD_REG   = 0x0A;
const uint8_t MT6701_I2C_EEPROM_PROG_CMD_VALUE = 0x05;

/*=== Valori ausiliari ===*/
// Tipo di configurazione dell'interfaccia di uscita (solo per il pacchetto QFN)
enum MT6701I2CConfigurationOutputType {
  MT6701I2_CONFIG_OUTPUT_TYPE_UVW,
  MT6701I2_CONFIG_OUTPUT_TYPE_A_B_Z,
};
// Tipo di interfaccia di uscita
enum MT6701I2COutputType {
  MT6701I2_OUTPUT_TYPE_ABZ,
  MT6701I2_OUTPUT_TYPE_UVW,
};
// Direzione di rotazione positiva
enum MT6701I2CDirection {
  MT6701I2_DIRECTION_COUNTERCLOCKWISE, // Contro orario
  MT6701I2_DIRECTION_CLOCKWISE, // Orario
};

// Larghezza dell'impulso Z
enum MT6701I2CZPulseWidth {
  MT6701I2_Z_PULSE_WIDTH_1LSB,
  MT6701I2_Z_PULSE_WIDTH_2LSB,
  MT6701I2_Z_PULSE_WIDTH_4LSB,
  MT6701I2_Z_PULSE_WIDTH_8LSB,
  MT6701I2_Z_PULSE_WIDTH_12LSB,
  MT6701I2_Z_PULSE_WIDTH_16LSB,
  MT6701I2_Z_PULSE_WIDTH_180DEG,
  MT6701I2_Z_PULSE_WIDTH_1LSB_2,
};
// Frequenza PWM
enum MT6701I2CFrequencyPWM {
  MT6701I2_PWM_FREQUENCY_9944,
  MT6701I2_PWM_FREQUENCY_4972,
};
// Polarità PWM
enum MT6701I2CPolarityPWM {
  MT6701I2_PWM_POLARITY_HIGH,
  MT6701I2_PWM_POLARITY_LOW,
};
// Modalità di uscita
enum MT6701I2COutputMode {
  MT6701I2_OUTPUT_MODE_ANALOG,
  MT6701I2_OUTPUT_MODE_PWM,
};
// Risposte standard di successo/errore
const uint8_t MT6701I2C_DEFAULT_REPORT_ERROR = 0;
const uint8_t MT6701I2C_DEFAULT_REPORT_OK    = 1;
// Selezione delle interfacce del sensore
const uint8_t MT6701I2C_MODE_I2C_SSI = 0;
const uint8_t MT6701I2C_MODE_UVW_ABZ = 1;


class MT6701I2C {
  private:
    TwoWire* _wire_; // Oggetto per l'uso dei metodi I2C
    int8_t _pin_mode_ = -1; // Contatto del microcontrollore a cui è collegato il pin MODE del sensore

  protected:
    uint8_t MT_RequestSingleRegister(uint8_t _reg_addr); // Richiesta del valore di un registro di dimensione 1 byte
    void MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload); // Scrittura di un byte in un registro di un byte

  struct RegisterInfo {
    uint8_t registerAddress;
    uint8_t interestBit;
    const char* name;
  };

  String pulseWidthToString(MT6701I2CZPulseWidth pulseWidth);

  static const RegisterInfo registerMap[];
  static const int registerMapSize;

  const char* getRegisterName(uint8_t registerAddress, uint8_t interestBit); //restituisce nome del registro e il bit di interesse

  public:
    MT6701I2C(TwoWire* _twi); // Costruttore con solo l'uso dell'interfaccia I2C

    void begin(void); // Chiamata a Wire.begin()
#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_STM32)
    void begin(int8_t _sda_pin, int8_t _scl_pin); // Chiamata a Wire.begin(SDA, SCL) specificando i pin
#endif
    void setClock(uint32_t _clock = MT6701_I2C_CLOCK_400KHZ); // Impostazione della frequenza a 100kHz, 400kHz, 1MHz, o un valore personalizzato (default 400kHz)

    void saveNewValues(void); // Metodo del produttore per salvare i valori nella memoria EEPROM. Si raccomanda di eseguirlo con una tensione di alimentazione tra 4.5V e 5.5V

    bool isConnected(void); // Verifica secondo l'algoritmo standard di ricerca dei dispositivi sulla linea I2C

    void attachModePin(byte _pin_mode); // Assegna un pin del microcontrollore per controllare la modalità dell'interfaccia
    void detachModePin(void); // Libera il pin assegnato del microcontrollore per controllare la modalità dell'interfaccia

    void enableI2CorSSI(void); // Abilita l'interfaccia I2C/SSI. MT6701I2C_MODE_I2C_SSI
    void enableUVWorABZ(void); // Abilita l'interfaccia UVW/ABZ. MT6701I2C_MODE_UVW_ABZ

    word getRawAngle(void); // Ottiene l'angolo in forma grezza. 0 - 16383
    float getDegreesAngle(void); // Ottiene l'angolo in gradi. 0.00 - 359.98
    float getRadiansAngle(void); // Ottiene l'angolo in radianti. 0.00 - 6.28 

    MT6701I2CConfigurationOutputType getConfigurationOutputType(void); // Ottiene il tipo di configurazione dell'interfaccia di uscita (solo per il pacchetto QFN). MT6701I2_CONFIG_OUTPUT_TYPE_UVW, MT6701I2_CONFIG_OUTPUT_TYPE_A_B_Z
    void setConfigurationOutputTypeABZ(void); // Imposta il tipo di configurazione dell'interfaccia di uscita -A-B-Z (solo per il pacchetto QFN)
    bool setConfigurationOutputTypeABZVerify(void); // Lo stesso, ma con conferma (solo per il pacchetto QFN)
    void setConfigurationOutputTypeUVW(void); // Imposta il tipo di configurazione dell'interfaccia di uscita UVW (solo per il pacchetto QFN)
    bool setConfigurationOutputTypeUVWVerify(void); // Lo stesso, ma con conferma (solo per il pacchetto QFN)

MT6701I2COutputType getOutputType(void); // Ottieni il tipo di interfaccia di uscita. MT6701I2_OUTPUT_TYPE_ABZ, MT6701I2_OUTPUT_TYPE_UVW
    void setOutputTypeABZ(void); // Imposta il tipo di interfaccia di uscita ABZ
    bool setOutputTypeABZVerify(void); // Lo stesso, ma con conferma
    void setOutputTypeUVW(void); // Imposta il tipo di interfaccia di uscita UVW
    bool setOutputTypeUVWVerify(void); // Lo stesso, ma con conferma

    MT6701I2CDirection getOutputRotationDirection(void); // Ottieni la direzione di rotazione. MT6701I2_DIRECTION_COUNTERCLOCKWISE, MT6701I2_DIRECTION_CLOCKWISE
    void setOutputRotationDirectionCounterclockwise(void); // Imposta la direzione di rotazione in senso antiorario
    bool setOutputRotationDirectionCounterclockwiseVerify(void); // Lo stesso, ma con conferma
    void setOutputRotationDirectionClockwise(void); // Imposta la direzione di rotazione in senso orario
    bool setOutputRotationDirectionClockwiseVerify(void); // Lo stesso, ma con conferma

    byte getOutputResolutionUVW(void); // Ottieni il valore della risoluzione di uscita in modalità UVW. 1 - 16
    void setOutputResolutionUVW(byte _resolution); // Imposta il valore della risoluzione di uscita in modalità UVW. 1 - 16
    bool setOutputResolutionUVWVerify(byte _resolution); // Lo stesso, ma con conferma

    word getOutputResolutionABZ(void); // Ottieni il valore della risoluzione di uscita in modalità ABZ. 1 - 1024
    void setOutputResolutionABZ(word _resolution); // Imposta il valore della risoluzione di uscita in modalità ABZ. 1 - 1024
    bool setOutputResolutionABZVerify(word _resolution); // Lo stesso, ma con conferma

    MT6701I2CZPulseWidth getZPulseWidth(void); // Ottieni il valore della larghezza dell'impulso sul pin Z in modalità ABZ. MT6701I2_Z_PULSE_WIDTH_1LSB, MT6701I2_Z_PULSE_WIDTH_2LSB,
    // MT6701I2_Z_PULSE_WIDTH_4LSB, MT6701I2_Z_PULSE_WIDTH_8LSB, MT6701I2_Z_PULSE_WIDTH_12LSB, MT6701I2_Z_PULSE_WIDTH_16LSB, MT6701I2_Z_PULSE_WIDTH_180DEG, MT6701I2_Z_PULSE_WIDTH_1LSB_2,
    void setZPulseWidth1LSB(void); // Imposta la larghezza dell'impulso a 1LSB
    bool setZPulseWidth1LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth2LSB(void); // Imposta la larghezza dell'impulso a 2LSB
    bool setZPulseWidth2LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth4LSB(void); // Imposta la larghezza dell'impulso a 4LSB
    bool setZPulseWidth4LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth8LSB(void); // Imposta la larghezza dell'impulso a 8LSB
    bool setZPulseWidth8LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth12LSB(void); // Imposta la larghezza dell'impulso a 12LSB
    bool setZPulseWidth12LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth16LSB(void); // Imposta la larghezza dell'impulso a 16LSB
    bool setZPulseWidth16LSBVerify(void); // Lo stesso, ma con conferma
    void setZPulseWidth180DEG(void); // Imposta la larghezza dell'impulso a 180 gradi
    bool setZPulseWidth180DEGVerify(void); // Lo stesso, ma con conferma

    word getZeroDegreePositionData(void); // Ottieni il valore della posizione a zero gradi. Vedi tabella nella documentazione pag 28. 0x000 - 0xFFF
    void setZeroDegreePositionData(word _zero_position_data); // Imposta il valore della posizione a zero gradi. Vedi tabella nella documentazione
    bool setZeroDegreePositionDataVerify(word _zero_position_data); // Lo stesso, ma con conferma

    MT6701I2CFrequencyPWM getFrequencyPWM(void); // Ottieni il valore della frequenza PWM. MT6701I2_PWM_FREQUENCY_9944, MT6701I2_PWM_FREQUENCY_4972
    void setFrequencyPWM9944(void); // Imposta la frequenza PWM a 994.4Hz
    bool setFrequencyPWM9944Verify(void); // Lo stesso, ma con conferma
    void setFrequencyPWM4972(void); // Imposta la frequenza PWM a 497.2Hz
    bool setFrequencyPWM4972Verify(void); // Lo stesso, ma con conferma

MT6701I2CPolarityPWM getPolarityPWM(void); // Ottieni il valore della polarità PWM. MT6701I2_PWM_POLARITY_HIGH, MT6701I2_PWM_POLARITY_LOW
    void setPolarityPWMHigh(void); // Imposta la polarità PWM HIGH
    bool setPolarityPWMHighVerify(void); // Lo stesso, ma con conferma
    void setPolarityPWMLow(void); // Imposta la polarità PWM LOW
    bool setPolarityPWMLowVerify(void); // Lo stesso, ma con conferma

    MT6701I2COutputMode getOutputMode(void); // Ottieni la modalità di uscita. MT6701I2_OUTPUT_MODE_ANALOG, MT6701I2_OUTPUT_MODE_PWM
    void setOutputModeAnalog(void); // Imposta la modalità di uscita Analogica
    bool setOutputModeAnalogVerify(void); // Lo stesso, ma con conferma
    void setOutputModePWM(void); // Imposta la modalità di uscita PWM
    bool setOutputModePWMVerify(void); // Lo stesso, ma con conferma

    // Metodo per leggere lo stato di un singolo registro
    void printRegisterState(uint8_t registerAddress, uint8_t interestBit);

    // Metodo per leggere lo stato di tutti i registri
    void printAllRegisterStates();

    // Metodo per leggere i valori di tutti i registri
   void printAllRegisterValue();

    // Metodo per scrivere i tutti i valori di un registro
    void MT_WriteOneRegister(uint8_t _reg_addr, uint8_t _payload);
};
