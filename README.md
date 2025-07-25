# 🌪 MT6701 Arduino Library

Questa libreria consente l'interfacciamento con il sensore magnetico **MagnTek MT6701** tramite Arduino, utilizzando il protocollo I2C. È pensata per misurazioni angolari ad alta precisione con encoder magnetici.

---

## 🧩 Compatibilità

Supporta:
- Arduino Uno, Nano, Mega
- Sensori della stessa famiglia come AS5600 / AS5601

---

## ⚙️ Funzionalità principali

- Lettura angolare ad alta risoluzione
- Accesso diretto ai registri I2C
- Facile integrazione con la libreria Wire

---
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 *
 * Copyright (C) 2022. v1.2 / Licenza MIT / Roman Sklyar S-LAB
 * Traduzione italiana a cura di Ennio SESANA 2024
 * 
 * Nota: nella versiona italiana sono state aggiunte le seguenti utility:
 * void printRegisterState(uint8_t registerAddress, uint8_t interestBit); // istruzione per leggere lo stato di un singolo registro
 * void printAllRegisterStates(); // istruzione per leggere lo stato di tutti i registri
 * void printAllRegisterValue(); // istruzione per leggere i valori di tutti i registri
 * void MT_WriteOneRegister(uint8_t _reg_addr, uint8_t _payload); // istruzione per scrivere i tutti i valori di un registro

 * Sono stati inoltre aggiunti ulteriori Sketch di esempio
