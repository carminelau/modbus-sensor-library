#include "ModbusSensorLibrary.h"

// Definizione comandi Modbus statici
const uint8_t ModbusSensorLibrary::_modbusRequest_ane[] = {
  0x01, 0x03, 0x00, 0x00, 0x00, 0x60, 0x45, 0xE2
};

const uint8_t ModbusSensorLibrary::_modbusRequest_soil[] = {
  0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08
};

// Costruttore
ModbusSensorLibrary::ModbusSensorLibrary(uint8_t rxPin, uint8_t txPin,
                                       uint8_t dePin, uint8_t rePin)
  : _rxPin(rxPin), _txPin(txPin), _dePin(dePin), _rePin(rePin),
    _debugEnabled(false) {
  _rs485 = new SoftwareSerial(_rxPin, _txPin);
}

// Distruttore
ModbusSensorLibrary::~ModbusSensorLibrary() {
  if (_rs485) {
    delete _rs485;
  }
}

// Inizializzazione
void ModbusSensorLibrary::begin() {
  pinMode(_dePin, OUTPUT);
  pinMode(_rePin, OUTPUT);
  setTransmitMode(false);

  if (_debugEnabled) {
    Serial.println("[INFO] ModbusSensorLibrary inizializzata");
  }
}

// Controllo modalità trasmissione RS485
void ModbusSensorLibrary::setTransmitMode(bool enable) {
  digitalWrite(_dePin, enable ? HIGH : LOW);
  digitalWrite(_rePin, enable ? HIGH : LOW);
}

// Conversione Little Endian a Short
unsigned short ModbusSensorLibrary::convertToShortLittleEndian(const uint8_t* data) {
  return (data[0] << 8) | data[1];
}

// Conversione Little Endian a Float
float ModbusSensorLibrary::convertToFloatLittleEndian(const uint8_t* data) {
  union { uint32_t i; float f; } u;
  u.i = (uint32_t)data[2] << 24 | (uint32_t)data[3] << 16 |
        (uint32_t)data[0] << 8 | data[1];
  return u.f;
}

// Calcolo CRC16 per Modbus
uint16_t ModbusSensorLibrary::calculateCRC16(uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Invio richiesta Modbus generica
bool ModbusSensorLibrary::sendModbusRequest(const uint8_t* request, size_t requestLen,
                                           uint8_t* response, size_t* responseLen,
                                           unsigned long baud, unsigned long timeout) {
  _rs485->begin(baud);
  delay(100);

  setTransmitMode(true);
  delay(2);
  _rs485->write(request, requestLen);
  _rs485->flush();
  delay(2);
  setTransmitMode(false);

  unsigned long startTime = millis();
  size_t idx = 0;

  while (millis() - startTime < timeout && idx < *responseLen - 1) {
    if (_rs485->available()) {
      response[idx++] = _rs485->read();
    }
  }

  *responseLen = idx;

  if (idx > 0 && response[0] == 0x01) {
    if (_debugEnabled) {
      Serial.print("[DEBUG] Risposta ricevuta (");
      Serial.print(idx);
      Serial.print(" bytes): ");
      for (size_t i = 0; i < idx; i++) {
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    return true;
  }

  if (_debugEnabled) {
    Serial.println("[ERROR] Nessuna risposta valida ricevuta");
  }
  return false;
}

// Lettura dati anemometro
AnemometerData ModbusSensorLibrary::readAnemometer(unsigned long baud) {
  AnemometerData data = {0};
  uint8_t response[128];
  size_t responseLen = sizeof(response);

  if (sendModbusRequest(_modbusRequest_ane, sizeof(_modbusRequest_ane),
                       response, &responseLen, baud)) {

    if (responseLen >= 23) {
      data.windDirection = convertToShortLittleEndian(&response[5]);
      data.windSpeed = convertToFloatLittleEndian(&response[7]);
      data.temperature = convertToFloatLittleEndian(&response[11]);
      data.humidity = convertToFloatLittleEndian(&response[15]);
      data.pressure = convertToFloatLittleEndian(&response[19]);
      data.valid = true;

      if (_debugEnabled) {
        Serial.println("[INFO] Dati anemometro letti con successo");
      }
    } else {
      if (_debugEnabled) {
        Serial.println("[ERROR] Risposta anemometro troppo corta");
      }
    }
  }

  return data;
}

// Lettura dati sensore suolo
SoilSensorData ModbusSensorLibrary::readSoilSensor(unsigned long baud) {
  SoilSensorData data = {0};
  uint8_t response[128];
  size_t responseLen = sizeof(response);

  if (sendModbusRequest(_modbusRequest_soil, sizeof(_modbusRequest_soil),
                       response, &responseLen, baud)) {

    if (responseLen >= 19) {
      // Verifica CRC
      uint16_t receivedCRC = response[17] | (response[18] << 8);
      uint16_t calculatedCRC = calculateCRC16(response, 17);

      if (receivedCRC == calculatedCRC) {
        data.humidity = ((response[3] << 8) | response[4]) * 0.1;
        data.temperature = ((response[5] << 8) | response[6]) * 0.1;
        data.ec = (response[7] << 8) | response[8];
        data.ph = ((response[9] << 8) | response[10]) * 0.1;
        data.nitrogen = (response[11] << 8) | response[12];
        data.phosphorus = (response[13] << 8) | response[14];
        data.potassium = (response[15] << 8) | response[16];
        data.valid = true;

        if (_debugEnabled) {
          Serial.println("[INFO] Dati sensore suolo letti con successo");
        }
      } else {
        if (_debugEnabled) {
          Serial.println("[ERROR] CRC sensore suolo non valido");
        }
      }
    } else {
      if (_debugEnabled) {
        Serial.println("[ERROR] Risposta sensore suolo troppo corta");
      }
    }
  }

  return data;
}

// Stampa dati anemometro
void ModbusSensorLibrary::printAnemometerData(const AnemometerData& data) {
  if (data.valid) {
    Serial.println("📊 Dati Anemometro:");
    Serial.print(" - Direzione vento: "); Serial.print(data.windDirection); Serial.println("°");
    Serial.print(" - Velocità vento:  "); Serial.print(data.windSpeed); Serial.println(" m/s");
    Serial.print(" - Temperatura:     "); Serial.print(data.temperature); Serial.println(" °C");
    Serial.print(" - Umidità:         "); Serial.print(data.humidity); Serial.println(" %");
    Serial.print(" - Pressione:       "); Serial.print(data.pressure); Serial.println(" hPa");
  } else {
    Serial.println("[ERROR] Dati anemometro non validi");
  }
}

// Stampa dati sensore suolo
void ModbusSensorLibrary::printSoilSensorData(const SoilSensorData& data) {
  if (data.valid) {
    Serial.println("🌱 Dati Sensore Suolo:");
    Serial.print(" - Umidità:         "); Serial.print(data.humidity); Serial.println(" %");
    Serial.print(" - Temperatura:     "); Serial.print(data.temperature); Serial.println(" °C");
    Serial.print(" - EC:              "); Serial.print(data.ec); Serial.println(" µS/cm");
    Serial.print(" - pH:              "); Serial.print(data.ph); Serial.println();
    Serial.print(" - Azoto (N):       "); Serial.print(data.nitrogen); Serial.println(" mg/kg");
    Serial.print(" - Fosforo (P):     "); Serial.print(data.phosphorus); Serial.println(" mg/kg");
    Serial.print(" - Potassio (K):    "); Serial.print(data.potassium); Serial.println(" mg/kg");
  } else {
    Serial.println("[ERROR] Dati sensore suolo non validi");
  }
}

// Abilita/disabilita debug
void ModbusSensorLibrary::enableDebug(bool enable) {
  _debugEnabled = enable;
}
