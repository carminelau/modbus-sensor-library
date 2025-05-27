#ifndef MODBUS_SENSOR_LIBRARY_H
#define MODBUS_SENSOR_LIBRARY_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// Strutture dati per i sensori
struct AnemometerData {
  short windDirection;      // Direzione vento in gradi
  float windSpeed;          // Velocità vento in m/s
  float temperature;        // Temperatura in °C
  float humidity;           // Umidità in %
  float pressure;           // Pressione in hPa
  bool valid;               // Flag di validità dati
};

struct SoilSensorData {
  float humidity;           // Umidità suolo in %
  float temperature;        // Temperatura suolo in °C
  int ec;                   // Conducibilità elettrica in µS/cm
  float ph;                 // pH del suolo
  int nitrogen;             // Azoto in mg/kg
  int phosphorus;           // Fosforo in mg/kg
  int potassium;            // Potassio in mg/kg
  bool valid;               // Flag di validità dati
};

class ModbusSensorLibrary {
  private:
    // Pin RS485
    uint8_t _dePin;
    uint8_t _rePin;
    uint8_t _rxPin;
    uint8_t _txPin;

    // Software Serial per RS485
    SoftwareSerial* _rs485;

    // Comandi Modbus
    static const uint8_t _modbusRequest_ane[];
    static const uint8_t _modbusRequest_soil[];

    // Metodi privati
    void setTransmitMode(bool enable);
    unsigned short convertToShortLittleEndian(const uint8_t* data);
    float convertToFloatLittleEndian(const uint8_t* data);
    uint16_t calculateCRC16(uint8_t* data, int length);
    bool sendModbusRequest(const uint8_t* request, size_t requestLen,
                          uint8_t* response, size_t* responseLen,
                          unsigned long baud, unsigned long timeout = 300);

  public:
    // Costruttore
    ModbusSensorLibrary(uint8_t rxPin, uint8_t txPin, uint8_t dePin, uint8_t rePin);

    // Distruttore
    ~ModbusSensorLibrary();

    // Inizializzazione
    void begin();

    // Lettura sensori
    AnemometerData readAnemometer(unsigned long baud = 9600);
    SoilSensorData readSoilSensor(unsigned long baud = 4800);

    // Metodi di utilità
    void printAnemometerData(const AnemometerData& data);
    void printSoilSensorData(const SoilSensorData& data);

    // Debug
    void enableDebug(bool enable);

  private:
    bool _debugEnabled;
};

#endif
