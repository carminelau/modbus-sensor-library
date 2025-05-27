# ModbusSensorLibrary
A professional Arduino/PlatformIO library for communicating with Modbus RS485 sensors, specifically designed for anemometers and soil sensors.

## Features

- ✅ Multi-platform support (Arduino, ESP32, ESP8266)
- ✅ RS485 Modbus RTU communication
- ✅ Automatic CRC validation and data verification
- ✅ Typed data structures for each sensor
- ✅ Integrated debug with formatted output
- ✅ Advanced error handling
- ✅ Configurable timeout
- ✅ Simple and intuitive API
- ✅ Memory efficient
- ✅ Non-blocking operations with yield() support

## Supported Sensors
- Anemometer
    - Wind speed (m/s)
    - Wind direction (degrees)
    - Temperature (°C)
    - Humidity (%)
    - Atmospheric pressure (hPa)

- Soil Sensor
    - Soil moisture (%)
    - Soil temperature (°C)
    - Electrical conductivity (µS/cm)
    - pH level
    - Nitrogen content (mg/kg)
    - Phosphorus content (mg/kg)
    - Potassium content (mg/kg)