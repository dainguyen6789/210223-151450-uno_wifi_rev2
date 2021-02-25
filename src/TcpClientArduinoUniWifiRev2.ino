#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino.h>
#include <Wire.h>
#include <BME280.h>
#define DELAYTIME 800
char ssid[] = "TANASE";                         //  your network SSID (name)
char pass[] = "3629678427"; // your network password

int status = WL_IDLE_STATUS;
IPAddress server(192, 168, 2, 206); 

// Initialize the client library
WiFiClient client;

int analogPin[6];
float voltageAtAdcPin;
int i;
int32_t humidityData;
int32_t tempData;

/*
DIG_T1: unsigned short
DIG_T2/3: signed short
*/

unsigned short dig_T1;
signed short dig_T2, dig_T3;

// humidity compensation data
unsigned char dig_H1, dig_H3;
signed short dig_H2, dig_H4, dig_H5;
char dig_H6;

void SetupTCP()
{
  Serial.begin(115200);
  Serial.println("Attempting to connect to WPA network...");
  Serial.print("SSID: ");
  Serial.println(ssid);

  status = WiFi.begin(ssid, pass);
  if (status != WL_CONNECTED)
  {
    Serial.println("Couldn't get a wifi connection");
    // don't do anything else:
    while (true)
      ;
  }
  else
  {
    Serial.println("Connected to wifi");
    Serial.println("\nStarting connection...");
    // if you get a connection, report back via serial:
    if (client.connect(server, 80))
    {
      Serial.println("connected");
    }
  }
}

void ConfigAnalogPins()
{
  analogPin[0] = A0;
  analogPin[1] = A1;
  analogPin[2] = A2;
  analogPin[3] = A3;
  analogPin[4] = A4;
  analogPin[5] = A5;
};

void InitBme280I2c()
{
  //Wire.begin(address)
  // address: the 7-bit slave address (optional); if not specified, join the bus as a master.
  Wire.begin();
  Wire.beginTransmission(BME280Address);                      // transmit to device
  Wire.write(BME280_CTRL_MEAS);                               // sets register pointer to the command register (0x00)
  Wire.write(BME280_NORMAL_MODE | BME280_TEMP_OVERAMPLING_1); // sets register pointer to the command register (0x00)
  Wire.endTransmission();

  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(BME280_CTRL_HUM);           // sets register pointer to the command register (0x00)
  Wire.write(BME280_HU_OVERAMPLING_1);   // sets register pointer to the command register (0x00)
  Wire.endTransmission();
}

// 0xFD: hum_msb[7:0] contains msb part of hum[15:8] of the raw humidity measurement output data
// 0xFE: hum_lsb[7:0] contains lsb part of hum[7:0] of the raw humidity measurement output data
int32_t ReadBME280HumidityData()
{
  int32_t reading = 0;
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(BME280_HUMIDITY_REG);       // sets register pointer to the command register (0x00)
  Wire.endTransmission();                // stop transmitting

  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.requestFrom(BME280Address, 2);    // request 2 bytes from slave device
  //Wire.beginTransmission(BME280Address); // transmit to device

  if (2 <= Wire.available())
  { // if two bytes were received

    reading = Wire.read(); // receive high byte (overwrites previous reading)

    reading = reading << 8; // shift high byte to be high 8 bits

    reading |= Wire.read(); // receive low byte as lower 8 bits

    //Serial.println(reading);   // print the reading
  }
  Wire.endTransmission();
  return reading;
}

int32_t ReadBME280TempData()
{
  int32_t reading = 0;
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(BME280_TEMP_REG);           // sets register pointer to the command register (0x00)
  Wire.endTransmission();                // stop transmitting

  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.requestFrom(BME280Address, 3);    // request 2 bytes from slave device
  //Wire.beginTransmission(BME280Address); // transmit to device

  if (3 <= Wire.available())
  { // if two bytes were received

    reading = Wire.read(); // receive high byte (overwrites previous reading)

    reading = reading << 12; // shift high byte to be high 8 bits

    reading |= Wire.read() << 4; // receive low byte as lower 8 bits

    reading |= Wire.read() & (0x0F);

    //Serial.println(reading);   // print the reading
  }
  Wire.endTransmission();
  return reading;
}

int16_t ReadCompRegister(byte registerAddress)
{
  int16_t data = 0;
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(byte(registerAddress));     // sets register pointer to the command register (0x00)
  Wire.endTransmission();                // stop transmitting

  // dig_H3 data
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.requestFrom(BME280Address, 1);    // request 1 bytes from slave device
  //Wire.beginTransmission(BME280Address); // transmit to device
  data = Wire.read();
  Wire.endTransmission(); // stop transmitting

  return data; // receive high byte (overwrites previous reading)
}
unsigned char ReadRegister(unsigned char registerAddress)
{
  unsigned char data = 0;
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(byte(registerAddress));     // sets register pointer to the command register (0x00)
  Wire.endTransmission();                // stop transmitting

  // dig_H3 data
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.requestFrom(BME280Address, 1);    // request 1 bytes from slave device
  //Wire.beginTransmission(BME280Address); // transmit to device
  data = Wire.read();
  Wire.endTransmission(); // stop transmitting
  return data;            // receive high byte (overwrites previous reading)
}

char ReadDigH6Register()
{
  char data = 0;
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.write(DIG_H6_REGISTER_ADDRESS);   // sets register pointer to the command register (0x00)
  Wire.endTransmission();                // stop transmitting

  // dig_H3 data
  Wire.beginTransmission(BME280Address); // transmit to device
  Wire.requestFrom(BME280Address, 1);    // request 1 bytes from slave device
  //Wire.beginTransmission(BME280Address); // transmit to device
  data = Wire.read();
  Wire.endTransmission(); // stop transmitting
  return data;            // receive high byte (overwrites previous reading)
}

void ReadAllHumidityCompRegister()
{
  int16_t dataHi, dataLo;
  dig_H1 = ReadRegister(DIG_H1_REGISTER_ADDRESS);
  // register address 0xE1/0xE2, data dig_H2[7:0]/[15:8]
  dataLo = ReadCompRegister(DIG_H2_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_H2_HI_REGISTER_ADDRESS);

  dig_H2 = (dataHi) << 8 | (dataLo);

  dig_H3 = ReadRegister(DIG_H3_REGISTER_ADDRESS);

  dataHi = ReadCompRegister(DIG_H4_HI_REGISTER_ADDRESS);
  dataLo = ReadCompRegister(DIG_H4_H5_REGISTER_ADDRESS);
  dig_H4 = (dataHi << 4) | (dataLo & 0x000F); // dig_H4[11:4]/[3:0]

  dataHi = ReadCompRegister(DIG_H5_HI_REGISTER_ADDRESS); // dig_H5[11:4]
  dig_H5 = (dataHi << 4) | ((dataLo & 0x00F0) >> 4);     // // dig_H5[11:4]/[3:0]   0xE6,0xE5[7:4]

  dig_H6 = ReadDigH6Register();
}

void ReadDigT1TempCompRegister()
{
  unsigned short dataHi, dataLo;
  dataLo = ReadCompRegister(DIG_T1_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_T1_HI_REGISTER_ADDRESS);
  dig_T1 = (dataHi << 8) | dataLo;
}
void ReadAllTempCompRegister()
{
  int16_t dataHi, dataLo;
  // dataLo=ReadCompRegister(DIG_T1_LO_REGISTER_ADDRESS);
  // dataHi=ReadCompRegister(DIG_T1_HI_REGISTER_ADDRESS);
  // dig_T1=(dataHi<<8)|dataLo;

  // register address 0xE1/0xE2, data dig_H2[7:0]/[15:8]
  dataLo = ReadCompRegister(DIG_T2_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_T2_HI_REGISTER_ADDRESS);
  dig_T2 = (dataHi << 8) | dataLo;

  dataLo = ReadCompRegister(DIG_T3_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_T3_HI_REGISTER_ADDRESS);
  dig_T3 = (dataHi << 8) | dataLo;
}

void setup()
{
  //Serial.begin(9600);           //  setup serial
  ConfigAnalogPins();
  InitBme280I2c();
  SetupTCP();
}

void loop()
{
  String  TcpStringData;
  // send the Gas Sensor ADC value to the SerialPort
  for (i = 0; i < 6; i++)
  {
    voltageAtAdcPin = (float)analogRead(analogPin[i]) / 1024 * 5; // read the input pin
    TcpStringData=String(i + 1)+String(voltageAtAdcPin,4);
    client.println(TcpStringData);
    client.flush();

    delay(DELAYTIME);
  }
  tempData = ReadBME280TempData();
  TcpStringData="E" +String(tempData);//+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  humidityData = ReadBME280HumidityData();//+".00";
  TcpStringData="U" +String(humidityData);//+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  ReadAllHumidityCompRegister();
  ReadDigT1TempCompRegister();
  ReadAllTempCompRegister();
  // use serial port console to see what is printed
  TcpStringData="T1" +String(dig_T1)+".00";
  client.println(TcpStringData);
  client.flush();
  
  delay(DELAYTIME);

  TcpStringData="T2" +String(dig_T2)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="T3" +String(dig_T3)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);
  // Send the Humidity Data and its compensation the the SerialPort
  // Compensation algorithm, Page 26:
  // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
  // unsigned char dig_H1,dig_H3;
  // signed short dig_H2,dig_H4,dig_H5;
  TcpStringData="H1" +String(dig_H1);".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H2" +String(dig_H2)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H3" +String((unsigned short)dig_H3);//+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H4" +String(dig_H4)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);


  TcpStringData="H5" +String(dig_H5)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H6"+String((float)dig_H6);
  client.println(TcpStringData);
  client.flush();
  delay(DELAYTIME);



}
