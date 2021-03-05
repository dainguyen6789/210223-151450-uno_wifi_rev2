# 1 "C:\\Users\\UTILIS~1\\AppData\\Local\\Temp\\tmpvceftug6"
#include <Arduino.h>
# 1 "C:/Users/Utilisateur/Documents/PlatformIO/Projects/210223-151450-uno_wifi_rev2/src/TcpClientArduinoUniWifiRev2.ino"
#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino.h>
#include <Wire.h>
#include <BME280.h>
#define DELAYTIME 500


#define WIFI_OFFICE 


#ifdef WIFI_HOME
char ssid[] = "TANASE";
char pass[] = "3629678427";
IPAddress server(192, 168, 2,206);
#endif


#ifdef WIFI_OFFICE
char ssid[] = "Office" ;
char pass[] = "erasure hunter mangle hydrated" ;
IPAddress server(192, 168, 11,166);
#endif
int status = WL_IDLE_STATUS;


WiFiClient client;

int analogPin[6];
float voltageAtAdcPin;
int i;
int32_t humidityData;
int32_t tempData;






unsigned short dig_T1;
signed short dig_T2, dig_T3;


unsigned char dig_H1, dig_H3;
signed short dig_H2, dig_H4, dig_H5;
char dig_H6;
void SetupTCP();
void ConfigAnalogPins();
void InitBme280I2c();
int32_t ReadBME280HumidityData();
int32_t ReadBME280TempData();
int16_t ReadCompRegister(byte registerAddress);
unsigned char ReadRegister(unsigned char registerAddress);
char ReadDigH6Register();
void ReadAllHumidityCompRegister();
void ReadDigT1TempCompRegister();
void ReadAllTempCompRegister();
void setup();
voi SendCompensationT1();
void SendCompensationData();
void loop();
#line 48 "C:/Users/Utilisateur/Documents/PlatformIO/Projects/210223-151450-uno_wifi_rev2/src/TcpClientArduinoUniWifiRev2.ino"
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

    while (true)
      ;
  }
  else
  {
    Serial.println("Connected to wifi");
    Serial.println("\nStarting connection...");

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




  Wire.begin();
  Wire.beginTransmission(BME280Address);
  Wire.write(BME280_CTRL_MEAS);
  Wire.write(BME280_NORMAL_MODE | BME280_TEMP_OVERAMPLING_1);
  Wire.endTransmission();

  Wire.beginTransmission(BME280Address);
  Wire.write(BME280_CTRL_HUM);
  Wire.write(BME280_HU_OVERAMPLING_1);
  Wire.endTransmission();
}



int32_t ReadBME280HumidityData()
{
  int32_t reading = 0;
  Wire.beginTransmission(BME280Address);
  Wire.write(BME280_HUMIDITY_REG);
  Wire.endTransmission();

  Wire.beginTransmission(BME280Address);
  Wire.requestFrom(BME280Address, 2);


  if (2 <= Wire.available())
  {

    reading = Wire.read();

    reading = reading << 8;

    reading |= Wire.read();


  }
  Wire.endTransmission();
  return reading;
}

int32_t ReadBME280TempData()
{
  int32_t reading = 0;
  Wire.beginTransmission(BME280Address);
  Wire.write(BME280_TEMP_REG);
  Wire.endTransmission();

  Wire.beginTransmission(BME280Address);
  Wire.requestFrom(BME280Address, 3);


  if (3 <= Wire.available())
  {

    reading = Wire.read();

    reading = reading << 12;

    reading |= Wire.read() << 4;

    reading |= Wire.read() & (0x0F);


  }
  Wire.endTransmission();
  return reading;
}

int16_t ReadCompRegister(byte registerAddress)
{
  int16_t data = 0;
  Wire.beginTransmission(BME280Address);
  Wire.write(byte(registerAddress));
  Wire.endTransmission();


  Wire.beginTransmission(BME280Address);
  Wire.requestFrom(BME280Address, 1);

  data = Wire.read();
  Wire.endTransmission();

  return data;
}
unsigned char ReadRegister(unsigned char registerAddress)
{
  unsigned char data = 0;
  Wire.beginTransmission(BME280Address);
  Wire.write(byte(registerAddress));
  Wire.endTransmission();


  Wire.beginTransmission(BME280Address);
  Wire.requestFrom(BME280Address, 1);

  data = Wire.read();
  Wire.endTransmission();
  return data;
}

char ReadDigH6Register()
{
  char data = 0;
  Wire.beginTransmission(BME280Address);
  Wire.write(DIG_H6_REGISTER_ADDRESS);
  Wire.endTransmission();


  Wire.beginTransmission(BME280Address);
  Wire.requestFrom(BME280Address, 1);

  data = Wire.read();
  Wire.endTransmission();
  return data;
}

void ReadAllHumidityCompRegister()
{
  int16_t dataHi, dataLo;
  dig_H1 = ReadRegister(DIG_H1_REGISTER_ADDRESS);

  dataLo = ReadCompRegister(DIG_H2_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_H2_HI_REGISTER_ADDRESS);

  dig_H2 = (dataHi) << 8 | (dataLo);

  dig_H3 = ReadRegister(DIG_H3_REGISTER_ADDRESS);

  dataHi = ReadCompRegister(DIG_H4_HI_REGISTER_ADDRESS);
  dataLo = ReadCompRegister(DIG_H4_H5_REGISTER_ADDRESS);
  dig_H4 = (dataHi << 4) | (dataLo & 0x000F);

  dataHi = ReadCompRegister(DIG_H5_HI_REGISTER_ADDRESS);
  dig_H5 = (dataHi << 4) | ((dataLo & 0x00F0) >> 4);

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





  dataLo = ReadCompRegister(DIG_T2_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_T2_HI_REGISTER_ADDRESS);
  dig_T2 = (dataHi << 8) | dataLo;

  dataLo = ReadCompRegister(DIG_T3_LO_REGISTER_ADDRESS);
  dataHi = ReadCompRegister(DIG_T3_HI_REGISTER_ADDRESS);
  dig_T3 = (dataHi << 8) | dataLo;
}

void setup()
{

  ConfigAnalogPins();
  InitBme280I2c();
  SetupTCP();
}
voi SendCompensationT1()
{
  TcpStringData="T1" +String(dig_T1)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);
}
void SendCompensationData()
{
  ReadAllHumidityCompRegister();
  ReadDigT1TempCompRegister();
  ReadAllTempCompRegister();

  SendCompensationT1();

  delay(DELAYTIME);

  TcpStringData="T2" +String(dig_T2)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="T3" +String(dig_T3)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);





  TcpStringData="H1" +String(dig_H1);".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H2" +String(dig_H2)+".00";
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  TcpStringData="H3" +String((unsigned short)dig_H3);
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

void loop()
{
  String TcpStringData;

  for (i = 0; i < 6; i++)
  {
    voltageAtAdcPin = (float)analogRead(analogPin[i]) / 1024 * 5;
    TcpStringData=String(i + 1)+String(voltageAtAdcPin,4);
    client.println(TcpStringData);
    client.flush();

    delay(DELAYTIME);
  }

  tempData = ReadBME280TempData();
  TcpStringData="E" +String(tempData);
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);

  humidityData = ReadBME280HumidityData();
  TcpStringData="U" +String(humidityData);
  client.println(TcpStringData);
  client.flush();

  delay(DELAYTIME);





}