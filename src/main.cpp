#define BLYNK_TEMPLATE_ID "TMPL21t4sKyIX"
#define BLYNK_TEMPLATE_NAME "Beetle fridge"
#define BLYNK_AUTH_TOKEN "NgOCm-GhGQeLESgIqKQ54MaOrfZ88xJL"
#define SHT31_ADDRESS 0x44

#include <Arduino.h>
#include <WiFi.h>
#include <SoftWire.h>
#include <SHT31_SW.h>
#include <BlynkSimpleEsp32.h>

// Replace with your network credentials
const char *ssid = "Not wifi";
const char *password = "Notapassword";

// Define the number of sensors
const int NUM_SENSORS = 2;

// Create software I2C instances using SoftWire
SoftWire swWires[NUM_SENSORS] = {
    SoftWire(23, 22), // SDA, SCL for the first bus
    SoftWire(19, 18)  // SDA, SCL for the second bus
};

// Create SHT31 sensor instances
SHT31_SW sensors[NUM_SENSORS] = {
    SHT31_SW(SHT31_ADDRESS, &swWires[0]),
    SHT31_SW(SHT31_ADDRESS, &swWires[1])};

// Blynk virtual pin arrays
const int temperaturePins[NUM_SENSORS] = {V1, V0};
const int humidityPins[NUM_SENSORS] = {V3, V2};
const int heaterControlPin = V4; // Virtual pin for heater control button

bool heaterState = false; // Variable to store the heater state

// Function prototypes
void readSHT30Sensors();
void printSensorReadings(float humidity, float temperature, int sensorNumber, bool heaterState);
bool readSHT30(SHT31_SW &sht, float &temperature, float &humidity, int sensorNumber, bool heaterState);

void setup()
{
  Serial.begin(115200);
  delay(10);

  // Initialize software I2C buses and sensors
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    swWires[i].begin();
    swWires[i].setClock(100000);
    sensors[i].begin();
    sensors[i].heatOff();
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
}

void loop()
{
  Blynk.run(); // Run Blynk
  readSHT30Sensors();
  delay(30000);
}

void readSHT30Sensors()
{
  float temperature, humidity;
  bool heaterState;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (readSHT30(sensors[i], temperature, humidity, i + 1, heaterState))
    {
      Blynk.virtualWrite(temperaturePins[i], temperature);
      Blynk.virtualWrite(humidityPins[i], humidity);
      // printSensorReadings(humidity, temperature, i + 1, heaterState);
    }
  }
}

// Function to print sensor readings
void printSensorReadings(float humidity, float temperature, int sensorNumber, bool heaterState)
{
  Serial.print("Sensor ");
  Serial.print(sensorNumber);
  Serial.print(" - Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  Serial.print("Heater state: ");
  Serial.println(heaterState ? "ON" : "OFF");
}

// Function to read SHT30 sensor using SoftWire
bool readSHT30(SHT31_SW &sht, float &temperature, float &humidity, int sensorNumber, bool heaterState)
{
  sht.read();
  temperature = sht.getTemperature();
  humidity = sht.getHumidity();
  heaterState = sht.isHeaterOn();
  return true;
}

// Blynk function to handle heater control button press
BLYNK_WRITE(heaterControlPin)
{
  heaterState = param.asInt(); // Get button state
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (heaterState)
    {
      sensors[i].heatOn(); // Turn on the heater
    }
    else
    {
      sensors[i].heatOff(); // Turn off the heater
    }
  }
}
