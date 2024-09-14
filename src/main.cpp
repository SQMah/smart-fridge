#define BLYNK_TEMPLATE_ID "TMPL21t4sKyIX"
#define BLYNK_TEMPLATE_NAME "Beetle fridge"
#define BLYNK_AUTH_TOKEN "NgOCm-GhGQeLESgIqKQ54MaOrfZ88xJL"
#define SHT31_ADDRESS 0x44

#include <Arduino.h>
#include <WiFi.h>
#include <SoftWire.h>
#include <SHT31_SW.h>
#include <BlynkSimpleEsp32.h>
#include <PID_v1.h>
#include <math.h> // Include math library for log functions

const char *ssid = "Not wifi";
const char *password = "Notapassword";

const int NUM_SENSORS = 2;
const int PELTIER_HZ = 50000; // 50 kHz for Peltiers
const int FAN_HZ = 20000;     // 20 kHz for fans
const int SENSOR_AND_PID_SAMPLE_TIME_MS = 1000;

// Function to calculate maximum resolution based on frequency
int getMaxResolution(int frequency)
{
  double clock = 80000000.0; // APB clock is 80 MHz
  double value = clock / frequency;
  int resolution = (int)floor(log(value) / log(2));
  return resolution;
}

// Calculate maximum resolutions based on frequencies
const int PELTIER_RESOLUTION = getMaxResolution(PELTIER_HZ);
const int FAN_RESOLUTION = getMaxResolution(FAN_HZ);

// Maximum duty values based on resolutions
const int MAX_PELTIER_DUTY = (1 << PELTIER_RESOLUTION) - 1;
const int MAX_FAN_DUTY = (1 << FAN_RESOLUTION) - 1;

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

// PWM-related definitions
const int topPeltierPwmPin = 26;       // Pin for top fridge PWM signal
const int bottomPeltierPwmPin = 27;    // Pin for bottom fridge PWM signal
const int topFanPin = 32;              // Pin for top fan PWM signal
const int bottomFanPin = 33;           // Pin for bottom fan PWM signal
const int topPeltierPwmChannel = 0;    // PWM channel for top fridge
const int bottomPeltierPwmChannel = 1; // PWM channel for bottom fridge
const int topFanChannel = 2;           // PWM channel for top fan
const int bottomFanChannel = 3;        // PWM channel for bottom fan

double topSetPoint = 18.0;                          // Default set point for top fridge (18°C)
double bottomSetPoint = 18.0;                       // Default set point for bottom fridge (18°C)
double topTemperature, bottomTemperature;           // Measured temperatures
double topPeltierOutputPwm, bottomPeltierOutputPwm; // PID outputs

// Threshold for clamping PWM to LOW or HIGH
const double peltierTwoSidedClamp = 5.0; // 5% threshold for clamping

// Fan clamp percentage (minimum speed)
double fanClamp = 20.0; // Default clamp at 20%

double Kp1 = 2.0, Ki1 = 5.0, Kd1 = 1.0;
PID topPID(&topTemperature, &topPeltierOutputPwm, &topSetPoint, Kp1, Ki1, Kd1, DIRECT);

// PID parameters for bottom fridge
double Kp2 = 2.0, Ki2 = 5.0, Kd2 = 1.0;
PID bottomPID(&bottomTemperature, &bottomPeltierOutputPwm, &bottomSetPoint, Kp2, Ki2, Kd2, DIRECT);

BLYNK_WRITE(V2)
{
  topSetPoint = param.asDouble();
}

BLYNK_WRITE(V3)
{
  bottomSetPoint = param.asDouble();
}

// Function to set up PWM with given frequency and resolution
void setupPWM(int channel, int pin, int frequency, int resolution)
{
  ledcSetup(channel, frequency, resolution); // Set PWM frequency with specified resolution
  ledcAttachPin(pin, channel);
}

// Function to update Peltier PWM output with clamping based on thresholds
void updatePeltierPWM(int channel, double duty)
{
  double dutyPercent = (duty / MAX_PELTIER_DUTY) * 100.0; // Convert duty cycle to percentage

  if (dutyPercent < peltierTwoSidedClamp)
  {
    // If duty cycle is less than threshold, set output LOW
    ledcWrite(channel, 0);
  }
  else if (dutyPercent > (100.0 - peltierTwoSidedClamp))
  {
    // If duty cycle is greater than (100 - threshold), set output HIGH
    ledcWrite(channel, MAX_PELTIER_DUTY); // Max for Peltier resolution
  }
  else
  {
    // Otherwise, use the computed PWM duty cycle
    ledcWrite(channel, (int)duty);
  }
}

// Function to update fan PWM output with fan clamp logic
void updateFanPWM(int channel, double duty)
{
  // Map duty from Peltier resolution to Fan resolution
  double dutyFan = (duty / MAX_PELTIER_DUTY) * MAX_FAN_DUTY;

  double dutyPercent = (dutyFan / MAX_FAN_DUTY) * 100.0; // Convert duty cycle to percentage

  if (duty == 0)
  {
    // If PID output is 0, turn off the fan
    ledcWrite(channel, 0);
  }
  else if (dutyPercent < fanClamp)
  {
    // If duty cycle is below the fan clamp, set it to the fan clamp value
    int fanPWM = (fanClamp / 100.0) * MAX_FAN_DUTY;
    ledcWrite(channel, fanPWM);
  }
  else
  {
    // Otherwise, set the fan to the proportional PID output scaled to fan resolution
    ledcWrite(channel, (int)dutyFan);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(10);

  // Initialize software I2C buses and sensors
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    swWires[i].begin();
    swWires[i].setClock(100000);
  }
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensors[i].begin();
    sensors[i].heatOff();
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Initialize the PWM channels with calculated resolutions
  setupPWM(topPeltierPwmChannel, topPeltierPwmPin, PELTIER_HZ, PELTIER_RESOLUTION);
  setupPWM(bottomPeltierPwmChannel, bottomPeltierPwmPin, PELTIER_HZ, PELTIER_RESOLUTION);
  setupPWM(topFanChannel, topFanPin, FAN_HZ, FAN_RESOLUTION);
  setupPWM(bottomFanChannel, bottomFanPin, FAN_HZ, FAN_RESOLUTION);

  // Initialize PID controllers
  topPID.SetOutputLimits(0, MAX_PELTIER_DUTY); // Set output limits for Peltier PID
  bottomPID.SetOutputLimits(0, MAX_PELTIER_DUTY);

  // Set PID sample times to match control loop timing (e.g., 1 second)
  topPID.SetSampleTime(SENSOR_AND_PID_SAMPLE_TIME_MS);
  bottomPID.SetSampleTime(SENSOR_AND_PID_SAMPLE_TIME_MS);

  topPID.SetMode(AUTOMATIC);
  bottomPID.SetMode(AUTOMATIC);
}

void readSensorsAndUpdateControl()
{
  // Read temperature and humidity sensors
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensors[i].requestData();
    delay(50); // Wait for measurement to complete
  }
  topTemperature = sensors[0].getTemperature();
  bottomTemperature = sensors[1].getTemperature();
  float topHumidity = sensors[0].getHumidity();
  float bottomHumidity = sensors[1].getHumidity();

  // Send sensor data to Blynk
  Blynk.virtualWrite(temperaturePins[0], topTemperature);
  Blynk.virtualWrite(temperaturePins[1], bottomTemperature);
  Blynk.virtualWrite(V4, bottomHumidity);
  Blynk.virtualWrite(V5, topHumidity); // Assuming V5 is the virtual pin for top humidity

  // Update the PID controllers
  topPID.Compute();
  bottomPID.Compute();

  // Update fridge PWM outputs with clamping logic
  updatePeltierPWM(topPeltierPwmChannel, topPeltierOutputPwm);
  updatePeltierPWM(bottomPeltierPwmChannel, bottomPeltierOutputPwm);

  // Update fan PWM outputs with fan clamp logic
  updateFanPWM(topFanChannel, topPeltierOutputPwm);
  updateFanPWM(bottomFanChannel, bottomPeltierOutputPwm);

  // Convert PWM duty cycles to percentage
  double topPeltierDutyPercent = (topPeltierOutputPwm / MAX_PELTIER_DUTY) * 100.0;
  double bottomPeltierDutyPercent = (bottomPeltierOutputPwm / MAX_PELTIER_DUTY) * 100.0;

  double topFanDutyPercent = (ledcRead(topFanChannel) / (double)MAX_FAN_DUTY) * 100.0;
  double bottomFanDutyPercent = (ledcRead(bottomFanChannel) / (double)MAX_FAN_DUTY) * 100.0;

  // Print temperature and humidity readings
  Serial.println("===============================================");
  Serial.println("                 Fridge Status                 ");
  Serial.println("===============================================");
  Serial.println("Top Fridge: ");
  Serial.print("  Temperature:      ");
  Serial.print(topTemperature, 2);
  Serial.println(" °C");
  Serial.print("  Setpoint:         ");
  Serial.print(topSetPoint, 2);
  Serial.println(" °C");
  Serial.print("  Humidity:         ");
  Serial.print(topHumidity, 2);
  Serial.println(" %");
  Serial.print("  Peltier Duty:     ");
  Serial.print(topPeltierDutyPercent, 2);
  Serial.println(" %");
  Serial.print("  Fan Duty:         ");
  Serial.print(topFanDutyPercent, 2);
  Serial.println(" %");
  Serial.println("===============================================");

  Serial.println("Bottom Fridge: ");
  Serial.print("  Temperature:      ");
  Serial.print(bottomTemperature, 2);
  Serial.println(" °C");
  Serial.print("  Setpoint:         ");
  Serial.print(bottomSetPoint, 2);
  Serial.println(" °C");
  Serial.print("  Humidity:         ");
  Serial.print(bottomHumidity, 2);
  Serial.println(" %");
  Serial.print("  Peltier Duty:     ");
  Serial.print(bottomPeltierDutyPercent, 2);
  Serial.println(" %");
  Serial.print("  Fan Duty:         ");
  Serial.print(bottomFanDutyPercent, 2);
  Serial.println(" %");
  Serial.println("===============================================");
}

void loop()
{
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  Blynk.run(); // Run Blynk as frequently as possible

  if (currentTime - lastUpdateTime >= SENSOR_AND_PID_SAMPLE_TIME_MS)
  {
    lastUpdateTime = currentTime;

    // Read sensors and update control
    readSensorsAndUpdateControl();
  }
}