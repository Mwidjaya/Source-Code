#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

// Pin definitions and constants
#define DO_PIN A1
#define VREF 5000     // VREF (mv)
#define ADC_RES 1024  // ADC Resolution

// Single-point calibration settings for DO
#define TWO_POINT_CALIBRATION 0
#define CAL1_V 1323  // mv
#define CAL1_T 25    // ℃

// pH sensor pin definitions
#define RX_PIN 2
#define TX_PIN 3

// Turbidity sensor settings
#define TURBID_PIN A2

// Depth sensor settings
#define ANALOG_PIN A0
#define RANGE 5000         // Depth measuring range 5000mm (for water)
#define CURRENT_INIT 4.00  // Current @ 0mm (unit: mA)
#define DENSITY_WATER 1    // Pure water density normalized to 1

// DO Table
const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// Global variables
float suhuAfterCal, phcal;  // unit: mA
int16_t dataVoltage;
float dataCurrent, depth, caldepth;
int dataADC;
unsigned long timepoint_measure;
String inputstring = "";
String sensorstring = "";
boolean input_string_complete = false;
boolean sensor_string_complete = false;
float pH;
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

// Initialize OneWire and DallasTemperature objects
const int pinOutput = 4;
OneWire oneWire(pinOutput);
DallasTemperature sensors(&oneWire);

// Initialize software serial for pH sensor
SoftwareSerial myserial(RX_PIN, TX_PIN);

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void sendDepth();

void setup() {
  Serial.begin(9600);
  // myserial.begin(9600);
  Serial3.begin(9600);  // set baud rate for software serial port_3 to 9600
  // inputstring.reserve(10);  // set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);  // set aside some bytes for receiving data from Atlas Scientific product

  // Initialize DS18B20 sensor
  sensors.begin();
  // sensorstring.reserve(30);

  // Set pin modes
  pinMode(pinOutput, OUTPUT);
  timepoint_measure = millis();

  //Serial.println("System Initializing...");
}

float readTemperature() {
  sensors.requestTemperatures();
  float temperatureCelsius = sensors.getTempCByIndex(0);
  suhuAfterCal = (1.0196 * temperatureCelsius) - 0.7646;
  return suhuAfterCal;
}

float readDO(float temperature) {
  Temperaturet = (uint8_t)temperature;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  float douncal = float(readDO(ADC_Voltage, Temperaturet));
  double docal = (0.8881 * douncal) + 160.16;
  return docal / 1000;
}

float readpH() {
  while (sensor_string_complete != true) {
    if (Serial3.available() > 0)  // Jika ada data dari produk Atlas Scientific
    {
      char inchar = (char)Serial3.read();  // Membaca karakter yang diterima
      sensorstring += inchar;              // Menambahkan karakter ke string sensor
      if (inchar == '\r')                  // Jika karakter yang diterima adalah carriage return (CR)
      {
        sensor_string_complete = true;  // Mengatur flag bahwa string lengkap telah diterima
      }
    }
  }
  float sensorfloat = sensorstring.toFloat();
  float phcal = (0.9986 * sensorfloat) - 0.0826;
  sensorstring = "";
  sensor_string_complete = false;
  return phcal;
}

float readDepth() {
  dataADC = analogRead(ANALOG_PIN);
  dataVoltage = dataADC / 1024.0 * VREF;
  dataCurrent = dataVoltage / 120.0;  // Sense Resistor: 120ohm
  depth = (dataCurrent - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0);
  if (depth < 0) {
    depth = 0.0;
  }
  caldepth = 0.9984 * depth + 114.37;
  caldepth = caldepth / 10;
  return caldepth;
}

float readTurbidity() {
  int sensorValue = analogRead(TURBID_PIN);      // read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0);  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float 
  // Serial.println(voltage); // print out the value you read:
  return voltage;
}

void sendWaterQuality() {
  const int numSamples = 10;
  // float suhuSamples[numSamples];
  // float phSamples[numSamples];
  // float doSamples[numSamples];
  // float turbiditySamples[numSamples];
  float sumSuhu = 0, sumPH = 0, sumDo = 0, sumTurbidity = 0;

  for (int i = 0; i < numSamples; i++) {
    float suhu = readTemperature();
    float phValue = readpH();
    float doxy = readDO(suhu);
    float turbidValue = readTurbidity();

    sumSuhu += suhu;
    sumPH += phValue;
    sumDo += doxy;
    sumTurbidity += turbidValue;

    delay(100);
  }

  float avgSuhu = sumSuhu / numSamples;
  float avgPh = sumPH / numSamples;
  float avgDo = sumDo / numSamples;
  float avgTurbidity = sumTurbidity / numSamples;

  Serial.print(avgSuhu);
  Serial.print(",");
  Serial.print(avgPh);
  Serial.print(",");
  Serial.print(avgDo);
  Serial.print(",");
  Serial.println(avgTurbidity);
}

String getDataComm() {
  String data = "";
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    data.trim();  // Remove any leading/trailing whitespace
  }
  return data;
}

void stateful() {
  String data = getDataComm();
  //Serial.print("Received command: ");
  //Serial.println(data); // Debug print to check received command
  if (data.length() > 0) {
    data.trim();  // Remove any leading/trailing whitespace or newlines
  }
  if (data.equals("dalam")) {
    while (true) {
      sendDepth();
      data = getDataComm();
      //Serial.print("Received command in loop: ");
      //Serial.println(data); // Debug print inside loop
      if (data.equals("air")) {
        //Serial.println("Reading sensor....");
        sendWaterQuality();
        break;
      }
    }
  } else if (data.equals("air")) {
    Serial.println("Reading sensor....");
    sendWaterQuality();
    // data = getDataComm();
  }
}

void sendDepth() {
  float depths = readDepth();
  Serial.println(depths);
  delay(500);
}

void loop() {
  stateful();
}