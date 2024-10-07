#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <ArduinoBLE.h>

// Pin definitions
#define XSHUT_PIN D7
#define LED_PIN LED_BUILTIN
#define RGB_RED_PIN A1
#define RGB_GREEN_PIN A2
#define RGB_BLUE_PIN A3

// Battery management pin definitions
#define GPIO_BATTERY_CHARGE_SPEED 13
#define GPIO_BATTERY_CHARGING_ENABLE 17
#define GPIO_BATTERY_READ_ENABLE 14
#define VBAT_PIN PIN_VBAT

#define NUM_SAMPLES 10

// Battery voltage calculation constants
#define VBAT_DIVIDER_R1   1000000
#define VBAT_DIVIDER_R2   510000
#define VBAT_DIVIDER      ((float)VBAT_DIVIDER_R2 / (VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2))
#define ADC_RESOLUTION    (1 << 12)
#define VREF_MV           3600
#define VBAT_CALIBRATION  0.3806F

// LiPo battery constants
#define LIPO_MIN_V 3.4
#define LIPO_MAX_V 4.2

VL53L4CD sensor_vl53l4cd_sat(&Wire, XSHUT_PIN);

// BLE service and characteristic UUIDs
BLEService epiPenService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic voltageCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEUnsignedShortCharacteristic distanceCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEBoolCharacteristic chargingStatusCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLECharCharacteristic commandCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic rgbCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 3, true);

void setup() {
  // Initialize BLE
  if (!BLE.begin()) {
    while (1);
  }

  // Set up the BLE device
  BLE.setLocalName("EpiPen");
  BLE.setAdvertisedService(epiPenService);

  // Add characteristics to the service
  epiPenService.addCharacteristic(voltageCharacteristic);
  epiPenService.addCharacteristic(distanceCharacteristic);
  epiPenService.addCharacteristic(chargingStatusCharacteristic);
  epiPenService.addCharacteristic(commandCharacteristic);
  epiPenService.addCharacteristic(rgbCharacteristic);

  // Add the service
  BLE.addService(epiPenService);

  // Set initial values for characteristics
  voltageCharacteristic.writeValue(0.0);
  distanceCharacteristic.writeValue(0);
  chargingStatusCharacteristic.writeValue(false);

  // Start advertising
  BLE.advertise();

  // Set up battery management pins
  pinMode(GPIO_BATTERY_CHARGE_SPEED, OUTPUT);
  pinMode(GPIO_BATTERY_CHARGING_ENABLE, INPUT);
  pinMode(GPIO_BATTERY_READ_ENABLE, OUTPUT);

  // Enable fast charging and battery reading
  setFastCharging(true);
  enableBatteryReading(true);

  Wire.begin();
  sensor_vl53l4cd_sat.begin();
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  sensor_vl53l4cd_sat.InitSensor();
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

  pinMode(LED_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  setRGBColor(0, 0, 0);

  analogReference(AR_INTERNAL);
  analogReadResolution(12);

  checkChargingStatus();
  readVoltage();
}

float readVBAT(void) {
  uint32_t total_raw = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total_raw += analogRead(VBAT_PIN);
    delay(10);
  }
  int adcReading = total_raw / (float)NUM_SAMPLES;
  float adcVoltage = (adcReading * VREF_MV) / ADC_RESOLUTION;
  float batteryVoltage = (adcVoltage / VBAT_DIVIDER) * VBAT_CALIBRATION;
  return batteryVoltage;
}

void setFastCharging(bool enable) {
  digitalWrite(GPIO_BATTERY_CHARGE_SPEED, enable ? HIGH : LOW);
}

void enableBatteryReading(bool enable) {
  digitalWrite(GPIO_BATTERY_READ_ENABLE, enable ? HIGH : LOW);
}

bool isCharging() {
  return digitalRead(GPIO_BATTERY_CHARGING_ENABLE) == HIGH;
}

void checkChargingStatus() {
  bool charging = isCharging();
  chargingStatusCharacteristic.writeValue(charging);
}

void readVoltage() {
  float vbat_v = readVBAT() / 1000.0;
  float vbat_per = lipoToPercent(vbat_v);
  voltageCharacteristic.writeValue(vbat_v);
}

float lipoToPercent(float voltage) {
  float percentage = (voltage - LIPO_MIN_V) / (LIPO_MAX_V - LIPO_MIN_V) * 100.0;
  return constrain(percentage, 0, 100);
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

void readDistance() {
  VL53L4CD_Result_t results;
  uint8_t status = sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
  
  if (!status) {
    distanceCharacteristic.writeValue(results.distance_mm);
  }
}

void handleCommand(char command) {
  switch(command) {
    case '1':
      readVoltage();
      break;
    case '2':
      readDistance();
      break;
    case '3':
      checkChargingStatus();
      break;
    case '4':
      setFastCharging(true);
      break;
    case '5':
      setFastCharging(false);
      break;
  }
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    while (central.connected()) {
      if (commandCharacteristic.written()) {
        char command = commandCharacteristic.value();
        handleCommand(command);
      }
      
      if (rgbCharacteristic.written()) {
        uint8_t rgbValues[3];
        rgbCharacteristic.readValue(rgbValues, 3);
        setRGBColor(rgbValues[0], rgbValues[1], rgbValues[2]);
      }
    }
  }
  
  sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();
}