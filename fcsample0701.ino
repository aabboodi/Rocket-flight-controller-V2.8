
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPIMemory.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <Bounce.h>
#include <Kalman.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS false         // Set to false for basic data read
#define SerialDebug true   // Set to true to get Serial output for debugging

#define I2Cclock 400000
#define Wire2 Wire2
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
#define MPU9250_ADDRESS 0x68

MPU9250 myIMU(MPU9250_ADDRESS, Wire2, I2Cclock);


#define GPSSerial Serial3

#if defined(SIMBLEE)
#define BAUD_RATE 250000
#define RANDPIN 1
#else
#define BAUD_RATE 115200
#if defined(ARCH_STM32)
#define RANDPIN PA0
#else
#define RANDPIN A0
#endif
#endif

SPIFlash flash;
const uint32_t logStartAddr = 0x00000;  // Starting address for the log in flash memory
const uint32_t logSize = 4096;         // Size of the log (adjust as needed)
uint32_t logEndAddr = 0;

Adafruit_BMP280 bmp(&Wire2);
Servo servoX;
Servo servoY;
const int servoXPin = 8;
const int servoYPin = 9;

bool testState = false;
bool prelaunchState = false;
bool launchState = false;
bool flightState = false;
bool descentState = false;
bool parachuteDeployed = false;

bool runTest();
bool deployParachute();
#define  measuringbatteryvoltage 16
Bounce teststatePIN = Bounce(33, 50);
Bounce prelaunchstatePIN = Bounce(34, 50);
Bounce launchstatePIN = Bounce(35, 50);
Bounce flashclearPin = Bounce(41, 50);
int redPin = 38;
int greenPin = 39;
int bluePin = 40;
int buzzer = 7;
void ringBuzzer();
enum BuzzerState {
  BUZZER_OFF,
  BUZZER_ON,
  BUZZER_DONE
};
BuzzerState buzzerState = BUZZER_OFF;
unsigned long lastBuzzerChangeTime = 0;
const unsigned long buzzerOnDuration = 300;


bool buttonPressed = false;
float previousFilteredGyro = 0.0;
float previousFilteredAccel = 0.0;
float PARACHUTE_DEPLOYMENT_ALTITUDE_MIN;
float maxAltitude = 0;
float initialPressure;
float currentAltitude = 0.0;
float velocity = 0.0;

float integralPitchError = 0;
float integralRollError = 0;
float integralYawError = 0;
float previousPitchError = 0;
float previousRollError = 0;
float previousYawError = 0;
const float MIN_ALTITUDE = 0.0;
const float MAX_ALTITUDE = 1000.0;
const float MIN_BATTERY_VOLTAGE = 11.0;
const float MAX_VOLTAGE = 13.0;
const float PARACHUTE_DEPLOYMENT_ALTITUDE = 5.0;
const float MAX_NOISE_VOLTAGE = 1.0;
const float DESCENT_VELOCITY_THRESHOLD = 1.0;


const int pyroPins[] = {19, 18, 37, 36};
const int numPins = sizeof(pyroPins) / sizeof(pyroPins[0]);
float voltageThreshold = 4.3;  // Set the voltage threshold of pyro
void setupServos();
int servoXAngle;
int servoYAngle;


float desiredRoll = 0.0f; // Set desired roll angle to 0 degrees
float desiredPitch = 0.0f; // Set desired pitch angle to 10 degrees
float desiredYaw = -5.33f; // Set desired yaw angle to -5 degrees

float gyroData[3];
bool isWelcomeMessagePrinted = false;
// Define buffer size and create a circular buffer
const int BUFFER_SIZE = 100;
struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float temperature;
  float yaw, pitch, roll;
};
IMUData buffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;

void calculatePitchError();
void calculateRollError();
void calculateYawError();


/***********************************************************************************************/
void setup() {
  while (!Serial) {}
  Serial.begin(115200);
  initializeMPU9250();
  setupBMP();
  setupLED();
  setupBuzzer();
  setupPyroChannels();
  setupFlashMemory();
  pinMode(33, INPUT_PULLUP); // Test state pin
  pinMode(34, INPUT_PULLUP); // Prelaunch pin
  pinMode(35, INPUT_PULLUP); // Launch pin
  setupServos();
   // Initialize initial pressure
  initialPressure = bmp.readPressure();// Measure at ground level

  // Initial parachute deployment altitude limits
  PARACHUTE_DEPLOYMENT_ALTITUDE_MIN = initialPressure - 25.0;// Adjust the value accordingly


  pinMode(buzzer, OUTPUT);


}

bool initializeMPU9250() {
    unsigned long startTime = millis(); // Store the current time
    const unsigned long timeout = 100; // Set a timeout of 500 milliseconds (adjust as needed)
    myIMU.initMPU9250();
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    Wire2.begin();

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    // Verify the WHO_AM_I value
    if (c == 0x71) {
        Serial.println(F("MPU9250 is online..."));
        return true;
    } else {
        Serial.print(F("Could not connect to MPU9250: 0x"));
        Serial.println(c, HEX);

        // Read the WHO_AM_I register again after reset
        c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

        // Verify the WHO_AM_I value after reset
        if (c == 0x71) {
            Serial.println(F("MPU9250 is online after reset..."));
            return true;
        } else {
            Serial.print(F("Could not connect to MPU9250 after reset: 0x"));
            Serial.println(c, HEX);

            // Check if the timeout has been reached
            if (millis() - startTime >= timeout) {
                Serial.println(F("Timeout reached while initializing MPU9250"));
                return false; // Return false if timeout is reached
            }
        }
    }

    return false; // Return false if initialization failed
}

void setupBMP() {
  unsigned status;
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // 5 seconds timeout
  unsigned int retryCount = 0;
  const unsigned int maxRetries = 10; // Maximum number of retries

  do {
    status = bmp.begin();
    if (status) break; // Initialization successful, exit the loop

    Serial.println(F("Could not find a valid BMP280 sensor, retrying..."));
    delay(500); // Wait for a short period before retrying
    retryCount++;
  } while (retryCount < maxRetries && (millis() - startTime < timeout));

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    // Handle the failure case here, e.g., return or set a flag
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}
void setupLED() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void setupBuzzer() {
  pinMode(buzzer, OUTPUT);
}

void setupPyroChannels() {

  for (int i = 0; i < 4; i++) {
    pinMode(18 + i, OUTPUT);  // Pyro channels
    digitalWrite(18 + i, LOW);
  }
}

void setupFlashMemory() {
  Serial.print(F("Initializing"));

  for (uint8_t i = 0; i < 10; ++i) {
    Serial.print(F("."));
  }

  Serial.println();
  randomSeed(analogRead(RANDPIN));

  // Setup the button pin
  pinMode(41, INPUT_PULLUP);
}


/*******************************************************************************************************************************************************************************************************/

void loop() {

readIMUData();
adjustFins();


  // Clear flash memory if the button is pressed
  bool currentButtonState = digitalRead(41) == LOW;


  // Check if the button state has changed
  if (currentButtonState != buttonPressed) {
    delay(50); // Debounce delay
    currentButtonState = digitalRead(41) == LOW;
    if (currentButtonState) {
      Serial.println("Clearing flash memory...");
      clearFlashMemory();
      delay(1000); // Add a delay to debounce the button
    }
    buttonPressed = currentButtonState;
 }
  
 
    int16_t gyroData[3];
    myIMU.readGyroData(gyroData);
    // Access gyro data
   
  teststatePIN.update();
  prelaunchstatePIN.update();
  launchstatePIN.update();

  bool testButtonPressed = teststatePIN.risingEdge();
  bool prelaunchButtonPressed = prelaunchstatePIN.risingEdge();
  bool launchButtonPressed = launchstatePIN.risingEdge();

  // Handle button press combinations
  if (testButtonPressed && !prelaunchButtonPressed && !launchButtonPressed) {
    // Test state
    testState = runTest();
    resetOtherStates();

    handleButtonCombinations(testButtonPressed, prelaunchButtonPressed, launchButtonPressed);
  } else if (!testButtonPressed && !prelaunchButtonPressed && !launchButtonPressed) {
    if(!isWelcomeMessagePrinted) {
    Serial.println("Hello, my name is H&FC01. I'm ready to be tested, sir.");
    isWelcomeMessagePrinted = true;
    }
  } else if (!testButtonPressed && prelaunchButtonPressed && !launchButtonPressed) {
    // Prelaunch state
    prelaunchState = prelaunchStateSetup();
    resetOtherStates();

    if (prelaunchState) {
      Serial.println("Ready to launch");
    } else {
      Serial.println("Pre-launch failed");
    }
  } else if (!testButtonPressed && !prelaunchButtonPressed && launchButtonPressed) {
    // Launch state
    launchState = launchstateSetup();
    resetOtherStates();

    if (launchState) {
      Serial.println("Transitioning to flight state.");
      flightStateSetup();
      digitalWrite(18, HIGH); // Assuming Pyro channel two is connected to pin 18
    } else {
      Serial.println("Launch failed");
    }
  } else {
    handleButtonCombinations(testButtonPressed, prelaunchButtonPressed, launchButtonPressed);
    resetWelcomeMessageFlag();
  }

  // Handle state transitions
  if (launchState) {
    flightState = flightStateSetup();
    resetOtherStates();
  }

  if (isApogeeDetected() && flightStateSetup()) {
    descentState = descentstateSetup();
    resetOtherStates();
  }

  delay(100); // Delay for stability
}

void handleButtonCombinations(bool testButtonPressed, bool prelaunchButtonPressed, bool launchButtonPressed) {
  resetStates();

  if (testButtonPressed && prelaunchButtonPressed && launchButtonPressed) {
    Serial.println("Warning: All buttons pressed simultaneously.");
  } else if (testButtonPressed && prelaunchButtonPressed) {
    Serial.println("Warning: Test state and prelaunch state buttons pressed simultaneously.");
  } else if (testButtonPressed && launchButtonPressed) {
    Serial.println("Warning: Test state and launch state buttons pressed simultaneously.");
  } else if (prelaunchButtonPressed && launchButtonPressed) {
    Serial.println("Warning: Prelaunch state and launch state buttons pressed simultaneously.");
  }
}

void resetStates() {
  testState = false;
  prelaunchState = false;
  launchState = false;
  flightState = false;
  descentState = false;
}

void resetOtherStates() {
  if (testState) {
    prelaunchState = false;
    launchState = false;
    flightState = false;
    descentState = false;
  } else if (prelaunchState) {
    testState = false;
    launchState = false;
    flightState = false;
    descentState = false;
  } else if (launchState) {
    testState = false;
    prelaunchState = false;
    flightState = false;
    descentState = false;
  } else if (flightState) {
    testState = false;
    prelaunchState = false;
    launchState = false;
    descentState = false;
  } else if (descentState) {
    testState = false;
    prelaunchState = false;
    launchState = false;
    flightState = false;
  }
}
void resetWelcomeMessageFlag() {
  isWelcomeMessagePrinted = false;
}
bool testBMP(unsigned long testTimeout) {
  blueLED();
  ringBuzzer();

  unsigned long startTime = millis();

  while (millis() - startTime < testTimeout) {
    if (printBMPData()) {
      greenled();
      ringBuzzer();
      return true; // Test passed
    }
    delay(10); // Add a small delay to prevent CPU hogging
  }

  // Test timed out
  redLED();
  ringBuzzer();
  return false; // Test failed
}

bool runTest() {
  initializeMPU9250();
  readNoiseVoltage();

  // Run tests for all capabilities
  const unsigned long testTimeout = 10000; // 10 seconds timeout for each test

  bool myIMUTest = testmyIMU(testTimeout);
  bool bmpTest = testBMP(testTimeout);
  bool servoTest = testServos(testTimeout);
  bool pyroTest = testPyroChannels(testTimeout);
  bool flashTest = testFlashMemory(testTimeout);

  // Print the status of each test
  Serial.print("myIMU Test: ");
  printTestResult(myIMUTest);

  Serial.print("BMP Test: ");
  printTestResult(bmpTest);

  Serial.print("Servo Test: ");
  printTestResult(servoTest);

  Serial.print("Pyro Channels Test: ");
  printTestResult(pyroTest);

  Serial.print("Flash Memory Test: ");
  printTestResult(flashTest);

  // Save logs regardless of test results
  saveLogsToFlash();

  // Return true only if all tests passed
  return myIMUTest && bmpTest && servoTest && pyroTest && flashTest;
}
void printTestResult(bool result) {
  if (result) {
    Serial.println("Passed");
  } else {
    Serial.println("Failed");
  }
}

bool prelaunchStateSetup() {
  initializeMPU9250();
  readIMUData();
  adjustFins();
  readNoiseVoltage();
  safetyChecks();
  saveLogsToFlash();
  ringBuzzer();
  blueLED();
  Serial.println("Performing prelaunch safety checks...");

  // Check conditions for successful setup
  bool successfulSetup = (calculatePitchError(desiredPitch) == 0.0) &&
                         (calculateRollError(desiredRoll) == 0.0) &&
                         (servoXAngle == 0) &&  // Assuming servoXAngle represents the angle of the servo controlling the fins
                         (servoYAngle == 0) &&  // Assuming servoYAngle represents the angle of the servo controlling the fins
                         (readNoiseVoltage() < 1.0) &&
                         (safetyChecks());
  return successfulSetup;
}


bool launchstateSetup() {
  
  readNoiseVoltage();
  readIMUData();
  adjustFins();
  saveLogsToFlash();
  ringBuzzer();
  blueLED();
 bool successfulSetup =(calculatePitchError(desiredPitch) == 0.0) &&
                         (calculateRollError(desiredRoll) == 0.0) &&
                       (servoXAngle == 0) && // Assuming servoXAngle represents the angle of the servo controlling the fins
                       (servoYAngle == 0) && // Assuming servoYAngle represents the angle of the servo controlling the fins
                       (readNoiseVoltage() < 1.0) &&
                       (safetyChecks());

  return successfulSetup;
}

bool flightStateSetup() {
  readNoiseVoltage();
  readIMUData();
  adjustFins();
  saveLogsToFlash();
  ringBuzzer();
  blueLED();

 return true;
}

bool descentstateSetup() {
  checkParachuteDeployment();  // Check and deploy parachute if conditions are met
  saveLogsToFlash();
  ringBuzzer();
  blueLED();

  if (!checkParachuteDeployment()) {  // Check parachute deployment again
  } else {
    deployParachute();  // Deploy parachute if conditions are met
  }

  if (!deployParachute()) {  // Check if parachute deployment was successful
    deployParachute();
    return true;  // Return true if setup is successful
  } else {
    return false;  // Return false otherwise
  }
}

bool deployParachute() {
  // Add code to trigger the parachute deployment mechanism, e.g., firing a pyro channel
  // For example, if the parachute is connected to pyro channel one:
  digitalWrite(pyroPins[19], HIGH);  // Assuming pyro channel one is connected to pyroPins[0]

  // Assuming parachute deployment is successful
  Serial.println("Parachute deployed!");
  saveLogsToFlash();
  greenled(); // Assuming you have a green LED function to turn on the green LED
  ringBuzzer(); // Assuming you have a function to produce a short buzzer sound

  // Return true to indicate successful parachute deployment
  return true;
}


void blueLED() {
  digitalWrite(bluePin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, HIGH);
}

void greenled() {
  digitalWrite(bluePin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, HIGH);
}

void redLED() {
  digitalWrite(bluePin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, LOW);
}

bool testmyIMU(unsigned long testTimeout) {
  blueLED();
  ringBuzzer();

  unsigned long startTime = millis();

  while (millis() - startTime < testTimeout) {
    if (readIMUData()) {
      greenled();
      ringBuzzer();
      return true; // Test passed
    }
    delay(10); // Add a small delay to prevent CPU hogging
  }

  // Test timed out
  redLED();
  ringBuzzer();
  return false; // Test failed
}


bool testGPS() {
  blueLED();
  ringBuzzer();
  if (processGPSData()) {
    greenled();
  } else {
    redLED();
  }
  ringBuzzer();
  return true;  // Replace with actual test result
}

bool testServos(unsigned long testTimeout) {
  blueLED();
  ringBuzzer();
  
  unsigned long startTime = millis();

  while (millis() - startTime < testTimeout) {
    if (adjustFins()) {
      greenled();
      ringBuzzer();
      return true; // Test passed
    } else {
      redLED();
      ringBuzzer();
      return false; // Test failed
    }
    delay(10); // Add a small delay to prevent CPU hogging
  }

  // Test timed out
  redLED();
  ringBuzzer();
  return false; // Test failed due to timeout
}


bool testPyroChannels(unsigned long testTimeout) {
  blueLED();
  ringBuzzer();

  unsigned long startTime = millis();

  for (int i = 0; i < numPins; ++i) {
    int pyroPin = pyroPins[i];

    // Apply voltage to the pyro pin
    digitalWrite(pyroPin, HIGH);
    delay(100);  // Adjust delay if needed

    // Read the voltage on the pyro pin
    float voltage = analogRead(A1) * (MAX_VOLTAGE / 1023.0);

    // Check if voltage is above the threshold
    if (voltage >= voltageThreshold) {
      Serial.println("Pyro channel_" + String(i + 1) + "_test true");
      greenled();
    } else {
      Serial.println("Pyro channel_" + String(i + 1) + "_test failed. Voltage: " + String(voltage));
      redLED();
    }

    // Turn off the pyro pin
    digitalWrite(pyroPin, LOW);
    delay(100);  // Adjust delay if needed

    // Check if the test timeout has been reached
    if (millis() - startTime >= testTimeout) {
      redLED(); // Test timed out
      ringBuzzer();
      return false; // Test failed due to timeout
    }
  }

  ringBuzzer();
  return true;  // All tests passed
}

void saveLogsToFlash() {
  // Replace this with your actual log data
  const char* logData = "Sample Log Data. Replace this with your actual log.";

  // Call the function to write log to flash
  writeLogToFlash(logData);

  // Optionally, print a message to Serial indicating successful log save
  Serial.println("Logs saved to flash memory.");
}
// New function to clear flash memory
void clearFlashMemory() {
  Serial.println("Clearing Flash Memory...");
  flash.eraseSector(logStartAddr);
  Serial.println("Flash Memory Cleared.");
}


void readLogFromFlash(char* logData, size_t logSize) {
  uint32_t logStartAddr = 0x00000;  // Starting address for the log in flash memory


  // Read the log data from flash memory
  while (flash.readByte(logEndAddr) != 0xFF && logEndAddr < (logStartAddr + logSize - 1)) {
    logEndAddr++;
  }

  // Ensure the data is null-terminated
  logSize = min(logSize, logEndAddr - logStartAddr);

  // Read multiple bytes at once using readAnything
  flash.readAnything(logStartAddr, logData);

  // Null-terminate the data
  logData[logSize] = '\0';
}

// Function to find the end address of the log data in flash memory
uint32_t findLogEndAddress() {

  // Find the first empty address in flash memory
  while (flash.readByte(logEndAddr) != 0xFF && logEndAddr < (logStartAddr + logSize)) {
    logEndAddr++;
  }
  return logEndAddr;
}

void writeLogToFlash(const char* logData) {
  // Determine the length of the logData
  size_t logLength = strlen(logData);
  
  // Write each byte of logData to flash memory
  for (size_t i = 0; i < logLength; i++) {
    flash.writeByte(logEndAddr + i, logData[i]);
  }
  
  // Update logEndAddr for the next log entry
  logEndAddr += logLength;
}




bool testFlashMemory(unsigned long testTimeout) {
  blueLED();
  ringBuzzer();

  unsigned long startTime = millis();

  const char testLetter = 'A';  // Replace with the letter you want to write and clear

  // Write the test letter to flash memory
  writeLogToFlash(String(testLetter).c_str());

  // Read the data from flash memory
  char readData[sizeof(testLetter) + 1];
  readLogFromFlash(readData, sizeof(readData)); // Ensure to call the function

  if (strcmp(readData, String(testLetter).c_str()) == 0) {
    greenled();
  } else {
    // Failed to write or read data
    redLED();
    ringBuzzer();
    delay(500);  // Adjust delay if needed
    ringBuzzer();
    Serial.println("flash_memory_test failed");
  }

  // Check if the test timeout has been reached
  if (millis() - startTime >= testTimeout) {
    redLED(); // Test timed out
    ringBuzzer();
    return false; // Test failed due to timeout
  }

  ringBuzzer();
  return true;  // Replace with actual test result
}


float readBatteryVoltage() {
    // Replace A0 with your actual analog pin used for measuring battery voltage
    int sensorValue = analogRead(16);
    // Replace 1023 with the maximum value your analog-to-digital converter can produce
    float voltage = sensorValue * (MAX_VOLTAGE / 1023.0); 
    return voltage;
}
bool safetyChecks() {
    // Check for sensor health
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;
    myIMU.readGyroData(myIMU.gyroCount);    // Read the x/y/z adc values
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    myIMU.readMagData(myIMU.magCount);      // Read the x/y/z adc values
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    
    if ((myIMU.accelCount == 0 || myIMU.gyroCount == 0 || myIMU.magCount == 0)) {
        Serial.println("Error reading from IMU. Check sensor connections.");
        // Implement appropriate actions (e.g., shut down motors, activate safety protocols)
        redLED();
        ringBuzzer();
        delay(2000);
        return false;
    }

    // Check for actuator saturation
    if (abs(servoXAngle) > 180 || abs(servoYAngle) > 180) {
        Serial.println("Actuator saturation detected. Adjust control parameters.");
        // Implement appropriate actions (e.g., adjust control gains, activate safety protocols)
        redLED();
        ringBuzzer();
        delay(2000);
        return false;
    }
    // Check for low battery voltage
    float batteryVoltage = readBatteryVoltage(); // Implement a function to read the battery voltage
    if (batteryVoltage < MIN_BATTERY_VOLTAGE) {
        Serial.println("Low battery voltage. Prepare for landing.");
        // Implement appropriate actions (e.g., initiate landing sequence, activate safety protocols)
        redLED();
        ringBuzzer();
        delay(2000);
        return false;
    }

    // Check for valid GPS data
    if (!processGPSData()) {
        Serial.println("Invalid GPS data. Check GPS module.");
        // Implement appropriate actions (e.g., activate safety protocols)
        redLED();
        ringBuzzer();
        delay(2000);
        return false;
    }

    // Additional checks can be added as needed

    // If all checks pass, turn on green LED and continue
    greenled();
    ringBuzzer();

    return true;
}

void updateMaxAltitude() {
  float currentAltitude = bmp.readAltitude(initialPressure);
  if (currentAltitude > maxAltitude) {
    maxAltitude = currentAltitude;
  }
}
bool isApogee() {

  // Read current altitude from the BMP280 sensor
  float currentAltitude = bmp.readAltitude(initialPressure);

  // Update the maximum altitude during flight
  updateMaxAltitude();

  // Check if the current altitude is less than the maximum altitude by 5 meters
  if (maxAltitude - currentAltitude >= 5.0) {
    // Apogee condition is met
    return true;
  } else {
    // Apogee condition not met
    return false;
  }
}

bool isApogeeDetected () {
  // Check for apogee and transition to descent state
  if (isApogee() && currentAltitude < PARACHUTE_DEPLOYMENT_ALTITUDE && velocity < DESCENT_VELOCITY_THRESHOLD) {
    // Deploy parachute
    Serial.println("Rocket reached apogee. Transitioning to descent state.");
    descentstateSetup();
    flightState = false;
    return true;  // Return true if setup is successful, false otherwise
  } else {
    return false;
  }
}

bool checkParachuteDeployment() {
  bool deploymentSuccessful = false;

  if (launchstatePIN.fallingEdge()) {
    float currentAltitude = bmp.readAltitude(initialPressure);
    float velocity = calculateVerticalVelocity();
    updateMaxAltitude();

    if (currentAltitude > PARACHUTE_DEPLOYMENT_ALTITUDE_MIN &&
        currentAltitude < maxAltitude - 5.0 &&
        velocity < DESCENT_VELOCITY_THRESHOLD) {
      
      if (!parachuteDeployed) {
        if (isSafeToDeployParachute()) {
          parachuteDeployed = true;
          deploymentSuccessful = true;
        } else {
          Serial.println("Parachute deployment conditions not met. Safety check failed.");
        }
      }
    } else {
      parachuteDeployed = false;
    }
  }

  return deploymentSuccessful;
}


bool isSafeToDeployParachute() {
  float velocity = calculateVerticalVelocity(); // Calculate the vertical velocity
  float noiseVoltage = readNoiseVoltage();

  // Adjust safety conditions based on your specific requirements
  if (noiseVoltage < MAX_NOISE_VOLTAGE && velocity < DESCENT_VELOCITY_THRESHOLD) {
    Serial.println("Safe to deploy parachute.");
    return true;
  } else {
    Serial.println("Unsafe to deploy parachute. High noise voltage or high descent velocity detected.");
    return false;
  }
}

float calculateVerticalVelocity() {
  static float previousAltitude = 0.0;
  static unsigned long previousTime = 0;

  float currentAltitude = bmp.readAltitude(initialPressure); // Assuming bmp is an instance of Adafruit_BMP280

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - previousTime;

  if (deltaTime > 0) {
    float velocity = (currentAltitude - previousAltitude) / (float)deltaTime * 1000.0; // Convert to meters per second
    previousAltitude = currentAltitude;
    previousTime = currentTime;
    return velocity;
  } else {
    return 0.0; // Avoid division by zero
  }
}



float readNoiseVoltage() {
    int sensorValue = analogRead(A1);
    float voltage = sensorValue * (MAX_VOLTAGE / 1023.0);
    return voltage;
}
bool printBMPData() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1019)); /* Adjusted to local forecast! */
  Serial.println(" m");

  // Assuming printing is successful
  return true;
}
bool processGPSData() {
  bool dataPrinted = false;

 if (Serial3.available()) {
  char c = Serial3.read();
  Serial3.write(c);  // Corrected from Serial3.print(c);
  dataPrinted = true;
 }

 if (Serial3.available()) {
  char c = Serial3.read();
  Serial.write(c);  // Corrected from Serial3.print(c);
  delay(100);
  dataPrinted = true;
 }


  return dataPrinted;
}
bool readIMUData() {
    // If intPin goes high, all data registers have new data
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        // Read accelerometer data
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;

        // Read gyroscope data
        myIMU.readGyroData(myIMU.gyroCount);
        myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

        // Read magnetometer data
        myIMU.readMagData(myIMU.magCount);
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

        // Update time for quaternion filter
        myIMU.updateTime();

        // Perform Mahony quaternion update
        MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD, myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);

        // Calculate Euler angles (yaw, pitch, roll) from the updated quaternion
        myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
        myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch *= RAD_TO_DEG;
        myIMU.yaw *= RAD_TO_DEG;
        myIMU.yaw -= 5.333; // Declination adjustment
        myIMU.roll *= RAD_TO_DEG;

        // Store sensor data and angles in the circular buffer
        buffer[bufferHead].ax = 1000 * myIMU.ax;
        buffer[bufferHead].ay = 1000 * myIMU.ay;
        buffer[bufferHead].az = 1000 * myIMU.az;
        buffer[bufferHead].gx = myIMU.gx;
        buffer[bufferHead].gy = myIMU.gy;
        buffer[bufferHead].gz = myIMU.gz;
        buffer[bufferHead].mx = myIMU.mx;
        buffer[bufferHead].my = myIMU.my;
        buffer[bufferHead].mz = myIMU.mz;
        buffer[bufferHead].temperature = ((float)myIMU.tempCount) / 333.87 + 21.0;
        buffer[bufferHead].yaw = myIMU.yaw;
        buffer[bufferHead].pitch = myIMU.pitch;
        buffer[bufferHead].roll = myIMU.roll;

        // Update the circular buffer head index
        bufferHead = (bufferHead + 1) % BUFFER_SIZE;

        return true; // Return true to indicate successful data read
    }

    return false; // Return false if no new data is available
}
// Function to adjust fins based on IMU data
bool adjustFins() {
    unsigned long startTime = millis();
    const unsigned long timeout = 500; // 500 milliseconds timeout

    // Check if there is valid data in the buffer
    if (bufferHead != bufferTail) {
        // Get the latest IMU data from the buffer
        IMUData data = buffer[bufferTail];
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;

        // Calculate pitch, roll, and yaw errors
         float pitchError = calculatePitchError(desiredPitch);
         float rollError = calculateRollError(desiredRoll);
         float yawError = calculateYawError(desiredYaw);

        // PID controller coefficients
        float Kp = 1.0; // Proportional gain
        float Ki = 0.0; // Integral gain
        float Kd = 0.0; // Derivative gain

        // Calculate integral and derivative terms
        integralPitchError += pitchError;
        integralRollError += rollError;
        integralYawError += yawError;
        float derivativePitchError = pitchError - previousPitchError;
        float derivativeRollError = rollError - previousRollError;
        float derivativeYawError = yawError - previousYawError;

        // Limit the integral windup
        float integratorLimit = 50.0;
        integralPitchError = constrain(integralPitchError, -integratorLimit, integratorLimit);
        integralRollError = constrain(integralRollError, -integratorLimit, integratorLimit);
        integralYawError = constrain(integralYawError, -integratorLimit, integratorLimit);

        // Calculate servo angles using PID controller
        servoXAngle = map(Kp * pitchError + Ki * integralPitchError + Kd * derivativePitchError, -90, 90, 0, 180);
        servoYAngle = map(Kp * rollError + Ki * integralRollError + Kd * derivativeRollError, -90, 90, 0, 180);
        // Add yaw error correction to one of the servo angles (e.g., servoYAngle)
        servoYAngle += map(Kp * yawError + Ki * integralYawError + Kd * derivativeYawError, -90, 90, -45, 45);

        // Update servo positions
        servoX.write(servoXAngle);
        servoY.write(servoYAngle);

        // Update previous errors for next iteration
        previousPitchError = pitchError;
        previousRollError = rollError;
        previousYawError = yawError;

        // Print errors for debugging (optional)
        Serial.print("Pitch Error: ");
        Serial.print(pitchError);
        Serial.print("\tRoll Error: ");
        Serial.print(rollError);
        Serial.print("\tYaw Error: ");
        Serial.println(yawError);

        return true; // Return true to indicate the adjustment was performed
    } else if (millis() - startTime >= timeout) {
        // Timeout reached, return false
        Serial.println("Timeout reached while reading IMU data for fin adjustment.");
        return false;
    }
    return false;
}


float calculatePitchError(float desiredPitch) {
    return desiredPitch - buffer[bufferHead].pitch;
}

float calculateRollError(float desiredRoll) {
    return desiredRoll - buffer[bufferHead].roll;
}

float calculateYawError(float desiredYaw) {
    return desiredYaw - buffer[bufferHead].yaw;

}
void setupServos() {
    // Initial servo angles
  servoX.write(0);
  servoY.write(0);
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
}
void ringBuzzer() {
    switch (buzzerState) {
    case BUZZER_OFF:
      // Buzzer is currently off
      noTone(buzzer);
      if (millis() - lastBuzzerChangeTime >= 1000) {
        // Wait for 1 second before turning the buzzer on
        buzzerState = BUZZER_ON;
        lastBuzzerChangeTime = millis();
      }
      break;

    case BUZZER_ON:
      // Buzzer is currently on
      tone(buzzer, 1000); // Send 1KHz sound signal
      if (millis() - lastBuzzerChangeTime >= buzzerOnDuration) {
        // Wait for the specified duration before turning the buzzer off
        buzzerState = BUZZER_DONE;
        lastBuzzerChangeTime = millis();
      }
      break;

    case BUZZER_DONE:
      // Buzzer is done ringing
      noTone(buzzer);
      // You can add additional logic or tasks here
      break;
  }
}

