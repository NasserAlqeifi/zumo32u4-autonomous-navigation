#include <Wire.h>
#include <Zumo32U4.h>
#include "1euroFilter.h"
#include <MadgwickAHRS.h>

//PD stuff 
const uint16_t maxSpeed = 400;   // Maximum motor speed
const int GOAL = 2000;

#define NUM_SENSORS 5  
unsigned int lineSensorValues[NUM_SENSORS];
int16_t lastError = 0;


 // Desired line sensor position and proxSensors 
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;
Madgwick filter;

// Create a OneEuroFilter object for each axis of the accelerometer
OneEuroFilter filterX, filterY, filterZ;

// Frequency and other parameters for the 1 Euro Filters
#define FREQUENCY   60   // Hz
#define MINCUTOFF   3.0  // Hz
#define BETA        0.1  

unsigned long microsPerReading, microsPrevious;
float roll, pitch, heading;
const uint16_t turnSpeed = 200; // Speed for turning


void calibrateSensors() {
    delay(1000);
    for (uint16_t i = 0; i < 120; i++) {
        if (i > 30 && i <= 90) {
            motors.setSpeeds(-200, 200);
        } else {
            motors.setSpeeds(200, -200);
        }
        lineSensors.calibrate();
    }
    motors.setSpeeds(0, 0);
}


void setup() {
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  calibrateSensors();
  // start the IMU and filter
  Wire.begin();
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  filter.begin(25);

  // Initialize filters for each axis
  filterX.begin(FREQUENCY, MINCUTOFF,  BETA);
  filterY.begin(FREQUENCY, MINCUTOFF, BETA);
  filterZ.begin(FREQUENCY, MINCUTOFF, BETA);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  
  proxSensors.read();
    int center_left_sensor = proxSensors.countsFrontWithLeftLeds();
    int center_right_sensor = proxSensors.countsFrontWithRightLeds();

    // Calculate proximity factor based on sensor readings
    float proximityFactor = calculateProximityFactor(center_left_sensor, center_right_sensor);

    // Check if ProximityFactor is 0
    if (proximityFactor == 0) {
        // Stop the motors
        motors.setSpeeds(0, 0);

        // Delay for 5 seconds
        delay(5000);

        // Recheck proximity after delay
        proxSensors.read();
        center_left_sensor = proxSensors.countsFrontWithLeftLeds();
        center_right_sensor = proxSensors.countsFrontWithRightLeds();
        proximityFactor = calculateProximityFactor(center_left_sensor, center_right_sensor);

        if (proximityFactor == 0) {
            // If still blocked, turn 180 degrees again
            turnRobotToHeading(180.0);
        }

        // Reset lastError
        lastError = 0;
    } 

    if (proximityFactor != 0) {
        // Existing code for line sensor values
        int16_t position = lineSensors.readLine(lineSensorValues);

        // Existing code for calculating error and motor speeds
        int16_t error = position - GOAL;
        int16_t adjustment = error / 3.5 + 0.5 * (error - lastError);
        lastError = error;

        // Adjust speed based on proximity factor
        int16_t speed = maxSpeed * proximityFactor;

        // Possibly adjust line following error based on proximity factor
        adjustment = adjustment * proximityFactor;

        // Calculate motor speeds
        int16_t leftSpeed = (int16_t)speed + adjustment;
        int16_t rightSpeed = (int16_t)speed - adjustment;

        // Constrain speeds
        leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
        rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

        // Set motor speeds
        motors.setSpeeds(leftSpeed, rightSpeed);
    }


}

float calculateProximityFactor(int leftSensor, int rightSensor) {
    // Define sensor thresholds and maximum proximity factor
    const int sensorThreshold = 6; // Adjust this based on your sensors
    const float maxProximityFactor = 1.0;
    const float minProximityFactor = 0.0;

    // Calculate the factor based on sensor readings
   int sensorValue = min(leftSensor, rightSensor);

    float factor = (float)(sensorThreshold - sensorValue) / sensorThreshold;

    // Ensure factor is within valid range
    return constrain(factor, minProximityFactor, maxProximityFactor);
}


float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 245.0) / 32768.0;
  return g;
}

void AHRS() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    imu.read();
    aix = imu.a.x;
    aiy = imu.a.y;
    aiz = imu.a.z;
    gix = imu.g.x;
    giy = imu.g.y;
    giz = imu.g.z;

    // Apply the 1euroFilter to the raw accelerometer data
    float filteredAx = filterX.filter(aix);
    float filteredAy = filterY.filter(aiy);
    float filteredAz = filterZ.filter(aiz);

    // Convert filtered raw data to gravity units
    ax = convertRawAcceleration(filteredAx);
    ay = convertRawAcceleration(filteredAy);
    az = convertRawAcceleration(filteredAz);

    // Convert raw gyro data to degrees/second
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);


    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
/*
    // Print the roll, pitch, and heading values
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Heading: ");
    Serial.println(heading);
*/

    microsPrevious = microsPrevious + microsPerReading;
  }
}

void turnRobotToHeading(float turnAngle) {
// Constants
const float MAX_HEADING = 360.0;
const float HEADING_THRESHOLD = 10;
const int BASE_SPEED = 100;
const int MAX_SPEED = 400;
const float PROPORTIONAL_GAIN = 2.0;

// Calculate the effective target heading, allowing values over 360
float effectiveTargetHeading = heading + turnAngle;

// Normalize target heading to be within the range [0, 360)
float normalizedTargetHeading = fmod(effectiveTargetHeading, MAX_HEADING);
if (normalizedTargetHeading < 0) normalizedTargetHeading += MAX_HEADING;

while (true) {
    AHRS(); // Update the heading

    float currentError = fmod(heading - normalizedTargetHeading + MAX_HEADING, MAX_HEADING);
    if (currentError > 180) {
        currentError -= MAX_HEADING; // Adjust for the shortest turning direction
    }

    // Exit condition
    if (abs(currentError) <= HEADING_THRESHOLD) {
        break;
    }

    // Proportional control for speed adjustment
    int speed = max(BASE_SPEED, MAX_SPEED - abs(currentError) * PROPORTIONAL_GAIN);

    motors.setSpeeds(-speed, speed);

     delay(20); // Small delay to prevent too frequent updates
    }
}


