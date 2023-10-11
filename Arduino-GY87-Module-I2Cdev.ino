/******************************************************************
// My learning Sources
MPU6050 Datasheet https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
HM5883L Datasheet https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
BMP180 Datasheet https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
https://www.i2cdevlib.com/
******************************************************************/

// Libraries used
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"

const char LED = 13;

// Constant for MPU6050
const float ACCEL_SENS = 16384.0;  // Accel Sensitivity with default 0 | +-2g scale (1 | +-4g = 8192.0, 2 | +-8g = 4096.0, 3 | +-16g = 2048.0)
const float GYRO_SENS = 131.0;     // Gyro Sensitivity with default 0 | +-250°/s scale (1 | +-500°/s = 65.5, 2 | +-1000°/s = 32.8, 3 | +-2000°/s = 16.4)

// Create object
MPU6050 accelgyro;  // Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
HMC5883L mag;       // Magnetometer class default I2C address is 0x1E
BMP085 barometer;   // Barometer class default I2C address is 0x77

// Create variables
// MPU6050
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accelX, accelY, accelZ;
float gyroRoll, gyroPitch, gyroYaw;
float angleRoll, anglePitch;
float mpuTemp;

// Set accelerometer measurement offset
float accelXOffset = 0.0;
float accelYOffset = 0.0;
float accelZOffset = 0.0;

// Set gyroscope measurement offset
float gyroXOffset = 0.0;
float gyroYOffset = 0.0;
float gyroZOffset = 0.0;

// HMC5883L
int16_t mx, my, mz;
float heading;

// BMP180
float bmpTemp;
float pressure;
float altitude;

// For millis timer
unsigned long previousMillis = 0;
unsigned long intervalMillis = 100;
bool blinkState = false;

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  unsigned int count = 0;
  while (!Serial && (count < 30)) {
    delay(200);  // Wait for serial port to connect with timeout. Needed for native USB
    digitalWrite(LED, blinkState);
    blinkState = !blinkState;
    count++;
  }
  digitalWrite(LED, HIGH);

  Wire.begin();  // Join I2C bus (I2Cdev library doesn't do this automatically)
  // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Initialize MPU6050
  accelgyro.initialize();
  Serial.print("Testing Accel/Gyro... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // Change the sensitivity according to set here
  accelgyro.setFullScaleAccelRange(0);  // 0 | +-2g scale is default
  accelgyro.setFullScaleGyroRange(0);   // 0 | +-250°/s scale is default

  // Set the bypass mode to use auxiliary SDA SCL (XDA XCL) for gateway to HMC5883L sensor (magnetometer)
  accelgyro.setI2CBypassEnabled(true);

  // Initialize HMC5883L
  mag.initialize();
  Serial.print("Testing Mag... ");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  // Choose a lower gain value (higher GN#) when total field strength causes overflow in one of the data output registers (saturation)
  mag.setGain(1);  // 1 | +-1.3Ga is default (0 | +-0.88Ga, 2 | +-1.9Ga, 3 | +-2.5Ga, 4 | +-4.0Ga, 5 | +-4.7Ga, 6 | +-5.6Ga, 7 | +-8.1Ga)

  // Initialize BMP180
  barometer.initialize();
  Serial.print("Testing Pressure... ");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

  // Accel Range, Gyro Range, Magnet Gain
  Serial.print("Acceleration scale range: ");
  Serial.print(accelgyro.getFullScaleAccelRange());
  Serial.print("\t");
  Serial.print("Gyroscope scale range: ");
  Serial.print(accelgyro.getFullScaleGyroRange());
  Serial.print("\t");
  Serial.print("Magnetation gain: ");
  Serial.print(mag.getGain());
  Serial.println();
  Serial.println("Setup Complete");
  delay(4000);
}

void loop() {
  if (millis() - previousMillis > intervalMillis) {
    previousMillis = millis();
    
    // MPU6050
    // read raw accel/gyro measurements
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate accel/gyro measurements with offset value if needed
    accelX = ax / ACCEL_SENS + accelXOffset;
    accelY = ay / ACCEL_SENS + accelYOffset;
    accelZ = az / ACCEL_SENS + accelZOffset;
    gyroRoll = gx / GYRO_SENS + gyroXOffset;
    gyroPitch = gy / GYRO_SENS + gyroYOffset;
    gyroYaw = gz / GYRO_SENS + gyroZOffset;

    // Calculate the absolute angles
    angleRoll = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 1 / (M_PI / 180);
    anglePitch = -atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 1 / (M_PI / 180);

    // Read MPU6050 internal temperature
    mpuTemp = accelgyro.getTemperature();
    mpuTemp = mpuTemp / 340.0 + 36.53; // Convert the measurement units to temperature in degrees °C (formula from the datasheet)

    // HMC5883L
    // read raw heading measurements
    mag.getHeading(&mx, &my, &mz);

    // To calculate heading in degrees. 0 degree indicates North
    heading = atan2(my, mx);
    if (heading < 0) heading += 2 * M_PI;
    heading *= 180 / M_PI;

    // BMP180
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);

    // read calibrated temperature value in degrees Celsius
    bmpTemp = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();

    // calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(pressure);

    // Print to serial all measurements
    // Get Acceleration offset value from here
    // there must be an offset if g != 0 when accelX static or g != +-1.0 when accelX fully roll
    // there must be an offset if g != 0 when accelY static or g != +-1.0 when accelY fully pitch
    // there must be an offset if g != 0 when accelZ fully roll/pitch or g != +-1.0 when accelZ static
    Serial.print("Acceleration X [g]: ");
    Serial.print(accelX);
    Serial.print("\t");
    Serial.print("Acceleration Y [g]: ");
    Serial.print(accelY);
    Serial.print("\t");
    Serial.print("Acceleration Z [g]: ");
    Serial.println(accelZ);

    // Get Gyroscope offset value from here
    // there must be an offset if °/s != 0 when gyroRoll static
    // there must be an offset if °/s != 0 when gyroPitch static
    // there must be an offset if °/s != 0 when gyroYaw static
    Serial.print("Gyro Roll [°/s]= ");
    Serial.print(gyroRoll);
    Serial.print("\t");
    Serial.print("Gyro Pitch [°/s]= ");
    Serial.print(gyroPitch);
    Serial.print("\t");
    Serial.print("Gyro Yaw [°/s]= ");
    Serial.println(gyroYaw);

    Serial.print("Roll Angle [°]: ");
    Serial.print(angleRoll);  // Roll angle value
    Serial.print("\t");
    Serial.print("Pitch Angle [°]: ");
    Serial.println(anglePitch);  // Pitch angle value

    Serial.print("MPU6050 Temperature [°C]: ");
    Serial.print(mpuTemp);
    Serial.print("\t");
    Serial.print("Heading [°]: ");
    Serial.print(heading);
    Serial.print("\t");
    Serial.print("BMP180 Temperature [°C]: ");
    Serial.print(bmpTemp);
    Serial.print("\t");
    Serial.print("Pressure [Pa]: ");
    Serial.print(pressure);
    Serial.print("\t");
    Serial.print("Altitude [m]: ");
    Serial.println(altitude);

    Serial.println();

    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
  }
}
