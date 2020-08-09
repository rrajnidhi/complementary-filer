#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
const int MPU = 0x68; // MPU6050 I2C address




// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        //Serial.print("a/g:\t");
      //  Serial.print(ax); Serial.print("\t");
       // Serial.print(ay); Serial.print("\t");
       // Serial.print(az); Serial.print("\t");
        //Serial.print("\t");
       // Serial.print(gx); Serial.print("\t");
       // Serial.print(gy); Serial.print("\t");
      //  Serial.println(gz);



      // Get angle values from accelerometer
      
      float RADIANS_TO_DEGREES = 180/3.14159;
      float accel_angle_y = (atan(-1*ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES)+1.58;
       float accel_angle_x = (atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RADIANS_TO_DEGREES)-0.58;
       
       //Yaw cannot be calculated without a magnetometer or other external reference.

        Serial.print(accel_angle_y); Serial.print("\t");
       // Serial.print(accel_angle_x); Serial.print("\t");
        
       
        // Get angle values from gyroscope
      
        float elapsedTime, currentTime, previousTime;
        float GyroX, GyroY, GyroZ;
        float gyroAngleX, gyroAngleY, gyroAngleZ;
        float roll, pitch, yaw;
        previousTime = currentTime;        // Previous time is stored before the actual time read
        currentTime = millis();            // Current time actual time read
        elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
        Wire.beginTransmission(MPU);
        Wire.write(0x43); // Gyro data first register address 0x43
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
        GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
        GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
        GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
        
        // Correct the outputs with the calculated error values
        GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
        GyroY = GyroY - 2; // GyroErrorY ~(2)
        GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
        gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
        gyroAngleY = gyroAngleY + GyroY * elapsedTime;
        gyroAngleZ =  gyroAngleZ + GyroZ * elapsedTime;

        Serial.print( gyroAngleY); Serial.print("\t");

        
        // Complementary filter - combine acceleromter and gyro angle values
        roll = 0.96 * gyroAngleX + 0.04 * accel_angle_x;
        pitch = 0.96 * gyroAngleY + 0.04 * accel_angle_y;
        yaw=gyroAngleZ;

        Serial.println(pitch);

        


    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
