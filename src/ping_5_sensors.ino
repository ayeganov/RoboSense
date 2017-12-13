// ---------------------------------------------------------------------------
// This example code was used to successfully communicate with 15 ultrasonic sensors. You can adjust
// the number of sensors in your project by changing SONAR_NUM and the number of NewPing objects in the
// "sonar" array. You also need to change the pins for each sensor for the NewPing objects. Each sensor
// is pinged at 33ms intervals. So, one cycle of all sensors takes 495ms (33 * 15 = 495ms). The results
// are sent to the "oneSensorCycle" function which currently just displays the distance data. Your project
// would normally process the sensor results in this function (for example, decide if a robot needs to
// turn and call the turn function). Keep in mind this example is event-driven. Your complete sketch needs
// to be written so there's no "delay" commands and the loop() cycles at faster than a 33ms rate. If other
// processes take longer than 33ms, you'll need to increase PING_INTERVAL so it doesn't get behind.
// ---------------------------------------------------------------------------
#include <Wire.h>
#include "NewPing.h"
#include "Arduino.h"
#include "LSM9DS0.h"

// IMU variables and functions
#define AA  0.97         // complementary filter constant
#define G_GAIN 0.070    // [deg/s/LSB]

unsigned long startTime;
byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;   // [deg/s]
float rate_gyr_x = 0.0;    // [deg/s]
float rate_gyr_z = 0.0;     // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;

void writeTo(int device, byte address, byte val);
void readFrom(int device, byte address, int num, byte buff[]);

// Ultrasonic variables
#define SONAR_NUM     5  // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 29 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
long BAUD_RATE = 115200;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
    NewPing(29, 28, MAX_DISTANCE),
    NewPing(31, 30, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(33, 32, MAX_DISTANCE),
    NewPing(35, 34, MAX_DISTANCE),
    NewPing(37, 36, MAX_DISTANCE)
};

void setup()
{
    Wire.begin();        // join i2c bus (address optional for master)
    Serial.begin(BAUD_RATE);
    pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    {
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    }

    //Enable accelerometer
    writeTo(ACC_ADDRESS,CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
    writeTo(ACC_ADDRESS,CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

    //Enable the magnetometer
    writeTo(MAG_ADDRESS,CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
    writeTo(MAG_ADDRESS,CTRL_REG6_XM, 0b01100000);   // +/-12gauss
    writeTo(MAG_ADDRESS,CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode

    // Enable Gyro
    writeTo(GYR_ADDRESS, CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
    writeTo(GYR_ADDRESS, CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
    startTime = millis();
}

void loop()
{
    bool cycle_complete = false;
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
      // Loop through all the sensors.
      if (millis() >= pingTimer[i])
      {
        // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.

        cycle_complete = i == 0 && currentSensor == SONAR_NUM - 1;

        if (cycle_complete)
        {
            oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
            cycle_complete = true;
        }

        sonar[currentSensor].timer_stop(); // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                 // Sensor being accessed.
        cm[currentSensor] = 0;             // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }

    // if sensor cycle finished append the heading to the reading as well
    if(cycle_complete)
    {
        readFrom(ACC_ADDRESS, 0x80 | OUT_X_L_A, 6, buff);
        accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
        accRaw[1] = (int)(buff[2] | (buff[3] << 8));
        accRaw[2] = (int)(buff[4] | (buff[5] << 8));

        readFrom(GYR_ADDRESS, 0x80 | OUT_X_L_G, 6, buff);
        gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));
        gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
        gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));

        readFrom(MAG_ADDRESS, 0x80 | OUT_X_L_M, 6, buff);
        magRaw[0] = (int)(buff[0] | (buff[1] << 8));
        magRaw[1] = (int)(buff[2] | (buff[3] << 8));
        magRaw[2] = (int)(buff[4] | (buff[5] << 8));

        //Convert Gyro raw to degrees per second
        rate_gyr_x = (float) gyrRaw[0] * G_GAIN;
        rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
        rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

        float DT = (millis() - startTime) / 1000.0;
        //Calculate the angles from the gyro
        gyroXangle+=rate_gyr_x*DT;
        gyroYangle+=rate_gyr_y*DT;
        gyroZangle+=rate_gyr_z*DT;

        //Convert Accelerometer values to degrees
        AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
        AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;

        //If IMU is up the correct way, use these lines
        AccXangle -= (float)180.0;
        if (AccYangle > 90)
        {
            AccYangle -= (float)270;
        }
        else
        {
            AccYangle += (float)90;
        }

        //Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
        CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;

        //Compute heading
        float heading = atan2(magRaw[1],magRaw[0]);

        Serial.print(heading); Serial.print(" ");
        Serial.print(rate_gyr_x); Serial.print(" ");
        Serial.print(rate_gyr_y); Serial.print(" ");
        Serial.print(rate_gyr_z); Serial.print(" ");

        Serial.print(gyroXangle); Serial.print(" ");
        Serial.print(gyroYangle); Serial.print(" ");
        Serial.print(gyroZangle); Serial.print(" ");

        Serial.print(AccYangle); Serial.print(" ");
        Serial.print(AccXangle); Serial.print(" ");

        Serial.print(CFangleX); Serial.print(" ");
        Serial.print(CFangleY);

        Serial.println();
        startTime = millis();
    }
}


void echoCheck()
{
    // If ping received, set the sensor distance to array.
    if (sonar[currentSensor].check_timer())
    {
        cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    }
}

void oneSensorCycle()
{
    // Sensor ping cycle complete, do something with the results.
    // The following code would be replaced with your code that does something with the ping results.
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        Serial.print(cm[i]);
        if(i < SONAR_NUM) Serial.print(" ");
    }
}

void writeTo(int device, byte address, byte val)
{
    Wire.beginTransmission(device); //start transmission to device 
    Wire.write(address);        // send register address
    Wire.write(val);        // send value to write
    Wire.endTransmission(); //end transmission
}

void readFrom(int device, byte address, int num, byte buff[])
{
    Wire.beginTransmission(device); //start transmission to device 
    Wire.write(address);        //sends address to read from
    Wire.endTransmission(); //end transmission

    Wire.beginTransmission(device); //start transmission to device (initiate again)
    Wire.requestFrom(device, num);    // request 6 bytes from device

    int i = 0;
    while(Wire.available())    //device may send less than requested (abnormal)
    {
        buff[i] = Wire.read(); // receive a byte
        i++;
    }
    Wire.endTransmission(); //end transmission
}
