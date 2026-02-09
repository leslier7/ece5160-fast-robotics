//
// Functions taken from the ICM_20948 example 1
//

#include "imu_functions.h"
#include <Arduino.h>
#include <ICM_20948.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// IMU variables
ICM_20948_I2C myICM; // Create an ICM_20948_I2C object

// Below here are some helper functions to print the data nicely!

bool initIMU(ICM_20948_I2C &imu){
  // Set up I2C
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
      myICM.begin(WIRE_PORT, AD0_VAL);

      SERIAL_PORT.print(F("Initialization of the sensor returned: "));
      SERIAL_PORT.println(imu.statusString());

      if (imu.status != ICM_20948_Stat_Ok) {
          SERIAL_PORT.println("Trying again...");
          delay(500);
      }
      else {
          initialized = true;
      }
  }
  return initialized;
}

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printSerialPlot(ICM_20948_I2C *sensor) {
  // Acc (mg)
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(",");

  // Gyro (DPS)
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(",");

  // Mag (uT)
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(",");

  // Temp (C)
  printFormattedFloat(sensor->temp(), 5, 2);

  // End of sample
  SERIAL_PORT.println();
}

void printAccelPitchRoll(ICM_20948_I2C *sensor) {
  float theta = atan2(sensor->accX(), sensor->accZ()) * 57.295779513f; // pitch (in deg)
  float phi = atan2(sensor->accY(), sensor->accZ()) * 57.295779513f; // roll (in deg)
  printFormattedFloat(theta, 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(phi, 5, 2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(millis());
  SERIAL_PORT.println("");
}

void printAccelPitchRoll(const float theta, const float phi) {
  printFormattedFloat(theta, 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(phi, 5, 2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(millis());
  SERIAL_PORT.println("");
}

void printAccelPitchRollLPF(LowPass *lp_theta, LowPass *lp_phi, float theta, float phi) {
  // Print regular values
  printFormattedFloat(theta, 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(phi, 5, 2);
  SERIAL_PORT.print(",");
  // Print Low Pass values
  printFormattedFloat(lp_theta->value_n, 5, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(lp_phi->value_n, 5, 2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(millis());
  SERIAL_PORT.println("");

}

void updateLowPass(LowPass *filter, float new_value) {
    filter->value_n = filter->alpha * new_value + (1 - filter->alpha) * filter->value_n_1;
    filter->value_n_1 = filter->value_n;
}

void updateGyroAttitude(Attitude *attitude, ICM_20948_I2C &sensor, const float dt) {
    attitude->pitch += sensor.gyrY() * dt; // Integrate gyro Y to get pitch (deg)
    attitude->roll += sensor.gyrX() * dt;  // Integrate gyro X to get roll (deg)
    attitude->yaw += sensor.gyrZ() * dt;   // Integrate gyro Z to get yaw (deg)
    // Wrap yaw to 0-360
    
    if (attitude->yaw >= 360.0f) {
      attitude->yaw -= 360.0f;
    } else if (attitude->yaw < 0.0f) {
      attitude->yaw += 360.0f;
    }
}

void updateAccelAttitude(Attitude *attitude, ICM_20948_I2C &sensor) {
    attitude->pitch = calculateTheta(&sensor); //pitch (in deg)
    attitude->roll = calculatePhi(&sensor); // roll (in deg)
    // Yaw cannot be obtained from accelerometer
}

void updateCompFilter(CompFilter *filter, ICM_20948_I2C &sensor, const Attitude& accel_attitude) {
  filter->comp_attitude.pitch = (filter->comp_attitude.pitch + sensor.gyrY() * filter->dt) * (1 - filter->alpha) + accel_attitude.pitch * filter->alpha;
  filter->comp_attitude.roll = (filter->comp_attitude.roll + sensor.gyrX() * filter->dt) * (1 - filter->alpha) + accel_attitude.roll * filter->alpha;
  filter->comp_attitude.yaw += sensor.gyrZ() * filter->dt; // Yaw is only from gyro

  // Wrap yaw to 0-360
  if (filter->comp_attitude.yaw >= 360.0f) {
    filter->comp_attitude.yaw -= 360.0f;
  } else if (filter->comp_attitude.yaw < 0.0f) {
    filter->comp_attitude.yaw += 360.0f;
  }
}

void printAttitude(const Attitude& attitude) {
    printFormattedFloat(attitude.pitch, 5, 2);
    SERIAL_PORT.print(",");
    printFormattedFloat(attitude.roll, 5, 2);
    SERIAL_PORT.print(",");
    printFormattedFloat(attitude.yaw, 5, 2);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(millis());
    SERIAL_PORT.println("");
}

void printManyAttitudes(Attitude* attitude, int length){
  for (int i = 0; i < length; i++){
    printFormattedFloat(attitude[i].pitch, 5, 2);
    SERIAL_PORT.print(",");
    printFormattedFloat(attitude[i].roll, 5, 2);
    SERIAL_PORT.print(",");
    printFormattedFloat(attitude[i].yaw, 5, 2);
    SERIAL_PORT.print(",");
  }
  SERIAL_PORT.print(millis());
  SERIAL_PORT.println("");
}