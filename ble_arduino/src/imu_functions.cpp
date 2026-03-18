//
// Some functions taken from the ICM_20948 example 1
// Also uses code from the DMP euler angle example
//

#include "imu_functions.h"
#include <Arduino.h>
#include <ICM_20948.h>
#include "debug.h"

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
      imu.begin(WIRE_PORT, AD0_VAL);

      SERIAL_PORT.print(F("Initialization of the IMU returned: "));
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

// Call after the IMU is already initialized
bool initDMP(ICM_20948_I2C &imu){
  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (imu.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

  if(success){
    DEBUG_PRINTLN("DMP set up!");
  } else {
    DEBUG_PRINTLN("Failed to start the DMP");
  }
  return success;

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
    attitude->pitch = calculatePitch(&sensor); //pitch (in deg)
    attitude->roll = calculateRoll(&sensor); // roll (in deg)
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

void updateDMP(Attitude *attitude, ICM_20948_I2C &sensor){
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  sensor.readDMPdataFromFIFO(&data);

  if ((sensor.status == ICM_20948_Stat_Ok) || (sensor.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      float q1 = ((float)data.Quat6.Data.Q1) / 1073741824.0; // Convert to float. Divide by 2^30
      float q2 = ((float)data.Quat6.Data.Q2) / 1073741824.0; // Convert to float. Divide by 2^30
      float q3 = ((float)data.Quat6.Data.Q3) / 1073741824.0; // Convert to float. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

      // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
      // Orientation of Axes of Sensitivity and Polarity of Rotation
      // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
      //
      // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
      // must be in aircraft reference frame.
      //  
      // We use the Tait Bryan angles (in terms of flight dynamics):
      // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
      //
      // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
      // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
      // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
      //
      // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
      // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
      // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

      // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
      // device is pointing up/down. (Gimbal lock)

      float q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      float qw = q0; // See issue #145 - thank you @Gord1
      float qx = q2;
      float qy = q1;
      float qz = -q3;

      // roll (x-axis rotation)
      float t0 = +2.0 * (qw * qx + qy * qz);
      float t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      float roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      float t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      float pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      float t3 = +2.0 * (qw * qz + qx * qy);
      float t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      float yaw = atan2(t3, t4) * 180.0 / PI;

      attitude->pitch = pitch;
      attitude->roll = roll;
      attitude->yaw = yaw;
    }
  }
}

void updateYaw(float *yaw, ICM_20948_I2C &sensor){
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  sensor.readDMPdataFromFIFO(&data);

  if ((sensor.status == ICM_20948_Stat_Ok) || (sensor.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      float q1 = ((float)data.Quat6.Data.Q1) / 1073741824.0; // Convert to float. Divide by 2^30
      float q2 = ((float)data.Quat6.Data.Q2) / 1073741824.0; // Convert to float. Divide by 2^30
      float q3 = ((float)data.Quat6.Data.Q3) / 1073741824.0; // Convert to float. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

      // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
      // Orientation of Axes of Sensitivity and Polarity of Rotation
      // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
      //
      // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
      // must be in aircraft reference frame.
      //  
      // We use the Tait Bryan angles (in terms of flight dynamics):
      // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
      //
      // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
      // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
      // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
      //
      // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
      // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
      // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

      // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
      // device is pointing up/down. (Gimbal lock)

      float q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      float qw = q0; // See issue #145 - thank you @Gord1
      float qx = q2;
      float qy = q1;
      float qz = -q3;

      // yaw (z-axis rotation)
      float t3 = +2.0 * (qw * qz + qx * qy);
      float t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      *yaw = atan2(t3, t4) * 180.0 / PI;      

      if (*yaw < 0.0f) {
        *yaw += 360.0f;
      }
    }
  }
}