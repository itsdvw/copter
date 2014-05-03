#define EMPL_TARGET_ATMEGA328
#include <Wire.h>
#include <I2Cdev.h>

#include <helper_3dmath.h>

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

volatile unsigned char new_mpu;

unsigned long sensor_timestamp;
Quaternion qu;


const float deg_to_rad = M_PI / 180.0;
const float rad_to_deg = 180.0 / M_PI;


void setup() {
  // put your setup code here, to run once:

  Wire.begin();

  Serial.begin(9600);

  boolean mpu_initialized = false;
  while ( !mpu_initialized ) {
    if ( initialize_mpu() ) {
      mpu_initialized = true;
      enable_mpu();
    } else {
      mpu_force_reset();
      delay(100);
    }

  }
}

#define DMP_FIFO_RATE 100
#define DEFAULT_MPU_HZ    (100)
#define MPU_INT_PIN 0 // which pin is the fifo interrupt set to? interrupt pin 0 = digital pin 2
unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000

void data_ready_cb(void) { // this function is called as an interrupt when new data is available from the mpu
  new_mpu = 1;
}

boolean initialize_mpu() {

  int result;
  struct int_param_s int_param;

  /* Set up gyro.
   * Every function preceded by mpu_ is a driver function and can be found
   * in inv_mpu.h.
   */
  int_param.cb = data_ready_cb;
  int_param.pin = MPU_INT_PIN;
  result = mpu_init(&int_param);

  if ( result != 0 ) {
#ifdef serial_comm
    Serial.print("mpu_init failed!");
#endif
    return false;
  }

  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  mpu_get_sample_rate(&dmp_update_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);

  result = dmp_load_motion_driver_firmware();
  if ( result != 0 ) {
#ifdef serial_comm
    Serial.print("Firmware Load ERROR ");
    Serial.println(result);
#endif
    return false;
  }
  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
      DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
  //unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO;
  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DMP_FIFO_RATE);
  return true;
}

void disable_mpu() {
  mpu_set_dmp_state(0);
}

void enable_mpu() {
  mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
}

void loop() {
  if (new_mpu) {
    short gyro[3], accel[3], sensors;
    unsigned char more = 0;
    long quat[4];
    
    int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if (!more) {
      new_mpu = 0;
    }
    
    if ( ( success == 0 ) && ( (sensors & INV_XYZ_ACCEL) != 0 ) && ( (sensors & INV_WXYZ_QUAT) != 0 ) ) {
      qu.w = (float)(quat[0] >> 16) / 16384.0f;
      qu.x = (float)(quat[1] >> 16) / 16384.0f;
      qu.y = (float)(quat[2] >> 16) / 16384.0f;
      qu.z = (float)(quat[3] >> 16) / 16384.0f;
    }
    
    float yaw, pitch, roll;
    
    yaw = atan2(2*qu.z*qu.w - 2*qu.y*qu.x,  1 - 2*qu.z*qu.z - 2*qu.x*qu.x) * rad_to_deg;
    pitch = asin(2*qu.y*qu.z + 2*qu.x*qu.w) * rad_to_deg;
    //roll = atan2(2*qu.y*qu.w - 2*qu.z*qu.x, 1 - 2*qu.y*qu.y - 2*qu.x*qu.x) * rad_to_deg; //actual roll
    roll = asin(2*qu.x*qu.z + 2*qu.y*qu.w) * rad_to_deg; // simpler roll (ignoring pitch)
    
    Serial.print("yaw: ");
    Serial.print(yaw);
    Serial.print("\t pitch: ");
    Serial.print(pitch);
    Serial.print("\t roll: ");
    Serial.println(roll);
  }
    
}
