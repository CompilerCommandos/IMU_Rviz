#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

// Initialize the MPU6050
Adafruit_MPU6050 mpu;

// ROS NodeHandle
ros::NodeHandle nh;

// IMU message
sensor_msgs::Imu imu_msg;

// ROS Publisher for IMU data
ros::Publisher imu_pub("/imu/data", &imu_msg);

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected. Check wiring.");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Configure the MPU6050 (optional: set ranges)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize ROS
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop() {
  // Get new data from MPU6050
  sensors_event_t accel_event, gyro_event, temp_event;
  mpu.getEvent(&accel_event, &gyro_event, &temp_event);

  // Fill the IMU message with acceleration data
  imu_msg.linear_acceleration.x = accel_event.acceleration.x;
  imu_msg.linear_acceleration.y = accel_event.acceleration.y;
  imu_msg.linear_acceleration.z = accel_event.acceleration.z;

  // Fill the IMU message with gyroscope data
  imu_msg.angular_velocity.x = gyro_event.gyro.x;
  imu_msg.angular_velocity.y = gyro_event.gyro.y;
  imu_msg.angular_velocity.z = gyro_event.gyro.z;

  // The MPU6050 doesn't provide orientation data directly (no magnetometer).
  // Set orientation to zero or calculate it externally using sensor fusion algorithms like Madgwick or Mahony.
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  // Add timestamp and frame ID
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";

  // Publish the IMU data
  imu_pub.publish(&imu_msg);

  // Handle ROS callbacks
  nh.spinOnce();

  // Delay for 10ms (100Hz update rate)
  delay(10);
}
