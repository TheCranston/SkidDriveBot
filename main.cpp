// All the Major Includes to get the hardware rolling
// UM FeatherS3 hardware support
#include <UMS3.h>
// micro-ROS components
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
// Adafruit Unified Sensor Framework
#include <Adafruit_Sensor.h>
// Motor PWM Shield
#include <Adafruit_MotorShield.h>
// IMU BNO085
#include <Adafruit_BNO08x.h>
// Distance Sensors SR04
#include <NewPing.h>
// ESP32 hardware encoder processing
#include <ESP32Encoder.h>
// PID
#include "Pid.hpp"

#define BNO08X_RESET -1
// micro-ROS handling defs
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Globals
// Core definitions (assuming you have dual-core ESP32)
static const BaseType_t pro_cpu = 0; // wifi_core
static const BaseType_t app_cpu = 1;
const int pwm[] = {0, 1, 2, 3};
// These are measured ticks/s at full PWM speed command.  Adjust as needed for real world
const double m1_ticks = 4560.3;
const double m2_ticks = 4446.3;
const double m3_ticks = 4539.0;
const double m4_ticks = 4481.4;
const double max_m_s = 0.673288186; // based on 1400 ticks/s and 0.2042m circumference
const double min_m_s = -0.673288186;
double m1_m_s = 0;
double m2_m_s = 0;
double m3_m_s = 0;
double m4_m_s = 0;

// uRos Globals
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
// Executor
rclc_executor_t my_executor;
// publisher
rcl_publisher_t publisher;
geometry_msgs__msg__Quaternion quatMsg;
//  subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twistMsg;

time_t loop_time = 0;

char SSID[] = "WiFi";
char WIFI_PASS[] = "password";
IPAddress uROS_GATE(x, x, x, x);
size_t uROS_PORT = 8888;
int color = 0;

// Unexpected Maker platform constructor
UMS3 ums3;
// IMU constructor
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
// Motor Shield constructors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motors[] = {AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3), AFMS.getMotor(4)};
// Encoder(s) constructor
ESP32Encoder encoderM1;
ESP32Encoder encoderM2;
ESP32Encoder encoderM3;
ESP32Encoder encoderM4;
// PID class constructor - must set parameters on invocation...
MbeddedNinja::MPidNs::Pid<double> m1_pid(0.6, 0.0, 0.00001, MbeddedNinja::MPidNs::Pid<double>::ControllerDirection::PID_DIRECT, MbeddedNinja::MPidNs::Pid<double>::OutputMode::VELOCITY_PID, 50.0, min_m_s, max_m_s, 0.0);
MbeddedNinja::MPidNs::Pid<double> m2_pid(0.6, 0.0, 0.00001, MbeddedNinja::MPidNs::Pid<double>::ControllerDirection::PID_DIRECT, MbeddedNinja::MPidNs::Pid<double>::OutputMode::VELOCITY_PID, 50.0, min_m_s, max_m_s, 0.0);
MbeddedNinja::MPidNs::Pid<double> m3_pid(0.6, 0.0, 0.00001, MbeddedNinja::MPidNs::Pid<double>::ControllerDirection::PID_DIRECT, MbeddedNinja::MPidNs::Pid<double>::OutputMode::VELOCITY_PID, 50.0, min_m_s, max_m_s, 0.0);
MbeddedNinja::MPidNs::Pid<double> m4_pid(0.6, 0.0, 0.00001, MbeddedNinja::MPidNs::Pid<double>::ControllerDirection::PID_DIRECT, MbeddedNinja::MPidNs::Pid<double>::OutputMode::VELOCITY_PID, 50.0, min_m_s, max_m_s, 0.0);

void error_loop()
{
  while (1)
  {
    ums3.setPixelColor(UMS3::colorWheel(254));
    delay(500);
    ums3.setPixelColor(UMS3::colorWheel(128));
    delay(500);
  }
}

void setMotor(int dir, int pwmVal, int pwm)
{
  if (pwmVal < 25) // Deal with PID not fully seeking 0.0 set points..
  {
    pwmVal = 0;
    dir = 0;
  }
  switch (dir)
  {
  case -1:
    Motors[pwm]->run(BACKWARD);
    Motors[pwm]->setSpeed(pwmVal);
    break;

  case 1:
    Motors[pwm]->run(FORWARD);
    Motors[pwm]->setSpeed(pwmVal);
    break;

  default:
    Motors[pwm]->run(RELEASE);
    break;
  }
}

// twist message callback
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *twistMsg = (const geometry_msgs__msg__Twist *)msgin;
  if (twistMsg->linear.x == 0)
  {
    ums3.setBlueLED(LOW);
  }
  else
  {
    ums3.setBlueLED(HIGH);
  }

  // Calculate the Left and Right velocities from the TwistMsg
  double left_wheel_vel = twistMsg->linear.x - (twistMsg->angular.z * 1.8);
  double right_wheel_vel = twistMsg->linear.x + (twistMsg->angular.z * 1.8);

  // update PIDs with new commanded set points
  m1_pid.setPoint = left_wheel_vel;
  m2_pid.setPoint = right_wheel_vel;
  m3_pid.setPoint = right_wheel_vel;
  m4_pid.setPoint = left_wheel_vel;
}

void doRosTask(void *parameters) // Spin me some ROS
{
  (void)parameters;
  const TickType_t xDelay = pdMS_TO_TICKS(100); 
  while (1)
  {
    RCCHECK(rclc_executor_spin_some(&my_executor, 1)); // RCL_MS_TO_NS(50))); 
    ums3.setPixelColor(UMS3::colorWheel(color++));
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

// Rather than a timer call back, lets spin up a thread
void doPidTask(void *parameters)
{
  (void)parameters;
  double m1_hold = 0.0, m2_hold = 0.0, m3_hold = 0.0, m4_hold = 0.0;
  const double mPerInterval = 0.14586 / 50; 
  const TickType_t xDelay = pdMS_TO_TICKS(50);
  while (1)
  {
    m1_hold = (double)encoderM2.getCount() * mPerInterval;
    m2_hold = (double)encoderM1.getCount() * mPerInterval;
    m3_hold = (double)encoderM3.getCount() * mPerInterval;
    m4_hold = (double)encoderM4.getCount() * mPerInterval;
    encoderM1.clearCount();
    encoderM2.clearCount();
    encoderM3.clearCount();
    encoderM4.clearCount();

    m1_pid.Run(m1_hold);
    m2_pid.Run(m2_hold);
    m3_pid.Run(m3_hold);
    m4_pid.Run(m4_hold);

    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

void doMotorDrive(void *parameters)
{
  (void)parameters;
  int Ldir = 0, Rdir = 0, m1_pid_drive = 0, m2_pid_drive = 0, m3_pid_drive = 0, m4_pid_drive = 0;
  double m1_pid_hold = 0.0, m2_pid_hold = 0.0, m3_pid_hold = 0.0, m4_pid_hold = 0.0;
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  while (1)
  {
    Ldir = 0, Rdir = 0, m1_pid_drive = 0, m2_pid_drive = 0, m3_pid_drive = 0, m4_pid_drive = 0;

    m1_pid_hold = m1_pid.output;
    m2_pid_hold = m2_pid.output;
    m3_pid_hold = m3_pid.output;
    m4_pid_hold = m4_pid.output;

    Ldir = (m1_pid_hold > 0.0) ? 1 : -1; //  No letting front and back wheels have different directions
    Rdir = (m2_pid_hold > 0.0) ? 1 : -1; //  Use the rear wheels for decision

    m1_pid_drive = (int)constrain((abs(m1_pid_hold) * 255.0), 0.0, 255.0);
    m2_pid_drive = (int)constrain((abs(m2_pid_hold) * 255.0), 0.0, 255.0);
    m3_pid_drive = (int)constrain((abs(m3_pid_hold) * 255.0), 0.0, 255.0);
    m4_pid_drive = (int)constrain((abs(m4_pid_hold) * 255.0), 0.0, 255.0);
    setMotor(Ldir, m1_pid_drive, pwm[0]);
    setMotor(Rdir, m2_pid_drive, pwm[1]);
    setMotor(Rdir, m3_pid_drive, pwm[2]);
    setMotor(Ldir, m4_pid_drive, pwm[3]);

    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

void doIMU(void *parameters)
{
  (void)parameters;
  rcl_ret_t myErrorReturn;
  quatMsg.w = 0.0;
  quatMsg.x = 0.0;
  quatMsg.y = 0.0;
  quatMsg.z = 0.0;
  const TickType_t xDelay = pdMS_TO_TICKS(33);
  while (1)
  {
    if (bno08x.getSensorEvent(&sensorValue))
    {
      switch (sensorValue.sensorId)
      {
      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        quatMsg.w = sensorValue.un.geoMagRotationVector.real;
        quatMsg.x = sensorValue.un.geoMagRotationVector.i;
        quatMsg.y = sensorValue.un.geoMagRotationVector.j;
        quatMsg.z = sensorValue.un.geoMagRotationVector.k;

        RCSOFTCHECK( rcl_publish(&publisher, &quatMsg, NULL) );
      break;
      }
    }
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

void setup()
{
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  Serial.begin();
  while (!Serial) 
  { 
    vTaskDelay(xDelay);
  } // Hang out until the serial port is ready

  ums3.begin();
  Serial.println("UMS ESP32S3 support package started.");
  // Brightness is 0-255. We set it to 1/3 brightness here
  ums3.setPixelBrightness(255 / 3);

  // Try to initialize!
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO085 IMU!");
    error_loop();
  }
  Serial.println("IMU BNO085 Found.");

  sh2_SensorId_t reportType = SH2_GEOMAGNETIC_ROTATION_VECTOR;
  long reportIntervalUs = 33000; // Aligned with the 50ms delay in the polling task
  if (!bno08x.enableReport(reportType, reportIntervalUs))
  {
    Serial.println("Could not enable IMU report");
  }

  // FreeRTOS return var
  BaseType_t xReturned;

  set_microros_wifi_transports(SSID, WIFI_PASS, uROS_GATE, uROS_PORT);
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi Connected to Access Point.");
  }
  else
  {
    Serial.println("Cannot connect to WiFi!");
    error_loop();
  }

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  //  create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
      "orient"));

  // create timer,
  // const unsigned int timer_timeout = 10;
  // RCCHECK(rclc_timer_init_default(
  //    &timer,
  //    &support,
  //    RCL_MS_TO_NS(timer_timeout),
  //    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&my_executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&my_executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&my_executor, &subscriber, &twistMsg, &subscription_callback, ON_NEW_DATA));

  // Motor controller startup
  AFMS.begin(); // create with the default frequency 1.6KHz

  // Now for the motor encoders
  encoderM1.attachHalfQuad(18, 17);
  encoderM2.attachHalfQuad(14, 12);
  encoderM3.attachHalfQuad(5, 6);
  encoderM4.attachHalfQuad(11, 10);
  encoderM1.clearCount();
  encoderM2.clearCount();
  encoderM3.clearCount();
  encoderM4.clearCount();

  // Lets pin the main executor to core 0 right next to the wifi processing..
  xReturned = xTaskCreatePinnedToCore(doRosTask,
                                      "ROS Task",
                                      200000,
                                      NULL,
                                      1,
                                      NULL,
                                      app_cpu);

  if (xReturned == pdPASS)
  {
    Serial.println("ROS executor task successful launch.");
  }

  xReturned = xTaskCreatePinnedToCore(doPidTask,
                                      "PID Task",
                                      5200,
                                      NULL,
                                      1,
                                      NULL,
                                      tskNO_AFFINITY);
  if (xReturned == pdPASS)
  {
    Serial.println("PID task successful launch.");
  }

  xReturned = xTaskCreatePinnedToCore(doMotorDrive,
                                      "Motor Task",
                                      4800,
                                      NULL,
                                      1,
                                      NULL,
                                      tskNO_AFFINITY);
  if (xReturned == pdPASS)
  {
    Serial.println("Motor Control task successful launch.");
  }

  xReturned = xTaskCreatePinnedToCore(doIMU,
                                      "IMU Task",
                                      5400,
                                      NULL,
                                      1,
                                      NULL,
                                      tskNO_AFFINITY);
  if (xReturned == pdPASS)
  {
    Serial.println("IMU task successful launch. ");
  }
  
  // Delete loop
  Serial.println("Finished set up, Setup task bailing out...");
  vTaskDelete(NULL);
}

void loop()
{
  // Nothing here as it's spun up as individual tasks via the RTOS
}
