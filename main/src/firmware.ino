#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "luna_robot.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif

#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20  // 50 Hz
#endif

#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif

// Safe check for createEntities(): logs error and returns false instead of hanging
#ifndef RCRETCHECK
#define RCRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; } }
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t rpm_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32MultiArray rpm_msg;

float rpm_data[4];
float current_rpm1 = 0.0f;
float current_rpm2 = 0.0f;
float current_rpm3 = 0.0f;
float current_rpm4 = 0.0f;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state = WAITING_AGENT;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

LUNA_MOTOR_DRIVE motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
LUNA_MOTOR_DRIVE motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
LUNA_MOTOR_DRIVE motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
LUNA_MOTOR_DRIVE motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P1, K_I1, K_D1);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P2, K_I2, K_D2);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P3, K_I3, K_D3);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P4, K_I4, K_D4);

Kinematics kinematics(
    Kinematics::LUNA_BASE,
    MOTOR_MAX_RPM, MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER, LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;

// ── Phase 1: Entry point ──────────────────────────────────────────────────────

void setup()
{
    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);

    bool imu_ok = imu.init();
    if (!imu_ok)
    {
        while (1) { flashLED(3); }
    }
    set_microros_serial_transports(Serial);

    rpm_msg.data.data     = rpm_data;
    rpm_msg.data.size     = 4;
    rpm_msg.data.capacity = 4;

    prev_odom_update = millis();
    prev_cmd_time    = millis();
}

// ── Phase 2: Main loop — micro-ROS state machine ──────────────────────────────

void loop()
{
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroyEntities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(CONTROL_TIMER));
            break;
        case AGENT_DISCONNECTED:
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

// ── Phase 2a: State machine helpers ──────────────────────────────────────────

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCRETCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    RCRETCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    RCRETCHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_PREFIX "imu/data"
    ));
    RCRETCHECK(rclc_publisher_init_default(
        &rpm_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_PREFIX "motor/rpm"
    ));
    RCRETCHECK(rclc_subscription_init_default(
        &twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    RCRETCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(CONTROL_TIMER), controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCRETCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(
        &executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA
    ));
    RCRETCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();
    prev_cmd_time    = millis();
    prev_odom_update = millis();
    digitalWrite(LED_PIN, HIGH);
    return true;
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true;
    if (RMW_RET_OK != rmw_uros_sync_session(timeout_ms)) return false;
    if (rmw_uros_epoch_synchronized())
    {
#if (_POSIX_TIMERS > 0)
        int64_t time_ns = rmw_uros_epoch_nanos();
        timespec tp;
        tp.tv_sec  = time_ns / 1000000000LL;
        tp.tv_nsec = time_ns % 1000000000LL;
        clock_settime(CLOCK_REALTIME, &tp);
#else
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
        time_offset = ros_time_ms - millis();
#endif
        return true;
    }
    return false;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&rpm_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    digitalWrite(LED_PIN, HIGH);
    return true;
}

void fullStop()
{
    twist_msg.linear.x  = 0.0;
    twist_msg.linear.y  = 0.0;
    twist_msg.angular.z = 0.0;
    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

// ── Phase 3: Executor callbacks ───────────────────────────────────────────────

// Called by timer every CONTROL_TIMER ms (50 Hz).
// moveBase() must run before publishData() — publish reads RPM updated by moveBase.
void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        moveBase();
        publishData();
    }
}

// Called on every new /cmd_vel message (ON_NEW_DATA).
// twist_msg is already filled by the executor before this fires.
void twistCallback(const void * msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
}

// ── Phase 4: Per-cycle control logic (called every 20 ms) ────────────────────

void moveBase()
{
    if ((millis() - prev_cmd_time) >= 200)
    {
        twist_msg.linear.x  = 0.0;
        twist_msg.linear.y  = 0.0;
        twist_msg.angular.z = 0.0;
        digitalWrite(LED_PIN, HIGH);
    }

    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z
    );

    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();
    current_rpm3 = motor3_encoder.getRPM();
    current_rpm4 = motor4_encoder.getRPM();

    if (twist_msg.linear.x == 0.0 &&
        twist_msg.linear.y == 0.0 &&
        twist_msg.angular.z == 0.0)
    {
        fullStop();
    }
    else
    {
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
        motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
        motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, current_rpm2, current_rpm3, current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0f;
    prev_odom_update = now;
    odometry.update(vel_dt, current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg  = imu.getData();

    double roll  = atan2(imu_msg.linear_acceleration.y,
                         imu_msg.linear_acceleration.z);
    double pitch = atan2(-imu_msg.linear_acceleration.x,
                         sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y +
                              imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z));
    double yaw   = 0.0;

    double cy = cos(yaw * 0.5),   sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);

    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;
    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;

    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec     = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg.header.stamp.sec      = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec  = time_stamp.tv_nsec;

    rpm_data[0] = current_rpm1;
    rpm_data[1] = current_rpm2;
    rpm_data[2] = current_rpm3;
    rpm_data[3] = current_rpm4;

    RCSOFTCHECK(rcl_publish(&imu_publisher,  &imu_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&rpm_publisher,  &rpm_msg,  NULL));
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    unsigned long long now = millis() + time_offset;
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

// ── Utilities ─────────────────────────────────────────────────────────────────

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
    delay(1000);
}

void rclErrorLoop()
{
    while (true) { flashLED(2); }
}
