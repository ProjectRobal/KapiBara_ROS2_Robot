#pragma once
#pragma GCC push_options
#pragma GCC optimize ("O0")

#define PACKET_TYPE_COUNT 7

typedef enum packet_type
{
    PING=0,
    IMU=1,
    Orientation=2,
    Encoder=3,
    TOF=4,
    GENERAL_CFG_DATA=5,
    ACK=6,
} packet_type_t;

typedef struct fusion_cfg
{
    float beta;
} fusion_cfg_t ;

typedef struct aux_cfg
{
    uint8_t addresses[8];
    int32_t data[8];
} aux_cfg_t ;

typedef struct motor_cfg
{
    int32_t velocities[2];
} motor_cfg_t ;

typedef struct servo_cfg
{
    uint8_t angel[2];
} servo_cfg_t ;

typedef struct PID
{
    float p;
    float i;
    float d;
} PID_t ;

typedef struct pid_cfg
{
    PID_t pid_left;
    PID_t pid_right;
    uint8_t open;
} pid_cfg_t ;

typedef struct ack_msg
{
    char msg[2];
} ack_msg_t ;

typedef struct ping_msg
{
    char msg[2];
} ping_msg_t ;

typedef struct imu_raw_gyro
{
    float x;
    float y;
    float z;   
} imu_raw_gyro_t ;

typedef struct imu_raw_accel
{
    float x;
    float y;
    float z;
} imu_raw_accel_t ;

typedef struct imu_raw
{
    imu_raw_gyro_t gyroscope;
    imu_raw_accel_t accelerometer;
} imu_raw_t ;


typedef struct imu_cfg
{
    imu_raw_gyro_t gyroscope_offset;
    imu_raw_accel_t accelerometer_offset;
} imu_cfg_t ;

typedef struct orientation
{
    float x;
    float y;
    float z;
    float w;
    imu_raw_t imu;
} orientation_t ;

typedef struct tof
{
    uint32_t id;
    int32_t distance;
} tof_t ;

typedef struct encoder
{
    int32_t speed_left;
    int32_t speed_right;
    int32_t distance_left;
    int32_t distance_right;
} encoder_t ;

#pragma GCC pop_options