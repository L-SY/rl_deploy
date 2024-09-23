#ifndef __JOINT_CTRL_PROTOCOL_H
#define __JOINT_CTRL_PROTOCOL_H

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}__attribute__((packed)) imu_accl_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}__attribute__((packed)) imu_gyro_t;

typedef struct
{
    int16_t  pos;
    int16_t  vel;
    int16_t  torque;
}__attribute__((packed)) motor_msgs_t;

typedef struct
{
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
}__attribute__((packed)) imu_orientation_t;

typedef struct
{
    int16_t          header;
    uint16_t       frame_cnt;
    int16_t    left_hip_tor;
    int16_t   left_knee_tor;
    int16_t  left_wheel_tor;
    int16_t   right_hip_tor;
    int16_t  right_knee_tor;
    int16_t right_wheel_tor;
    int16_t           CRC16;  
}__attribute__((packed)) motor_torque_t;

typedef struct
{   
    uint16_t              header;    //package header double check 0xBBAA
    uint16_t             frame_cnt;
    uint16_t             err_msg;    
    uint8_t               capacity;
    imu_gyro_t              gyro;
    imu_accl_t              accl;
    imu_orientation_t   orientation;
    motor_msgs_t        left_hip;           
    motor_msgs_t       left_knee;
    motor_msgs_t      left_wheel;              
    motor_msgs_t       right_hip;
    motor_msgs_t      right_knee;
    motor_msgs_t     right_wheel;
    uint16_t  CRC16;            
}__attribute__((packed)) uart_packet_t;

typedef struct
{         
    uint8_t   SOF;          //starting byte, fixed to be 0xAA
    uint16_t  LEN     : 10; //len of frame
    uint16_t  VERSION :  6; //version of the frame header, set to be 0
    uint8_t   SESSION :  5; //session ID, 0: Sender doesn't need ACKs. 1:Sender needs ACKs but can be tolerated. 2:Sender needs ACKs.*
    uint8_t   ACK     :  1; //frame type, 0: CMD, 1:ACK
    uint8_t   RES0    :  2;
    uint8_t   PADDING :  5; //len of padding data used by the Data encryption
    uint8_t   ENC     :  3; //encryption type, 0: no encryption, 1: AES encryption
    uint8_t   RES1[3];
    uint16_t  SEQ;          //frame sequence num
    uint16_t  CRC16;        //CRC16 frame header checksum
}__attribute__((packed)) OSDK_Uart_Header_t;

#endif /*__JOINT_CTRL_PROTOCOL_H*/
