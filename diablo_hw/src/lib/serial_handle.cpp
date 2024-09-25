#include "diablo_hw/lib/serial_handle.hpp"
#include "math.h"

SerialHandle::~SerialHandle(){
    mySerial.Close();
    this->rec_loop = false;
    serial_rx_thd->join();
}

void SerialHandle::serial_init(const std::string dev){
    rec_package = std::make_shared<uart_packet_t>();
    mySerial.SetDevice(dev.c_str());
    mySerial.SetBaudRate(VulcanSerial::BaudRate::B_460800);
    mySerial.SetNumDataBits(VulcanSerial::NumDataBits::EIGHT);
    mySerial.SetNumStopBits(VulcanSerial::NumStopBits::ONE);
    mySerial.Open();
    this->rec_loop = true;
    this->serial_rx_thd = std::make_shared<std::thread>(&SerialHandle::serial_recive,this);
    usleep(10000); //ensure stability
    std::cout<<"Serial port \""<<dev<<"\" connected"<<std::endl;   
}
uint32_t frame_in_cnt = 0;
uint8_t receive_test = 0;
void SerialHandle::serial_recive(void){
    uint32_t byte_micro = 1000000 * 10/460800;
    while(rec_loop){
        int start = -1;
        while(start != 0xAA){  //header AA
            start = mySerial.ReadChar();
            if(start == -1){
                std::cerr<<"Serial receive timeout occured 1!"<<std::endl;
            }
        }
        rec_buffer[0] = 0xAA;  
        usleep(byte_micro * sizeof(uart_packet_t));
        while((int)mySerial.Available() < (int)sizeof(uart_packet_t) - 1) 
        {
            usleep(byte_micro);
        }
        for(uint16_t i = 1; i < sizeof(uart_packet_t); i++)
        {
            int result = mySerial.ReadChar();
            if(result == -1){
                std::cerr<<"Serial receive timeout occured 2!"<<std::endl;
		        continue;
            }
            rec_buffer[i] = result;
        }
        if(rec_buffer[1] != 0xBB) continue; //header double check.
        
        if(receive_test == 0)
        {
            for(uint8_t i = 0;i< sizeof(uart_packet_t);i++)
            {
//                std::cout << dec << (uint32_t)rec_buffer[i] << "\t"  << dec << (uint32_t)i<< std::endl;
            }
            receive_test = 1;
        }
        // Decode the packet and print in here
        if(JOINT_CTRL::verify_crc16(rec_buffer,sizeof(uart_packet_t)))
        {
//            std::cout<<  "Accl :\t "
//            << ((uart_packet_t*)(rec_buffer.get()))->accl.x / 1638.5f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->accl.y / 1638.5f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->accl.z / 1638.5f << "\r\n"
//            << std::endl;
//            std::cout<<  "Gyro :\t "
//            << ((uart_packet_t*)(rec_buffer.get()))->gyro.x / 327.67f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->gyro.y / 327.67f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->gyro.z / 327.67f << "\r\n"
//            << std::endl;
//            std::cout<<  "Quat :\t "
//            << ((uart_packet_t*)(rec_buffer.get()))->orientation.w / 32767.f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->orientation.x / 32767.f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->orientation.y / 32767.f<< "\t\t"
//            << ((uart_packet_t*)(rec_buffer.get()))->orientation.z / 32767.f << "\r\n"
//            << std::endl;
            continue;
        }
        rec_package = std::shared_ptr<uart_packet_t>((uart_packet_t*)(rec_buffer));
    }
}


uint16_t torque_2_package(_Float32 value){
    uint16_t out_buf;
    if(value >= 50.f)    value = 50.f;
    else if(value <= -50.f) value = -50.f;
    int16_t data = (int)(value*655.34f);// 32767 / 50
    return data;
}

void SerialHandle::create_package(_Float32* motor_tor,motor_torque_t &motor_package){
    motor_package.header = 0xBBAA;
    motor_package.frame_cnt = send_cnt;
    motor_package.left_hip_tor = torque_2_package(motor_tor[0]);
    motor_package.left_knee_tor = torque_2_package(motor_tor[1]);
    motor_package.left_wheel_tor = torque_2_package(motor_tor[2]);
    motor_package.right_hip_tor = torque_2_package(motor_tor[3]);
    motor_package.right_knee_tor = torque_2_package(motor_tor[4]);
    motor_package.right_wheel_tor = torque_2_package(motor_tor[5]);
    uint8_t serial_txbuf[100];
    memcpy(serial_txbuf, &motor_package, sizeof(motor_torque_t)-2);
    uint16_t CRC16 = JOINT_CTRL::update_crc16(serial_txbuf,sizeof(motor_torque_t)-2);
    motor_package.CRC16 = CRC16;
}   

void SerialHandle::send_commond(const motor_torque_t& ctrl_package){
    uint8_t serial_txbuf[100];
    memcpy(serial_txbuf, &ctrl_package, sizeof(motor_torque_t));
    // for(uint16_t i = 0; i < sizeof(motor_torque_t); i++){
    //     printf("0x%x,",serial_txbuf[i]);
    // }
    {
        mySerial.Write(serial_txbuf,sizeof(motor_torque_t));
        send_cnt += 1;
    }


    // if(send_cnt > 10000) send_cnt=0;
}


void SerialHandle::start_joint_sdk(void)
{
    uint8_t serial_txbuf[100];
    OSDK_Uart_Header_t *p = (OSDK_Uart_Header_t*)serial_txbuf;
    p->SOF = 0xAC;
    p->LEN = 0x0A;
    p->ACK = 1;
    p->SESSION = 0x0A;
    p->VERSION = 0x0A;

    
    //memcpy(serial_txbuf, &p, sizeof(OSDK_Uart_Header_t));
    // for(uint16_t i = 0; i < sizeof(motor_torque_t); i++){
    //     printf("0x%x,",serial_txbuf[i]);
    // }
    {
        mySerial.Write(serial_txbuf,sizeof(OSDK_Uart_Header_t));
    }


    // if(send_cnt > 10000) send_cnt=0;
}

