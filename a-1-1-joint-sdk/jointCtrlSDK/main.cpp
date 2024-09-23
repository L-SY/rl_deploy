#include "serial_handle.hpp"

using namespace std;

int main(int argc, char **argv) {
    SerialHandle handle;
    motor_torque_t send_struct;
    handle.serial_init("/dev/ttyUSB0");
    uint8_t status = 0;
    uint32_t counter = 0;
    while (true)
    {
        // motor torque array;
        float _data[6] = {1,2.1,3.02,4,5,6};
        handle.create_package(_data,send_struct);

        switch(status)
        {
            case 0:
                handle.start_joint_sdk();
                counter++;
                if(counter >= 10)
                {
                    status = 1;
                }
                usleep(10000);
                break;
            case 1:
                handle.send_commond(send_struct);
                break;
        }

        //sleep(1);
        
        usleep(1500);
    }
    return 0;
}
