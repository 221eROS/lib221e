#ifndef MITCH_HW_H
#define MITCH_HW_H

#include <serial/SerialConnection.h>

namespace Mitch
{
    /**
    * @brief Hardware related communication protocol specifications
    */

    class Mitch_HW
    {
    public:
        static const int COMM_MESSAGE_LEN = 20;	            // Overall message length [Bytes]
        static const int COMM_MSG_HEADER_LEN_4 = 4;
        static const int COMM_MSG_HEADER_LEN_2 = 2;

        static const int COMM_PAYLOAD_LEN_18 = 18;          // Command, Payload maximum length [Bytes]
        static const int COMM_PAYLOAD_LEN_16 = 16;          // Response, Payload maximum length [Bytes]

        static const int NUM_OF_CHANNELS_9DOF = 3;
        static const int NUM_OF_CHANNELS_TOF = 2;

        enum Command : uint8_t
        {
            CMD_ACK = 0x00,                                 
            CMD_SHUTDOWN = 0x01,                            
            CMD_STATE = 0x02,                               
            CMD_RESTART = 0x03,                             
            CMD_APP_CRC = 0x04,                            
            CMD_FW_UPLOAD = 0x05,                           
            CMD_START_APP = 0x06,                           
            CMD_BATTERY_CHARGE = 0x07,                     
            CMD_BATTERY_VOLTAGE = 0x08,                    
            CMD_CHECK_UP = 0x09,                            
            CMD_FW_VERSION = 0x0A,                         
            CMD_TIME = 0x0B,                                
            CMD_BLE_NAME = 0x0C,                            
            CMD_HW_VERSION = 0x0D,                         
            CMD_DEVICE_ID = 0x0E,                           

            // memory
            CMD_MEM_CONTROL = 0x20,                         
            CMD_MEM_FILE_INFO = 0x21,                       
            CMD_MEM_FILE_DOWNLOAD = 0x22,                  

            // time sync
            CMD_CLK_DRIFT = 0x30,                           
            CMD_CLK_OFFSET = 0x31,                         
            CMD_TIME_SYNC = 0x32,                           
            CMD_EXIT_TIME_SYNC = 0x33,                      

            // IMU
            CMD_FS_AXL_GYRO = 0x40,                         
            CMD_FS_AXL = 0x41,                              
            CMD_FS_GYRO = 0x42,                             

            // Autostart
            CMD_AUTO_START = 0x43,                          

            // TOF sensors
            CMD_FS_DS1 = 0x44,                             
            CMD_FS_DS2 = 0x45,                              
            CMD_OFFSET_DS1 = 0x46,                          
            CMD_OFFSET_DS2 = 0x47,                          

            // Calibration matrices
            CMD_MATRIX_CALIBRATION = 0x48,                  

            //Other
            CMD_BTN_LOG = 0x50                             
        };

        enum Acknowledge_Type : uint8_t
        {
            CMD_ACK_SUCCESS = 0x00,                         
            CMD_ACK_ERROR = 0x01                            
        };

        enum SystemState : uint8_t
        {
            SYS_NULL = 0x00,    
            SYS_BOOT_STARTUP = 0xF0,                        
            SYS_BOOT_IDLE = 0xF1,                           
            SYS_BOOT_WRITE = 0xF2,                          
            SYS_ERROR = 0xFF,                               
            SYS_STARTUP = 0x01,                            
            SYS_IDLE = 0x02,                               
            SYS_STANDBY = 0x03,                           
            SYS_LOG = 0x04,                               
            SYS_READOUT = 0x05,                             
            SYS_TX = 0xF8                                  
        };

        enum RestartMode : uint8_t
        {
            RESTART_RESET = 0x00,                           
            RESTART_BOOT_LOADER = 0x01                      
        };

        enum LogMode : uint8_t
        {
            LOG_MODE_NONE = 0x00,      				       
            LOG_MODE_IMU = 0x01,
            LOG_MODE_IMU_INSOLE = 0x02,
            LOG_MODE_ALL = 0x03,
            LOG_MODE_IMU_TIMESTAMP = 0x04,
            LOG_MODE_IMU_INSOLE_TIMPESTAMP = 0x05,
            LOG_MODE_ALL_TIMESTAMP = 0x06
        };

        enum LogFrequency : uint8_t
        {
            LOG_FREQ_NONE = 0x00,                           
            LOG_FREQ_25HZ = 0x01,                           
            LOG_FREQ_50HZ = 0x02,                           
            LOG_FREQ_100HZ = 0x04,                         
            LOG_FREQ_200HZ = 0x08,                         
            LOG_FREQ_500HZ = 0x14,                         
            LOG_FREQ_1000HZ = 0x28                          
        };

        enum LogPacketDimension : uint8_t
        {
            LOG_PACKET_DIM_NONE = 0,                        
            LOG_PACKET_DIM_IMU = 18,                        
            LOG_PACKET_DIM_IMU_INSOLE = 42,                 
            LOG_PACKET_DIM_ALL = 42,                       
            LOG_PACKET_DIM_IMU_withTimestamp = 26,          
            LOG_PACKET_DIM_IMU_INSOLE_withTimestamp = 50,   
            LOG_PACKET_DIM_ALL_withTimestamp = 52          
        };

        enum StreamMode : uint8_t
        {
            STREAM_MODE_NONE = 0x00,                        
            STREAM_MODE_PRESSURE = 0x01,                   
            STREAM_MODE_6DOF_TOF = 0x02,                    
            STREAM_MODE_TOF = 0x03,                         
            STREAM_MODE_6DOF = 0x04,                        
            STREAM_MODE_9DOF = 0x05,                        
            STREAM_MODE_6DOFs_ORIENTATION = 0x06,           
            STREAM_MODE_ORIENTATION = 0x07	                
        };

        enum StreamFrequency : uint8_t
        {
            STREAM_FREQ_NONE = 0x00,                       
            STREAM_FREQ_5Hz = 0x01,                         
            STREAM_FREQ_10Hz = 0x02,                        
            STREAM_FREQ_25Hz = 0x03,                        
            STREAM_FREQ_50Hz = 0x04                        
        };

        enum StreamPacketDimension : uint8_t
        {
            STREAM_PACKET_DIM_NONE = 0,                    
            STREAM_PACKET_DIM_PRESSURE = 18,                
            STREAM_PACKET_DIM_6DOF_TOF = 16,               
            STREAM_PACKET_DIM_TOF = 4,                      
            STREAM_PACKET_DIM_6DOF = 14,                    
            STREAM_PACKET_DIM_9DOF = 18                     
        };

        enum MemorySelection : uint8_t
        {
            MEMORY_ONE = 0x01,                             
            MEMORY_TWO = 0x02                              
        };

        enum MemoryErase_Type : uint8_t
        {
            ERASE_NONE = 0x00,                              // Erase is not ongoing
            ERASE_PARTIAL = 0x01,                           // Partial erase of the flash memory, from the beginning to the last sector occupied
            ERASE_BULK1 = 0x02,                             // Total erase of the memory
            ERASE_BULK2 = 0x03                              // Total erase of the memory
        };

        enum Gyroscope_FS : uint8_t
        {
            GYR_FS_NULL = 0xFF,
            GYR_FS_245_DPS = 0x00,
            GYR_FS_500_DPS = 0x04,
            GYR_FS_1000_DPS = 0x08,
            GYR_FS_2000_DPS = 0x0C
        };

        // Gyroscope resolution
        static constexpr float GYR_RESOLUTION_245dps = 0.00875f;
        static constexpr float GYR_RESOLUTION_500dps = 0.0175f;
        static constexpr float GYR_RESOLUTION_1000dps = 0.035f;
        static constexpr float GYR_RESOLUTION_2000dps = 0.07f;

        enum Accelerometer_FS : uint8_t
        {
            AXL_FS_NULL = 0xFF,
            AXL_FS_2_g = 0x00,
            AXL_FS_4_g = 0x08,
            AXL_FS_8_g = 0x0C,
            AXL_FS_16_g = 0x04
        };

        // Accelerometer resolution
        static constexpr float AXL_RESOLUTION_2g = 0.061f;
        static constexpr float AXL_RESOLUTION_4g = 0.122f;
        static constexpr float AXL_RESOLUTION_8g = 0.244f;
        static constexpr float AXL_RESOLUTION_16g = 0.488f;

        // Magnetometer resolution
        static constexpr float MAG_RESOLUTION = 1.5f;

        enum TOF_FS : uint8_t
        {
            TOF_FS_NULL = 0x00,
            TOF_FS_200mm = 0x01,                           
            TOF_FS_400mm = 0x02,                            
            TOF_FS_600mm = 0x03                            
        };

        enum CalibMatrixType : uint8_t
        {
            AXL_MATRIX = 0x00,
            GYR_MATRIX = 0x01,
            MAG_MATRIX = 0x02
        };

    };
}
#endif