#ifndef MITCH_V2_DATA_H
#define MITCH_V2_DATA_H

#include <sstream>
#include <array>

namespace MitchV2
{
    /**
     * @brief A Mitch Data object
     * 
     * The object lets manage Mitch data such as 
     * - device ID: the unique device identifier;
     * - timestamp: the current date / time in epoch format;
     * - gyroscope: the gyroscope reading around x, y, z axes;
     * - accelerometer: the accelerometer reading along x, y, z axes;
     * - magnetometer: the magnetometer reading along x, y, z axes;
     * - Time Of Flight (TOF): the TOF reading of peripheral #1 and #2, respectively;
     * - pressure: the 16-channels insole pressure reading.
     * 
     */
    class MitchV2_Data
    {
    private:
        unsigned short device_id_;
        uint64_t timestamp_;
        std::array<float, 3> gyroscope_;
        std::array<float, 3>  accelerometer_;
        std::array<float, 3>  magnetometer_;
        std::array<uint8_t, 2>  tof_;
        std::array<float, 16>  pressure_;
    public:

        void setDeviceID(unsigned short value){device_id_ = value;}
        unsigned short getDeviceID(){return device_id_;}
        
        void setTimestamp(uint64_t value){timestamp_ = value;}
        uint64_t getTimestamp(){return timestamp_;}
       
        void setGyr(std::array<float, 3>  value){gyroscope_ = value;}
        std::array<float, 3> getGyr(){return gyroscope_;}
   
        void setAxl(std::array<float, 3> value){accelerometer_  = value;}
        std::array<float, 3> getAxl(){return accelerometer_;}
        
        void setMag(std::array<float, 3> value){magnetometer_ = value;}
        std::array<float, 3> getMag(){return magnetometer_;}

        void setTOF(std::array<uint8_t, 2> value){tof_ = value;}
        std::array<uint8_t, 2> getTOF(){return tof_;}
        
        void setPressure(std::array<float, 16> value){pressure_ = value;}
        std::array<float, 16> getPressure(){return pressure_;}

        /**
         * @brief Convert a Mitch Data object into a string
         * @return std::string - A string representing a Mitch Data object
         */
        std::string toString()
        {
            std::ostringstream ss;

            ss << timestamp_ << "\t";
            for (int i = 0; i < 3; i++)
                ss << gyroscope_[i] << "\t";
            for (int i = 0; i < 3; i++)
                ss << accelerometer_[i] << "\t";
            for (int i = 0; i < 3; i++)
                ss << magnetometer_[i] << "\t";
            for (int i = 0; i < 2; i++)
                ss << tof_[i] << "\t";
            for (int i = 0; i < 15; i++)
                ss << pressure_[i] << "\t";
            ss << pressure_[15];
            
            std::string s(ss.str());
                
            return s;
        }
    };
}

#endif