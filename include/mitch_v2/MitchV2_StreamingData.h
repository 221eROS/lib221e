#ifndef MITCH_V2_STREAMING_DATA_H
#define MITCH_V2_STREAMING_DATA_H

#include <mitch_v2/MitchV2_Data.h>
#include <mitch_v2/MitchV2_HW.h>

#include <algorithm>
#include <chrono>

using namespace std::chrono;

namespace MitchV2
{

    /**
     * @brief Collection of Mitch Data functions specific to create the Streaming Data
     */
    class MitchV2_StreamingData
    {
    
    public:

    /**
     * @brief Create a Pressure Data object from an input payload
     * @param current_payload A uint8_t array of size Mitch_HW::STREAM_PACKET_DIM_PRESSURE
     * @return Mitch_Data - The object representing a pressure value
     */
    static MitchV2_Data dataTypePressure(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_PRESSURE> current_payload);

    /**
     * @brief Create a 6 Degrees of Freedom (DOFs) and Time of Flight (TOF) Data object from an input payload
     * @param current_payload A uint8_t array of size Mitch_HW::STREAM_PACKET_DIM_6DOF_TOF
     * @param gyr_res The current gyroscope resolution
     * @param axl_res The current accelerometer resolution
     * @return Mitch_Data - The object collecting the 6-DOFs and TOF values
     */
    static MitchV2_Data dataType6DOFandTOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_6DOF_TOF> current_payload, float gyr_res, float axl_res);

    /**
     * @brief Create a TOF Data object from an input payload
     * @param current_payload A uint8_t array of size Mitch_HW::STREAM_PACKET_DIM_TOF
     * @return Mitch_Data - The object representing a TOF value
     */
    static MitchV2_Data dataTypeTOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_TOF> current_payload);

    /**
     * @brief Create a 6-DOFs Data object from an input payload
     * @param current_payload A uint8_t array of size Mitch_HW::STREAM_PACKET_DIM_6DOF
     * @param gyr_res The current gyroscope resolution
     * @param axl_res The current accelerometer resolution
     * @return Mitch_Data - The object representing a 6-DOFs value
     */
    static MitchV2_Data dataType6DOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_6DOF> current_payload, float gyr_res, float axl_res);

    /**
     * @brief Create a 9-DOFs Data object from an input payload
     * @param current_payload A uint8_t array of size Mitch_HW::STREAM_PACKET_DIM_9DOF
     * @param gyr_res The current gyroscope resolution
     * @param axl_res The current accelerometer resolution
     * @param mag_res The current magnetometer resolution
     * @return Mitch_Data - The object representing a 9-DOFs value
     */
    static MitchV2_Data dataType9DOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_9DOF> current_payload, float gyr_res, float axl_res, float mag_res);
  
    };
}

#endif