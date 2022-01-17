#include <mitch_v2/MitchV2_StreamingData.h>

using namespace MitchV2;

MitchV2_Data MitchV2_StreamingData::dataTypePressure(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_PRESSURE> current_payload)
{
    MitchV2_Data current_data;
    std::array<uint8_t, 2> tmp;

    // Decode ID field 
    std::reverse_copy(current_payload.begin(), current_payload.begin() + 1, tmp.begin());
    unsigned short current_device_id = *reinterpret_cast<uint16_t*>(&tmp);
    current_data.setDeviceID(current_device_id);

    // Set timestamp
    current_data.setTimestamp(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());

    // Decode data - pressure
    std::array<float, 16> current_pressure;
    for (int i = 2; i < (uint8_t)MitchV2_HW::StreamPacketDimension::STREAM_PACKET_DIM_PRESSURE; i++)
         current_pressure[i - 2] = (int)(100 * (1 - (float)current_payload[i] / 255));
    current_data.setPressure(current_pressure);

    return current_data;
}

MitchV2_Data MitchV2_StreamingData::dataType6DOFandTOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_6DOF_TOF> current_payload, float gyr_res, float axl_res)
{
    MitchV2_Data current_data;
    std::array<uint8_t, 2> tmp;

    // Decode ID field 
    std::reverse_copy(current_payload.begin(), current_payload.begin() + 1, tmp.begin());
    unsigned short current_device_id = *reinterpret_cast<uint16_t*>(&tmp);
    current_data.setDeviceID(current_device_id);

    // Set timestamp
    current_data.setTimestamp(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());

    // Decode data

    // Gyroscope
    std::array<float, 3>  current_gyr; 
    std::copy_n(current_payload.begin()+2, 2, tmp.begin());
    current_gyr[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    std::copy_n(current_payload.begin()+4, 2, tmp.begin());
    current_gyr[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    std::copy_n(current_payload.begin()+6, 2, tmp.begin());
    current_gyr[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    current_data.setGyr(current_gyr);

    // Accelerometer
    std::array<float, 3>  current_axl;
    std::copy_n(current_payload.begin()+8, 2, tmp.begin());
    current_axl[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+10, 2, tmp.begin());
    current_axl[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+12, 2, tmp.begin());
    current_axl[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    current_data.setAxl(current_axl);

    // TOF
    std::array<uint8_t, 2>  current_tof;
    current_tof[0] = current_payload[14];
    current_tof[1] = current_payload[15];
    current_data.setTOF(current_tof);

    return current_data;
}

MitchV2_Data MitchV2_StreamingData::dataTypeTOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_TOF> current_payload)
{
    MitchV2_Data current_data;
    std::array<uint8_t, 2> tmp;

    // Decode ID field 
    std::reverse_copy(current_payload.begin(), current_payload.begin() + 1, tmp.begin());
    unsigned short current_device_id = *reinterpret_cast<uint16_t*>(&tmp);
    current_data.setDeviceID(current_device_id);

    // Set timestamp
    current_data.setTimestamp(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());

    // Decode data - TOF
    std::array<uint8_t, 2>  current_tof;
    current_tof[0] = current_payload[2];
    current_tof[1] = current_payload[3];
    current_data.setTOF(current_tof);

    return current_data;
}

MitchV2_Data MitchV2_StreamingData::dataType6DOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_6DOF> current_payload, float gyr_res, float axl_res)
{
    MitchV2_Data current_data;
    std::array<uint8_t, 2> tmp;

    // Decode ID field 
    std::reverse_copy(current_payload.begin(), current_payload.begin() + 1, tmp.begin());
    unsigned short current_device_id = *reinterpret_cast<uint16_t*>(&tmp);
    current_data.setDeviceID(current_device_id);

    // Set timestamp
    current_data.setTimestamp(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());

    // Decode data

    // Gyroscope
    std::array<float, 3>  current_gyr;
    std::copy_n(current_payload.begin()+2, 2, tmp.begin());
    current_gyr[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    std::copy_n(current_payload.begin()+4, 2, tmp.begin());
    current_gyr[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    std::copy_n(current_payload.begin()+6, 2, tmp.begin());
    current_gyr[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res);
    current_data.setGyr(current_gyr);

    // Accelerometer
    std::array<float, 3>  current_axl;
    std::copy_n(current_payload.begin()+8, 2, tmp.begin());
    current_axl[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+10, 2, tmp.begin());
    current_axl[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+12, 2, tmp.begin());
    current_axl[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    current_data.setAxl(current_axl);

    return current_data;
}

MitchV2_Data MitchV2_StreamingData::dataType9DOF(std::array<uint8_t, MitchV2_HW::STREAM_PACKET_DIM_9DOF> current_payload, float gyr_res, float axl_res, float mag_res)
{
    MitchV2_Data current_data;
    std::array<uint8_t, 2>  tmp;

    // Set timestamp
    current_data.setTimestamp(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());

    // Decode data
    
    // Gyroscope
    std::array<float, 3>  current_gyr;
    std::copy_n(current_payload.begin(), 2, tmp.begin());
    current_gyr[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res / 1000);
    std::copy_n(current_payload.begin()+2, 2, tmp.begin());
    current_gyr[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res / 1000);
    std::copy_n(current_payload.begin()+4, 2, tmp.begin());
    current_gyr[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* gyr_res / 1000);
    current_data.setGyr(current_gyr);

    // Accelerometer
    std::array<float, 3>  current_axl;
    std::copy_n(current_payload.begin()+6, 2, tmp.begin());
    current_axl[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+8, 2, tmp.begin());
    current_axl[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    std::copy_n(current_payload.begin()+10, 2, tmp.begin());
    current_axl[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* axl_res);
    current_data.setAxl(current_axl);

    // Magnetometer
    std::array<float, 3>  current_mag;  
    std::copy_n(current_payload.begin()+12, 2, tmp.begin());
    current_mag[0] = (float)(*reinterpret_cast<int16_t*>(&tmp)* mag_res);
    std::copy_n(current_payload.begin()+14, 2, tmp.begin());
    current_mag[1] = (float)(*reinterpret_cast<int16_t*>(&tmp)* mag_res);
    std::copy_n(current_payload.begin()+16, 2, tmp.begin());
    current_mag[2] = (float)(*reinterpret_cast<int16_t*>(&tmp)* mag_res);

    current_data.setMag(current_mag);

    return current_data;
}
