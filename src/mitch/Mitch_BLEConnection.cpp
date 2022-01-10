#include <mitch/Mitch_BLEConnection.h>

#include <iostream>

using namespace Mitch;
using namespace std::placeholders;

std::ofstream myfile;

float Mitch_BLEConnection::battery_charge_ = -1;
bool Mitch_BLEConnection::acquisition_started_ = false;
bool Mitch_BLEConnection::acquisition_stopped_ = false;
Mitch_HW::SystemState Mitch_BLEConnection::current_state_ = Mitch_HW::SystemState::SYS_NULL;

float Mitch_BLEConnection::current_gyr_resolution_ = 0; 
float Mitch_BLEConnection::current_axl_resolution_ = 0;
float Mitch_BLEConnection::current_mag_resolution_ = 0;

bool Mitch_BLEConnection::handleAcquisition(const uint8_t *data)
{

    bool handled = false;

    if (data[0] == 0x00 && data[1] == 5 && data[2] == (uint8_t)Mitch_HW::Command::CMD_STATE && data[3] == (uint8_t)_221e_Mitch::Mitch_HW::CMD_ACK_SUCCESS && data[4] == (uint8_t)Mitch_HW::SystemState::SYS_TX)
    {
        handled = true;
        acquisition_started_ = true;

        // Get gyroscope resolution from full scale
        switch (data[5])
        {
        case (uint8_t)Mitch_HW::Gyroscope_FS::GYR_FS_NULL:
            current_gyr_resolution_ = 0;
            break;
        case (uint8_t)Mitch_HW::Gyroscope_FS::GYR_FS_245_DPS:
            current_gyr_resolution_ = Mitch_HW::GYR_RESOLUTION_245dps;
            break;
        case (uint8_t)Mitch_HW::Gyroscope_FS::GYR_FS_500_DPS:
            current_gyr_resolution_ = Mitch_HW::GYR_RESOLUTION_500dps;
            break;
        case (uint8_t)Mitch_HW::Gyroscope_FS::GYR_FS_1000_DPS:
            current_gyr_resolution_ = Mitch_HW::GYR_RESOLUTION_1000dps;
            break;
        case (uint8_t)Mitch_HW::Gyroscope_FS::GYR_FS_2000_DPS:
            current_gyr_resolution_ = Mitch_HW::GYR_RESOLUTION_2000dps;
            break;
        default:
            current_gyr_resolution_ = 0;
            break;
        }

        // Get accelerometer resolution from full scale
        switch (data[6])
        {
        case (uint8_t)Mitch_HW::Accelerometer_FS::AXL_FS_NULL:
            current_axl_resolution_ = 0;
            break;
        case (uint8_t)Mitch_HW::Accelerometer_FS::AXL_FS_2_g:
            current_axl_resolution_ = Mitch_HW::AXL_RESOLUTION_2g;
            break;
        case (uint8_t)Mitch_HW::Accelerometer_FS::AXL_FS_4_g:
            current_axl_resolution_ = Mitch_HW::AXL_RESOLUTION_4g;
            break;
        case (uint8_t)Mitch_HW::Accelerometer_FS::AXL_FS_8_g:
            current_axl_resolution_ = Mitch_HW::AXL_RESOLUTION_8g;
            break;
        case (uint8_t)Mitch_HW::Accelerometer_FS::AXL_FS_16_g:
            current_axl_resolution_ = Mitch_HW::AXL_RESOLUTION_16g;
            break;
        default:
            current_axl_resolution_ = 0;
            break;
        }

        // Mag resolution has been hard-coded on fw side
        current_mag_resolution_ = Mitch_HW::MAG_RESOLUTION;

    }

    return handled;
}

bool Mitch_BLEConnection::handleState(const uint8_t *data)
{

    bool handled = false;

    if (data[0] == Mitch_HW::CMD_ACK && data[2] == (uint8_t)(Mitch_HW::CMD_STATE + 0x80) && data[3] == Mitch_HW::CMD_ACK_SUCCESS)
    {
        handled = true;
        current_state_ = Mitch_HW::SystemState(data[ACK_OFFSET]);
    }

    return handled;
}

bool Mitch_BLEConnection::handleBattery(const uint8_t *data)
{

    bool handled = false;
    if (data[0] == Mitch_HW::CMD_ACK && data[2] == (uint8_t)(Mitch_HW::CMD_BATTERY_CHARGE + 0x80) && data[3] == Mitch_HW::CMD_ACK_SUCCESS)
    {
        handled = true;
        battery_charge_ = data[ACK_OFFSET];
    }

    return handled;
}

 bool Mitch_BLEConnection::handleStreamingData(const uint8_t *data)
{
    bool handled = false;

    Mitch_Data current_data;
    Json::Value event;

    Json::Value timestamp(Json::intValue);

    Json::Value gyr_vec(Json::arrayValue);
    Json::Value axl_vec(Json::arrayValue);
    Json::Value tof_vec(Json::arrayValue);
    Json::Value mag_vec(Json::arrayValue);
    Json::Value press_vec(Json::arrayValue);

    std::array<float, 3> gyroscope;
    std::array<float, 3> accelerometer;
    std::array<float, 3> magnetometer;
    std::array<uint8_t, 2> distance;
    std::array<float, 16> pressure;

    switch (data[0])
    {
    case (uint8_t)Mitch_HW::StreamMode::STREAM_MODE_PRESSURE:

        // Get
        std::array<uint8_t, Mitch_HW::STREAM_PACKET_DIM_PRESSURE> pressure_payload;
        std::copy_n(&data[2], Mitch_HW::STREAM_PACKET_DIM_PRESSURE, std::begin(pressure_payload));

        // Decode
        current_data = Mitch_StreamingData::dataTypePressure(pressure_payload);

         // JSON Publish
        pressure = current_data.getPressure();

        for(unsigned int i = 0; i < pressure.size(); i++ ){
            press_vec.append(Json::Value(pressure[i]));
        }

        event["items"]["Timestamp"] = Json::Value((double)current_data.getTimestamp());
        event["items"]["Pressure"] = press_vec;

        myfile << event;

        handled = true;

        break;

    case (uint8_t)Mitch_HW::StreamMode::STREAM_MODE_6DOF_TOF:

        // Get
        std::array<uint8_t, Mitch_HW::STREAM_PACKET_DIM_6DOF_TOF> six_dof_tof_payload;
        std::copy_n(&data[2], Mitch_HW::STREAM_PACKET_DIM_6DOF_TOF, std::begin(six_dof_tof_payload));

        // Decode
        current_data = Mitch_StreamingData::dataType6DOFandTOF(six_dof_tof_payload, current_gyr_resolution_, current_axl_resolution_);

        // JSON Publish
        gyroscope = current_data.getGyr();
        accelerometer = current_data.getAxl();
        distance = current_data.getTOF();

        for(unsigned int i = 0; i < gyroscope.size(); i++ ){
            gyr_vec.append(Json::Value(gyroscope[i]));
        }

         for(unsigned int i = 0; i < accelerometer.size(); i++ ){
            axl_vec.append(Json::Value(accelerometer[i]));
        }

        for(unsigned int i = 0; i < distance.size(); i++ ){
            tof_vec.append(Json::Value(distance[i]));
        }
        
        event["items"]["Timestamp"] = Json::Value((double)current_data.getTimestamp());
        event["items"]["Gyr"] = gyr_vec;
        event["items"]["Axl"] = axl_vec;
        event["items"]["TOF"] = tof_vec;

        myfile << event;

        handled = true;

        break;
    case (uint8_t)Mitch_HW::StreamMode::STREAM_MODE_TOF:

        // Get
        std::array<uint8_t, Mitch_HW::STREAM_PACKET_DIM_TOF> tof_payload;
        std::copy_n(&data[2], Mitch_HW::STREAM_PACKET_DIM_TOF, std::begin(tof_payload));

        // Decode
        current_data = Mitch_StreamingData::dataTypeTOF(tof_payload);

        // JSON Publish
        distance = current_data.getTOF();

        for(unsigned int i = 0; i < distance.size(); i++ ){
            tof_vec.append(Json::Value(distance[i]));
        }

        event["items"]["Timestamp"] = Json::Value((double)current_data.getTimestamp());
        event["items"]["TOF"] = tof_vec;

        myfile << event;

        handled = true;

        break;
    case (uint8_t)Mitch_HW::StreamMode::STREAM_MODE_6DOF:

        // Get 
        std::array<uint8_t, Mitch_HW::STREAM_PACKET_DIM_6DOF> six_dof_payload;
        std::copy_n(&data[2], Mitch_HW::STREAM_PACKET_DIM_6DOF, std::begin(six_dof_payload));

        // Decode
        current_data = Mitch_StreamingData::dataType6DOF(six_dof_payload, current_gyr_resolution_, current_axl_resolution_);

        // JSON Publish
        gyroscope = current_data.getGyr();
        accelerometer = current_data.getAxl();

        for(unsigned int i = 0; i < gyroscope.size(); i++ ){
            gyr_vec.append(Json::Value(gyroscope[i]));
        }

         for(unsigned int i = 0; i < accelerometer.size(); i++ ){
            axl_vec.append(Json::Value(accelerometer[i]));
        }  
        
        event["items"]["Timestamp"] = Json::Value((double)current_data.getTimestamp());
        event["items"]["Gyr"] = gyr_vec;
        event["items"]["Axl"] = axl_vec;
        
        myfile << event;

        handled = true;

        break;
    case (uint8_t)Mitch_HW::StreamMode::STREAM_MODE_9DOF:

        // Get
        std::array<uint8_t, Mitch_HW::STREAM_PACKET_DIM_9DOF> nine_dof_payload;
        std::copy_n(&data[2], Mitch_HW::STREAM_PACKET_DIM_9DOF, std::begin(nine_dof_payload));

        // Decode
        current_data = Mitch_StreamingData::dataType9DOF(nine_dof_payload, current_gyr_resolution_, current_axl_resolution_, current_mag_resolution_);

        // JSON Publish
        gyroscope = current_data.getGyr();
        accelerometer = current_data.getAxl();
        magnetometer = current_data.getMag();

        for(unsigned int i = 0; i < gyroscope.size(); i++ ){
            gyr_vec.append(Json::Value(gyroscope[i]));
        }

         for(unsigned int i = 0; i < accelerometer.size(); i++ ){
            axl_vec.append(Json::Value(accelerometer[i]));
        }

        for(unsigned int i = 0; i < magnetometer.size(); i++ ){
            mag_vec.append(Json::Value(magnetometer[i]));
        }

        event["items"]["Timestamp"] = Json::Value((double)current_data.getTimestamp());
        event["items"]["Gyr"] = gyr_vec;
        event["items"]["Axl"] = axl_vec;
        event["items"]["Mag"] = mag_vec;
        
        myfile << event;

        handled = true;

        break;
    default:
        break;
    }

    return handled;
}

void Mitch_BLEConnection::notificationHandler(const uuid_t *uuid, const uint8_t *data, size_t data_length, void *user_data)
{
    // for (size_t i = 0; i < data_length; i++)
    // {
    //     printf("%02x ", data[i]);
    // }
    // printf("\n");

    handleBattery(data);
    handleState(data);
    handleAcquisition(data);
    handleStreamingData(data);
}

Mitch_BLEConnection::Mitch_BLEConnection(const char *mac_address, const char *cmd_uuid, const char *data_uuid, const char * json_file_path) : 
g_mutex(PTHREAD_MUTEX_INITIALIZER),
mac_address_(mac_address), cmd_uuid_(cmd_uuid), data_uuid_(data_uuid),
connection_status_(false), transmission_status_(false), uuid_status_(false), notification_status_(false)
{
    myfile.open(json_file_path);

    if ((gattlib_string_to_uuid(cmd_uuid_, std::strlen(cmd_uuid_) + 1, &g_cmd_uuid_) >= 0) &&
        (gattlib_string_to_uuid(data_uuid_, std::strlen(data_uuid_) + 1, &g_data_uuid_) >= 0))
    {
        uuid_status_ = true;
    }
    else
        puts("Failed to parse UUID");
}

Mitch_BLEConnection::~Mitch_BLEConnection()
{
    connection_status_ = false;
    transmission_status_ = false;
    uuid_status_ = false;
    notification_status_ = false;
    mac_address_ = "";
}

void Mitch_BLEConnection::BLEDiscoverdDevice(void *adapter, const char *addr, const char *name, void *user_data)
{
    if (name)
        printf("Discovered %s - '%s'\n", addr, name);
    else
        printf("Discovered %s\n", addr);
}

bool Mitch_BLEConnection::BLEScan()
{
    void *adapter;
    bool scanned = true;

    LIST_INIT(&g_ble_connections);

    cmd_ret_ = gattlib_adapter_open(NULL, &adapter);

    if (cmd_ret_)
    {
        puts("Failed to open adapter.");
        scanned = false;
    }

    pthread_mutex_lock(&g_mutex);
    cmd_ret_ = gattlib_adapter_scan_enable(adapter, BLEDiscoverdDevice, BLE_SCAN_TIMEOUT, NULL /* user_data */);
    if (cmd_ret_)
    {
        puts("Failed to scan.");
        scanned = false;
    }

    gattlib_adapter_scan_disable(adapter);

    puts("Scan completed");
    pthread_mutex_unlock(&g_mutex);

    return scanned;
}

bool Mitch_BLEConnection::connect()
{
    bool connected = true;

    connection = gattlib_connect(NULL, mac_address_, GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);

    if (connection == NULL)
    {
        puts("Device connection failed.");
        connected = false;
    }
    else
    {
        connection_status_ = true;
        puts("Device connected.");

        if (checkUUIDStatus())
        {

            gattlib_register_notification(connection, notificationHandler, NULL);
            cmd_ret_ = gattlib_notification_start(connection, &g_cmd_uuid_);
            data_ret_ = gattlib_notification_start(connection, &g_data_uuid_);

            if (cmd_ret_ == GATTLIB_SUCCESS && data_ret_ == GATTLIB_SUCCESS)
                notification_status_ = true;
        }
    }

    return connected;
}

void Mitch_BLEConnection::disconnect()
{
    if (checkConnectionStatus())
    {
        // Stop transmission - transmission status = false

        gattlib_disconnect(connection);
        connection_status_ = false;
        transmission_status_ = false;
        uuid_status_ = false;
        notification_status_ = false;

        myfile.close();
    }
}

bool Mitch_BLEConnection::checkConnectionStatus()
{
    return connection_status_;
}

bool Mitch_BLEConnection::checkTransmissionStatus()
{
    return transmission_status_;
}

bool Mitch_BLEConnection::checkUUIDStatus()
{
    return uuid_status_;
}

bool Mitch_BLEConnection::checkNotificationStatus()
{
    return notification_status_;
}

bool Mitch_BLEConnection::sendStartAcquisitionCommand(Mitch_HW::StreamMode mode, Mitch_HW::StreamFrequency frequency)
{

    bool written = false;

    if (!checkConnectionStatus() || !checkUUIDStatus() || !checkNotificationStatus())
        return written;

    uint8_t *command_buffer = (uint8_t *)malloc(sizeof(uint8_t) * Mitch_HW::COMM_MESSAGE_LEN);
    int resp_len = 3;

    command_buffer[0] = (uint8_t)(Mitch_HW::Command::CMD_STATE); 
    command_buffer[1] = (uint8_t)resp_len; 
    command_buffer[2] = (uint8_t)Mitch_HW::SystemState::SYS_TX; 
    command_buffer[3] = (uint8_t)mode; 
    command_buffer[4] = (uint8_t)frequency; 

    cmd_ret_ = gattlib_write_char_by_uuid(connection, &g_cmd_uuid_, command_buffer, Mitch_HW::COMM_MESSAGE_LEN);

    if (cmd_ret_ != GATTLIB_SUCCESS)
    {
        printf("Error while reading GATT Characteristic with UUID %s .", cmd_uuid_);
        return written;
    }

    written = true;
    return written;
}

bool Mitch_BLEConnection::sendStopAcquisitionCommand()
{

    bool written = false;

    if (!checkConnectionStatus() || !checkUUIDStatus() || !checkNotificationStatus())
        return written;

    uint8_t *command_buffer = (uint8_t *)malloc(sizeof(uint8_t) * Mitch_HW::COMM_MESSAGE_LEN);
    int resp_len = 3;

    command_buffer[0] = (uint8_t)(Mitch_HW::Command::CMD_STATE);
    command_buffer[1] = (uint8_t)resp_len;
    command_buffer[2] = (uint8_t)Mitch_HW::SystemState::SYS_TX;

    cmd_ret_ = gattlib_write_char_by_uuid(connection, &g_cmd_uuid_, command_buffer, Mitch_HW::COMM_MESSAGE_LEN);

    if (cmd_ret_ != GATTLIB_SUCCESS)
    {
        printf("Error while reading GATT Characteristic with UUID %s .", cmd_uuid_);
        return written;
    }

    written = true;

    myfile.close();

    return written;

}

bool Mitch_BLEConnection::sendBatteryChargeCommand()
{
    bool written = false;

    if (!checkConnectionStatus() || !checkUUIDStatus() || !checkNotificationStatus())
        return written;

    long int value_data = (uint8_t)Mitch_HW::CMD_BATTERY_CHARGE + 0x80;

    cmd_ret_ = gattlib_write_char_by_uuid(connection, &g_cmd_uuid_, &value_data, sizeof(value_data));

    if (cmd_ret_ != GATTLIB_SUCCESS)
    {
        printf("Error while reading GATT Characteristic with UUID %s .", cmd_uuid_);
        return written;
    }

    written = true;
    return written;
}

bool Mitch_BLEConnection::sendGetStateCommand()
{

    bool written = false;

    if (!checkConnectionStatus() || !checkUUIDStatus() || !checkNotificationStatus())
        return written;

    long int value_data = (uint8_t)Mitch_HW::Command::CMD_STATE + 0x80;

    cmd_ret_ = gattlib_write_char_by_uuid(connection, &g_cmd_uuid_, &value_data, sizeof(value_data));

    if (cmd_ret_ != GATTLIB_SUCCESS)
    {
        printf("Error while reading GATT Characteristic with UUID %s .", cmd_uuid_);
        return written;
    }

    written = true;
    return written;

}

Mitch_HW::SystemState Mitch_BLEConnection::getState()
{

    return current_state_;
}

bool Mitch_BLEConnection::acquisitionStarted()
{

    return acquisition_started_;
}

float Mitch_BLEConnection::getBatteryCharge()
{
    return battery_charge_;
}
