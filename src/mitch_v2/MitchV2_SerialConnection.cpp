#include <mitch_v2/MitchV2_SerialConnection.h>
#include <sstream>

using namespace MitchV2;

MitchV2_SerialConnection::MitchV2_SerialConnection(const std::string& port,
	uint32_t baudrate,
	Timeout timeout,
	bytesize_t bytesize,
	parity_t parity,
	stopbits_t stopbits,
	flowcontrol_t flowcontrol)
	: serial_connection_(port, baudrate, timeout, bytesize, parity, stopbits, flowcontrol),
	connection_status_(false), transmission_status_(false),
	serial_port_(port)
{
	serial_connection_.SetTimeout(Timeout::max(), 1000, 0, 1000, 0);
	if (serial_connection_.IsOpen() == true)
	{
		serial_connection_.Flush();
		connection_status_ = true;

	}
}

MitchV2_SerialConnection::~MitchV2_SerialConnection()
{
	connection_status_ = false;
	transmission_status_ = false;
	serial_port_ = "";
}

bool MitchV2_SerialConnection::checkConnectionStatus()
{
	return connection_status_;
}

bool MitchV2_SerialConnection::checkTransmissionStatus()
{
	return transmission_status_;
}

bool MitchV2_SerialConnection::startStreaming(MitchV2_HW::LogMode mode, MitchV2_HW::LogFrequency frequency) {
	return startLog(mode, frequency);
}

bool MitchV2_SerialConnection::stopStreaming() {
	return stopLog();
}

bool MitchV2_SerialConnection::startLog(MitchV2_HW::LogMode mode, MitchV2_HW::LogFrequency frequency) {

	if(!checkConnectionStatus())
		return false;

	serial_connection_.Flush();

	bool result = false;

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 3;
	
	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::Command::CMD_STATE);
	command_buffer[3] = (uint8_t)resp_len;
	command_buffer[4] = (uint8_t)MitchV2_HW::SystemState::SYS_LOG;
	command_buffer[5] = (uint8_t)mode;
	command_buffer[6] = (uint8_t)frequency;
	command_buffer[7] = (uint8_t)'!';
	command_buffer[8] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 9);
	
	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 10));

	if (serial_connection_.Available() > 0)
	{
		transmission_status_ = true;
		uint8_t acquisition_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(acquisition_buffer,MitchV2_HW::COMM_MESSAGE_LEN);
		resp_len = acquisition_buffer[3];
		if (acquisition_buffer[0] == '?' && acquisition_buffer[1] == '!' && acquisition_buffer[resp_len + ACK_OFFSET] == '!' && acquisition_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (acquisition_buffer[2] == 0x00 && acquisition_buffer[4] == (uint8_t)(MitchV2_HW::Command::CMD_STATE) && acquisition_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS)
				result = true;
	}
	return result;
}

bool MitchV2_SerialConnection::stopLog() {
	
	if(!checkConnectionStatus() && !checkTransmissionStatus())
		return false;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 1;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::Command::CMD_STATE);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)MitchV2_HW::SystemState::SYS_IDLE;
	command_buffer[5] = (uint8_t)'!';
	command_buffer[6] = (uint8_t)'?';
	serial_connection_.Write(command_buffer, 7);
	
	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0)
	{
		transmission_status_ = false;
		uint8_t acquisition_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(acquisition_buffer, MitchV2_HW::COMM_MESSAGE_LEN);
		resp_len = acquisition_buffer[3];
		if (acquisition_buffer[0] == '?' && acquisition_buffer[1] == '!' && acquisition_buffer[resp_len + ACK_OFFSET] == '!' && acquisition_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (acquisition_buffer[2] == 0x00 && acquisition_buffer[4] == (uint8_t)(MitchV2_HW::Command::CMD_STATE) && acquisition_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS)
				return true;
	}
	return false;
}

void MitchV2_SerialConnection::stopTransmission()
{
	if(checkConnectionStatus() && checkTransmissionStatus()){
		serial_connection_.Flush();

		uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
		int resp_len = 1;

		command_buffer[0] = (uint8_t)MitchV2_HW::Command::CMD_STATE;
		command_buffer[1] = (uint8_t)(resp_len);							
		command_buffer[2] = (uint8_t)MitchV2_HW::SystemState::SYS_IDLE;

		serial_connection_.Write(command_buffer, 3);

		transmission_status_ = false;
	}
	
}

void MitchV2_SerialConnection::shutdown()
{
	if(checkConnectionStatus()){
		serial_connection_.Flush();
		uint8_t* command_buffer = cmdState(MitchV2_HW::SystemState::SYS_STANDBY, {});
		serial_connection_.Write(command_buffer, sizeof(command_buffer));
		disconnect();
	}	
}

void MitchV2_SerialConnection::disconnect()
{
	if(checkConnectionStatus()){
		serial_connection_.Flush();
		stopTransmission();
		serial_connection_.Close();
		connection_status_ = false;
	}
}

uint8_t* MitchV2_SerialConnection::cmdState(bool enableRead, MitchV2_HW::SystemState state, uint8_t* stateParams) {

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';

	if (enableRead) {
		command_buffer[2] = (uint8_t)((uint8_t)MitchV2_HW::Command::CMD_STATE + 0x80);
		command_buffer[3] = (uint8_t)(resp_len);
		command_buffer[4] = (uint8_t)'!';
		command_buffer[5] = (uint8_t)'?';
	}
	else {
		command_buffer[2] = (uint8_t)MitchV2_HW::Command::CMD_STATE;
		if (state != MitchV2_HW::SystemState::SYS_NULL)
		{
			resp_len = 1;
			command_buffer[3] = (uint8_t)(resp_len);

			switch (state)
			{
			case MitchV2_HW::SystemState::SYS_BOOT_STARTUP:
			case MitchV2_HW::SystemState::SYS_BOOT_IDLE:
			case MitchV2_HW::SystemState::SYS_BOOT_WRITE:
			case MitchV2_HW::SystemState::SYS_ERROR:
				break;
			case MitchV2_HW::SystemState::SYS_STARTUP:
				command_buffer[4] = (uint8_t)state;
				break;
			case MitchV2_HW::SystemState::SYS_IDLE:
				command_buffer[4] = (uint8_t)state;
				break;
			case MitchV2_HW::SystemState::SYS_STANDBY:
				command_buffer[4] = (uint8_t)state;
				break;
			case MitchV2_HW::SystemState::SYS_LOG:
				break;
			case MitchV2_HW::SystemState::SYS_READOUT:
				break;
			default:
				break;
			}
			command_buffer[5] = (uint8_t)'!';
			command_buffer[6] = (uint8_t)'?';
		}
	}

	return command_buffer;
}

MitchV2_HW::SystemState MitchV2_SerialConnection::getState() {
	
	if(!checkConnectionStatus())
		return MitchV2_HW::SystemState::SYS_ERROR;
	
	serial_connection_.Flush();

	uint8_t* command_buffer = cmdState();
	serial_connection_.Write(command_buffer, 6);

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	MitchV2_HW::SystemState current_state = MitchV2_HW::SystemState::SYS_NULL;

	if (serial_connection_.Available() > 0) {


		uint8_t state_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(state_buffer, MitchV2_HW::COMM_MESSAGE_LEN);
		int resp_len = state_buffer[3];

		if (state_buffer[0] == '?' && state_buffer[1] == '!' && state_buffer[resp_len + ACK_OFFSET] == '!' && state_buffer[resp_len + ACK_OFFSET + 1] == '?') 
			if (state_buffer[2] == MitchV2_HW::CMD_ACK && state_buffer[4] == (uint8_t)(MitchV2_HW::Command::CMD_STATE + 0x80) && state_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS)
				current_state = MitchV2_HW::SystemState(state_buffer[resp_len + ACK_OFFSET -1]);
	
	}

	return current_state;
}

float MitchV2_SerialConnection::getBatteryCharge()
{
	if(!checkConnectionStatus())
		return -1;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_BATTERY_CHARGE + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);
	
	float charge = 0;

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t battery_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(battery_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = battery_buffer[3];

		if (battery_buffer[0] == '?' && battery_buffer[1] == '!' && battery_buffer[resp_len + ACK_OFFSET] == '!' && battery_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (battery_buffer[2] == MitchV2_HW::CMD_ACK && battery_buffer[4] == (uint8_t)(MitchV2_HW::CMD_BATTERY_CHARGE + 0x80) && battery_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS)
				charge = battery_buffer[ACK_OFFSET + VALUE_OFFSET];
		
	}
	
	return charge;
}

float MitchV2_SerialConnection::getBatteryVoltage() {

	if (!checkConnectionStatus())
		return -1;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_BATTERY_VOLTAGE + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	float voltage = 0;

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t battery_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(battery_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = battery_buffer[3];

		if (battery_buffer[0] == '?' && battery_buffer[1] == '!' && battery_buffer[resp_len + ACK_OFFSET] == '!' && battery_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (battery_buffer[2] == MitchV2_HW::CMD_ACK && battery_buffer[4] == (uint8_t)(MitchV2_HW::CMD_BATTERY_VOLTAGE + 0x80) && battery_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {
				uint8_t voltage_array[2] = { battery_buffer[ACK_OFFSET + VALUE_OFFSET +1], battery_buffer[ACK_OFFSET + VALUE_OFFSET]};
				voltage = *reinterpret_cast<uint16_t*>(voltage_array);
			}

	}

	return voltage;
}

string MitchV2_SerialConnection::getFirmwareVersion() {

	if (!checkConnectionStatus())
		return NULL;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_FW_VERSION + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	string fw_version = "";

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t firmware_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(firmware_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = firmware_buffer[3];

		if (firmware_buffer[0] == '?' && firmware_buffer[1] == '!' && firmware_buffer[resp_len + ACK_OFFSET] == '!' && firmware_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (firmware_buffer[2] == MitchV2_HW::CMD_ACK && firmware_buffer[4] == (uint8_t)(MitchV2_HW::CMD_FW_VERSION + 0x80) && firmware_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {

				std::stringstream list;

				for (int i = 0; i < resp_len - VALUE_OFFSET; ++i)
					list << (char)firmware_buffer[ACK_OFFSET + VALUE_OFFSET + i];
				
				fw_version = list.str();

			}
	}

	return fw_version;
}

string MitchV2_SerialConnection::getBLEName() {

	if (!checkConnectionStatus())
		return NULL;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_BLE_NAME + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	string ble_name = "";

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t ble_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(ble_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = ble_buffer[3];

		if (ble_buffer[0] == '?' && ble_buffer[1] == '!' && ble_buffer[resp_len + ACK_OFFSET] == '!' && ble_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (ble_buffer[2] == MitchV2_HW::CMD_ACK && ble_buffer[4] == (uint8_t)(MitchV2_HW::CMD_BLE_NAME + 0x80) && ble_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {

				std::stringstream list;

				for (int i = 0; i < resp_len - VALUE_OFFSET; ++i)
					list << (char)ble_buffer[ACK_OFFSET + VALUE_OFFSET + i];

				ble_name = list.str();
			}
	}

	return ble_name;

}

uint16_t MitchV2_SerialConnection::getDeviceId() {

	if (!checkConnectionStatus())
		return 0;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_DEVICE_ID + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	uint16_t device_id = 0;

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t device_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(device_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = device_buffer[3];

		if (device_buffer[0] == '?' && device_buffer[1] == '!' && device_buffer[resp_len + ACK_OFFSET] == '!' && device_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (device_buffer[2] == MitchV2_HW::CMD_ACK && device_buffer[4] == (uint8_t)(MitchV2_HW::CMD_DEVICE_ID + 0x80) && device_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {

				int temp_size = resp_len - VALUE_OFFSET;
				uint8_t* temp = (uint8_t*)malloc(sizeof(uint8_t) * temp_size);

				for (int i = 0; i < resp_len - VALUE_OFFSET; ++i)
					temp[i] = device_buffer[ACK_OFFSET + VALUE_OFFSET + temp_size - 1 - i];

				device_id = *reinterpret_cast<uint16_t*>(temp);
			}
	}

	return device_id;
}

ptime MitchV2_SerialConnection::getTime() {

	ptime current_time = time_from_string("1970-01-01 00:00:00.000");

	if (!checkConnectionStatus())
		return current_time;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_TIME + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 1000));

	if (serial_connection_.Available() > 0) {

		uint8_t device_buffer[MitchV2_HW::COMM_MESSAGE_LEN];

		serial_connection_.Read(device_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = device_buffer[3];

		if (device_buffer[0] == '?' && device_buffer[1] == '!' && device_buffer[resp_len + ACK_OFFSET] == '!' && device_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (device_buffer[2] == MitchV2_HW::CMD_ACK && device_buffer[4] == (uint8_t)(MitchV2_HW::CMD_TIME + 0x80) && device_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {

				int temp_size = resp_len - VALUE_OFFSET;
				uint8_t* temp = (uint8_t*)malloc(sizeof(uint8_t) * temp_size);

				for (int i = 0; i < resp_len - VALUE_OFFSET; ++i)
					temp[i] = device_buffer[ACK_OFFSET + VALUE_OFFSET + i];

				current_time += seconds(*reinterpret_cast<int*>(temp));

			}
	}

	return current_time;
}

uint8_t* MitchV2_SerialConnection::getClockOffset() {

	if (!checkConnectionStatus())
		return NULL;

	serial_connection_.Flush();

	uint8_t* command_buffer = (uint8_t*)malloc(sizeof(uint8_t) * MitchV2_HW::COMM_MESSAGE_LEN);
	int resp_len = 0;

	command_buffer[0] = (uint8_t)'?';
	command_buffer[1] = (uint8_t)'!';
	command_buffer[2] = (uint8_t)(MitchV2_HW::CMD_CLK_OFFSET + 0x80);
	command_buffer[3] = (uint8_t)(resp_len);
	command_buffer[4] = (uint8_t)'!';
	command_buffer[5] = (uint8_t)'?';

	serial_connection_.Write(command_buffer, 6);

	std::time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	uint8_t* clock_offset;

	if (serial_connection_.Available() > 0) {
		uint8_t offset_buffer[MitchV2_HW::COMM_MESSAGE_LEN];
		serial_connection_.Read(offset_buffer, MitchV2_HW::COMM_MESSAGE_LEN);

		resp_len = offset_buffer[3];

		if (offset_buffer[0] == '?' && offset_buffer[1] == '!' && offset_buffer[resp_len + ACK_OFFSET] == '!' && offset_buffer[resp_len + ACK_OFFSET + 1] == '?')
			if (offset_buffer[2] == MitchV2_HW::CMD_ACK && offset_buffer[4] == (uint8_t)(MitchV2_HW::CMD_CLK_OFFSET + 0x80) && offset_buffer[5] == MitchV2_HW::CMD_ACK_SUCCESS) {

				int temp_size = resp_len - VALUE_OFFSET;
				uint8_t* clock_offset = (uint8_t*)malloc(sizeof(uint8_t) * temp_size);

				for (int i = 0; i < resp_len - VALUE_OFFSET; ++i)
					clock_offset[i] = offset_buffer[ACK_OFFSET + VALUE_OFFSET + temp_size - 1 - i];

			}
	}

	return clock_offset;
}



