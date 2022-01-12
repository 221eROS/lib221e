#include <muse/Muse_SerialConnection.h>

using namespace Muse;

Muse_SerialConnection::Muse_SerialConnection(const string& port,
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

Muse_SerialConnection::~Muse_SerialConnection()
{
	connection_status_ = false;
	transmission_status_ = false;
	serial_port_ = "";
}

bool Muse_SerialConnection::checkConnectionStatus()
{
	return connection_status_;
}

bool Muse_SerialConnection::checkTransmissionStatus()
{
	return transmission_status_;
}

void Muse_SerialConnection::stopTransmission()
{
	if (checkConnectionStatus() && checkTransmissionStatus()) {
		serial_connection_.Flush();
		serial_connection_.Write(Muse_HW::StopTransmission);
		Sleep(1000);
		while (serial_connection_.Available() > 0)
			serial_connection_.Read();
		transmission_status_ = false;
	}

}

void Muse_SerialConnection::shutdown()
{
	if (checkConnectionStatus()) {
		serial_connection_.Write(Muse_HW::Shutdown);
		disconnect();
	}
}

void Muse_SerialConnection::disconnect()
{
	if (checkConnectionStatus()) {
		serial_connection_.Flush();
		stopTransmission();
		serial_connection_.Close();
		connection_status_ = false;
	}
}

float Muse_SerialConnection::getBatteryCharge(){

	float charge = -1;

	if (!checkConnectionStatus())
		return charge;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetBatteryCharge);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t battery_charge_buffer[Muse_HW::BATTERY_BUFFER_SIZE];
		int offset = 2;
		serial_connection_.Read(battery_charge_buffer, sizeof(battery_charge_buffer));

		if (battery_charge_buffer[0] == 'B' && battery_charge_buffer[1] == 'Q')
			charge = (uint16_t)battery_charge_buffer[offset];

	}

	return charge;

}

float Muse_SerialConnection::getBatteryVoltage() {

	float voltage = -1;

	if (!checkConnectionStatus())
		return voltage;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetBatteryVoltage);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t battery_voltage_buffer[Muse_HW::BATTERY_BUFFER_SIZE];
		int offset = 2;
		serial_connection_.Read(battery_voltage_buffer, sizeof(battery_voltage_buffer));

		if (battery_voltage_buffer[0] == 'B' && battery_voltage_buffer[1] == 'V') {
			uint8_t voltage_array[2] = {battery_voltage_buffer[offset], battery_voltage_buffer[offset + 1]};
			voltage = *reinterpret_cast<uint16_t*>(voltage_array);
		}

	}

	return voltage;
}

uint16_t Muse_SerialConnection::getGyroscopeFullScale() {

	uint16_t gyroscope = 0;

	if (!checkConnectionStatus())
		return gyroscope;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P') {

			switch (configuration_buffer[2])
			{
			case 0x00:	// 0
				gyroscope = Muse_HW::GYROSCOPE_FULL_SCALE_500DPS;
				break;
			case 0x08:	// 8
				gyroscope = Muse_HW::GYROSCOPE_FULL_SCALE_1000DPS;
				break;
			case 0x10:	// 16
				gyroscope = Muse_HW::GYROSCOPE_FULL_SCALE_2000DPS;
				break;
			case 0x18:	// 24
				gyroscope = Muse_HW::GYROSCOPE_FULL_SCALE_4000DPS;
				break;
			}
		}
	}

	return gyroscope;
}

uint8_t Muse_SerialConnection::getAccFullScale() {

	uint8_t acc = 0;

	if (!checkConnectionStatus())
		return acc;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P') {

			switch (configuration_buffer[3])
			{
			case 0x00:	// 0
				acc = Muse_HW::ACCELEROMENTER_FULL_SCALE_2g;
				break;
			case 0x08:	// 8
				acc = Muse_HW::ACCELEROMENTER_FULL_SCALE_4g;
				break;
			case 0x10:	// 16
				acc = Muse_HW::ACCELEROMENTER_FULL_SCALE_6g;
				break;
			case 0x18:	// 24
				acc = Muse_HW::ACCELEROMENTER_FULL_SCALE_8g;
				break;
			case 0x20:	// 32
				acc = Muse_HW::ACCELEROMENTER_FULL_SCALE_16g;
				break;
			}
		}
	}

	return acc;
}

uint16_t Muse_SerialConnection::getAccHdrFullScale() {

	uint16_t acc_hdr = 0;

	if (!checkConnectionStatus())
		return acc_hdr;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P') {

			switch (configuration_buffer[5])
			{
			case 0x00:	// 0
				acc_hdr = Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_100g;
				break;
			case 0x10:	// 16
				acc_hdr = Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_200g;
				break;
			case 0x18:	// 24
				acc_hdr = Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_400g;
				break;
			}
		}
	}

	return acc_hdr;
}

uint8_t Muse_SerialConnection::getMagnetometerFullScale() {

	uint8_t mag = 0;

	if (!checkConnectionStatus())
		return mag;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P') {

			switch (configuration_buffer[4])
			{
			case 0x00:	// 0
				mag = Muse_HW::MAGNETOMETER_FULL_SCALE_2G;
				break;
			case 0x20:	// 32
				mag = Muse_HW::MAGNETOMETER_FULL_SCALE_4G;
				break;
			case 0x40:	// 64
				mag = Muse_HW::MAGNETOMETER_FULL_SCALE_8G;
				break;
			case 0x60:	// 96
				mag = Muse_HW::MAGNETOMETER_FULL_SCALE_12G;
				break;
			}
		}
	}

	return mag;
}

uint8_t Muse_SerialConnection::getLogMode() {

	uint8_t log_mode = UINT8_MAX;

	if (!checkConnectionStatus())
		return log_mode;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P')
			log_mode = (uint16_t)(configuration_buffer[6]);

	}

	return log_mode;
}

uint8_t Muse_SerialConnection::getLogFrequency() {

	uint8_t log_frequency = UINT8_MAX;

	if (!checkConnectionStatus())
		return log_frequency;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetConfigurationParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t configuration_buffer[Muse_HW::CONFIGURATION_BUFFER_SIZE];
		serial_connection_.Read(configuration_buffer, sizeof(configuration_buffer));

		if (configuration_buffer[0] == 'P' && configuration_buffer[1] == 'P') 
			log_frequency = (uint16_t)(configuration_buffer[7]);
	
	}

	return log_frequency;
}

bool Muse_SerialConnection::setGyroscopeFullScale(const uint16_t value)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	switch (value)
	{
	case Muse_HW::GYROSCOPE_FULL_SCALE_500DPS:
		sprintf(cmd, Muse_HW::SetGyroscopeFullScale, 0x00);	// 0
		break;
	case Muse_HW::GYROSCOPE_FULL_SCALE_1000DPS:
		sprintf(cmd, Muse_HW::SetGyroscopeFullScale, 0x08);	// 8
		break;
	case Muse_HW::GYROSCOPE_FULL_SCALE_2000DPS:
		sprintf(cmd, Muse_HW::SetGyroscopeFullScale, 0x10);	// 16
		break;
	case Muse_HW::GYROSCOPE_FULL_SCALE_4000DPS:
		sprintf(cmd, Muse_HW::SetGyroscopeFullScale, 0x18);	// 24
		break;
	default:
		printf("Gyroscope full scale value not valid.\n");
		out = false;
		return out;
	}

	stopTransmission();

	Sleep(10);

	serial_connection_.Write(cmd);

	return out;

}

bool Muse_SerialConnection::setAccelerometerFullScale(const uint8_t value)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	switch (value)
	{
	case Muse_HW::ACCELEROMENTER_FULL_SCALE_2g:
		sprintf(cmd, Muse_HW::SetAccelerometerFullScale, 0x00);	// 0
		break;
	case Muse_HW::ACCELEROMENTER_FULL_SCALE_4g:
		sprintf(cmd, Muse_HW::SetAccelerometerFullScale, 0x08);	// 8
		break;
	case Muse_HW::ACCELEROMENTER_FULL_SCALE_6g:
		sprintf(cmd, Muse_HW::SetAccelerometerFullScale, 0x10);	// 16
		break;
	case Muse_HW::ACCELEROMENTER_FULL_SCALE_8g:
		sprintf(cmd, Muse_HW::SetAccelerometerFullScale, 0x18);	// 24
		break;
	case Muse_HW::ACCELEROMENTER_FULL_SCALE_16g:
		sprintf(cmd, Muse_HW::SetAccelerometerFullScale, 0x20);	// 32
		break;
	default:
		printf("Accelerometer full scale value not valid.\n");
		out = false;
		return out;
	}

	stopTransmission();

	Sleep(10);

	serial_connection_.Write(cmd);

	return out;
}

bool Muse_SerialConnection::setAccelerometerHDRFullScale(const uint16_t value)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	switch (value)
	{
	case Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_100g:
		sprintf(cmd, Muse_HW::SetAccelerometerHDRFullScale, 0x00);	// 0
		break;
	case Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_200g:
		sprintf(cmd, Muse_HW::SetAccelerometerHDRFullScale, 0x10);	// 16
		break;
	case Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_400g:
		sprintf(cmd, Muse_HW::SetAccelerometerHDRFullScale, 0x18);	// 24
		break;
	default:
		printf("Accelerometer HDR full scale value not valid.\n");
		out = false;
		return out;
	}

	stopTransmission();

	Sleep(10);

	serial_connection_.Write(cmd);

	return out;
}

bool Muse_SerialConnection::setMagnetometerFullScale(const uint8_t value)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	switch (value)
	{
	case Muse_HW::MAGNETOMETER_FULL_SCALE_2G:
		sprintf(cmd, Muse_HW::SetMagnetometerFullScale, 0x00);	// 0
		break;
	case Muse_HW::MAGNETOMETER_FULL_SCALE_4G:
		sprintf(cmd, Muse_HW::SetMagnetometerFullScale, 0x20);	// 32
		break;
	case Muse_HW::MAGNETOMETER_FULL_SCALE_8G:
		sprintf(cmd, Muse_HW::SetMagnetometerFullScale, 0x40);	// 64
		break;
	case Muse_HW::MAGNETOMETER_FULL_SCALE_12G:
		sprintf(cmd, Muse_HW::SetMagnetometerFullScale, 0x60);	// 96
		break;
	default:
		printf("Magnetometer full scale value not valid.\n");
		out = false;
		return out;
	}

	stopTransmission();

	serial_connection_.Write(cmd);

	return out;
}

bool Muse_SerialConnection::setLogMode(const uint8_t mode)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	if (mode == Muse_HW::LOG_NONE || mode == Muse_HW::LOG_QUATERNION || mode == Muse_HW::LOG_HDR || 
		mode == Muse_HW::LOG_RAW || mode == Muse_HW::LOG_RAW_AND_QUATERNION || mode == Muse_HW::LOG_HIGH_RESOLUTION)
		sprintf(cmd, Muse_HW::SetLogMode, mode);
	else {
		printf("Invalid Log Mode.\n");
		out = false;
		return out;
	}

	stopTransmission();

	serial_connection_.Write(cmd);

	return out;
}

bool Muse_SerialConnection::setLogFrequency(const uint8_t frequency)
{
	bool out = true;
	char cmd[Muse_HW::CONFIGURATION_BUFFER_SIZE];

	if (!checkConnectionStatus()) {
		out = false;
		return out;
	}

	if (frequency <= Muse_HW::MAX_LOG_FREQUENCY)
		sprintf(cmd, Muse_HW::SetLogFrequency, frequency);
	else {
		printf("Invalid Log Frequency.\n");
		out = false;
		return out;
	}

	stopTransmission();

	serial_connection_.Write(cmd);

	return out;
}

vector<float> Muse_SerialConnection::getGyroscopeOffset() {

	vector<float> params;

	if (!checkConnectionStatus())
		return params;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetGyroscopeOffset);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t offset_buffer[Muse_HW::GYROSCOPE_OFFSET_BUFFER_SIZE];
		serial_connection_.Read(offset_buffer, sizeof(offset_buffer));

		int c_element_size = 2;
		int offset = 4;

		if (offset_buffer[0] == 'G' && offset_buffer[1] == 'B')
		{
			params.resize(3);

			for (int i = 0; i < params.size(); i++) {
				uint8_t params_array[4] = {offset_buffer[c_element_size + 3 + offset * i], 
					offset_buffer[c_element_size + 2 + offset * i],
					offset_buffer[c_element_size + 1 + offset * i], 
					offset_buffer[c_element_size + offset * i]};
				params[i] = *reinterpret_cast<float*>(params_array);

			}

		}
	}

	return params;
}

vector<float> Muse_SerialConnection::getAccelerometerCalibParams() {

	vector<float> params;

	if (!checkConnectionStatus())
		return params;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetAccelerometerCalibParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t calib_buffer[Muse_HW::CALIBRATION_BUFFER_SIZE];
		serial_connection_.Read(calib_buffer, sizeof(calib_buffer));

		int c_element_size = 2;
		int offset = 4;

		if (calib_buffer[0] == 'C' && calib_buffer[1] == 'C')
		{
			params.resize(Muse_HW::CALIB_PARAMS_MESSAGE_SIZE);

			for (int i = 0; i < params.size(); ++i) {
				uint8_t params_array[4] = {calib_buffer[c_element_size + 3 + offset * i], 
					calib_buffer[c_element_size + 2 + offset * i],
					calib_buffer[c_element_size + 1 + offset * i],
					calib_buffer[c_element_size + offset * i] };

				params[i] = *reinterpret_cast<float*>(params_array);
			}
		}
	}

	return params;
}

vector<float> Muse_SerialConnection::getMagnetometerCalibParams() {

	vector<float> params;

	if (!checkConnectionStatus())
		return params;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetMagnetometerCalibParams);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t calib_buffer[Muse_HW::CALIBRATION_BUFFER_SIZE];
		serial_connection_.Read(calib_buffer, sizeof(calib_buffer));

		int c_element_size = 2;
		int offset = 4;

		if (calib_buffer[0] == 'C' && calib_buffer[1] == 'C')
		{
			params.resize(Muse_HW::CALIB_PARAMS_MESSAGE_SIZE);

			for (int i = 0; i < params.size(); ++i) {
				uint8_t params_array[4] = {calib_buffer[c_element_size + 3 + offset * i],
				calib_buffer[c_element_size + 2 + offset * i],
				calib_buffer[c_element_size + 1 + offset * i], 
				calib_buffer[c_element_size + offset * i] };

				params[i] = *reinterpret_cast<float*>(params_array);
			}
		}
	}

	return params;

}

bool Muse_SerialConnection::isFrequencyAdmissible(uint8_t frequency) {
	bool result = false;

	if (frequency <= Muse_HW::MAX_STREAM_FREQUENCY)
		result = true;
	else { printf("Invalid frequency."); }

	return result;

}

Acceleration Muse_SerialConnection::getAcceleration(uint8_t frequency) {

	Acceleration out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);
	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];

		int a_components = 3;
		int a_element_size = 2;
		int offset = 24;

		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if (stream_buffer[22] == 'A' && stream_buffer[23] == 'A')
		{
			short* a_tmp = new short[a_components];

			for (int j = 0; j < a_components; j++) {
				uint8_t acceleration[2] = {stream_buffer[j * a_element_size + offset + 1], stream_buffer[j * a_element_size + offset] };
				a_tmp[j] = *reinterpret_cast<short*>(acceleration);
			}

			out.x = a_tmp[0];
			out.y = a_tmp[1];
			out.z = a_tmp[2];

		}
	}

	return out;
}

AngularVelocity Muse_SerialConnection::getAngularVelocity(uint8_t frequency) {

	AngularVelocity out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);

	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		int g_components = 3;
		int g_element_size = 2;
		int offset = 7;

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];

		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if (stream_buffer[6] == 'G' && stream_buffer[7] == 'G')
		{
			short* g_tmp = new short[g_components];

			for (int j = 0; j < g_components; j++) {
				uint8_t angular_velocity[2] = { stream_buffer[j * g_element_size + offset + 1], stream_buffer[j * g_element_size + offset] };
				g_tmp[j] = *reinterpret_cast<short*>(angular_velocity);
			}

			out.x = g_tmp[0];
			out.y = g_tmp[1];
			out.z = g_tmp[2];
		}
	}

	return out;
}

Imu Muse_SerialConnection::getIMU(uint8_t frequency) {
	
	Imu out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);

	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		int q_components = 4;
		int q_element_size = 4;
		float* q_tmp = new float[q_components];

		int g_components = 3;
		int g_element_size = 2;
		short* g_tmp = new short[g_components];

		int a_components = 3;
		int a_element_size = 2;
		short* a_tmp = new short[a_components];

		int offset;

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];
		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if ((stream_buffer[36] == 'Q' && stream_buffer[37] == 'Q') &&
			(stream_buffer[6] == 'G' && stream_buffer[7] == 'G') &&
			(stream_buffer[22] == 'A' && stream_buffer[23] == 'A'))
		{

			// Quaternion

			offset = 38;

			for (int j = 0; j < q_components; j++) {
				uint8_t orientation[4] = {stream_buffer[j * q_element_size + offset +3], stream_buffer[j * q_element_size + offset + 2],
					stream_buffer[j * q_element_size + offset + 1], stream_buffer[j * q_element_size + offset]};
				q_tmp[j] = *reinterpret_cast<float*>(orientation);
			}

			out.quaternion.w = q_tmp[0];
			out.quaternion.x = q_tmp[1];
			out.quaternion.y = q_tmp[2];
			out.quaternion.z = q_tmp[3];

			// Gyroscope - Angular velocity

			offset = 8;

			for (int j = 0; j < g_components; j++) {
				uint8_t angular_velociry[2] = {stream_buffer[j * g_element_size + offset+1], stream_buffer[j * g_element_size + offset]};
				g_tmp[j] = *reinterpret_cast<short*>(angular_velociry);
			}

			out.gyroscope.x = g_tmp[0];
			out.gyroscope.y = g_tmp[1];
			out.gyroscope.z = g_tmp[2];

			//Linear Acceleration

			offset = 24;

			for (int j = 0; j < a_components; j++) {
				uint8_t linear_acceleration[2] = { stream_buffer[j * a_element_size + offset+1], stream_buffer[j * a_element_size + offset] };
				a_tmp[j] = *reinterpret_cast<short*>(linear_acceleration);
			}

			out.linear_acceleration.x = a_tmp[0];
			out.linear_acceleration.y = a_tmp[1];
			out.linear_acceleration.z = a_tmp[2];

		}

	}

	return out;
}

MagneticField Muse_SerialConnection::getMag(uint8_t frequency) {
	MagneticField out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);

	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		int m_components = 3;
		int m_element_size = 2;
		int offset = 16;

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];
		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if (stream_buffer[14] == 'M' && stream_buffer[15] == 'M')
		{
			short* m_tmp = new short[m_components];

			for (int j = 0; j < m_components; j++) {
				uint8_t magnetic_field[2] = { stream_buffer[j * m_element_size + offset+1], stream_buffer[j * m_element_size + offset] };
				m_tmp[j] = *reinterpret_cast<short*>(magnetic_field);
			}

			out.x = m_tmp[0];
			out.y = m_tmp[1];
			out.z = m_tmp[2];

		}
	}

	return out;
}

Quaternion Muse_SerialConnection::getQuaternion(uint8_t frequency) {
	Quaternion out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);

	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		int q_components = 4;
		int q_element_size = 4;
		int offset = 38;

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];
		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if (stream_buffer[36] == 'Q' && stream_buffer[37] == 'Q')
		{

			float* q_tmp = new float[q_components];

			for (int j = 0; j < q_components; j++) {
				uint8_t quaternion[4] = { stream_buffer[j * q_element_size + offset+3], stream_buffer[j * q_element_size + offset + 2],
					stream_buffer[j * q_element_size + offset + 1], stream_buffer[j * q_element_size + offset] };
				q_tmp[j] = *reinterpret_cast<float*>(quaternion);
			}

			out.w = q_tmp[0];
			out.x = q_tmp[1];
			out.y = q_tmp[2];
			out.z = q_tmp[3];

		}
	}

	return out;
}

EulerAngles Muse_SerialConnection::getRPY(uint8_t frequency) {

	EulerAngles out = {};

	if (!checkConnectionStatus())
		return out;

	while (serial_connection_.Available() > 0)
		serial_connection_.Read();

	char cmd[sizeof(Muse_HW::StreamRawData) + sizeof(frequency)];
	sprintf(cmd, Muse_HW::StreamRawData, frequency);

	serial_connection_.Write(cmd);

	transmission_status_ = true;

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		int q_components = 4;
		int q_element_size = 4;
		int offset = 38;

		uint8_t stream_buffer[Muse_HW::RAW_DATA_MESSAGE_SIZE];
		serial_connection_.Read(stream_buffer, sizeof(stream_buffer));

		if (stream_buffer[36] == 'Q' && stream_buffer[37] == 'Q')
		{
			float* q_tmp = new float[q_components];

			for (int j = 0; j < q_components; j++) {
				uint8_t quaternion[4] = { stream_buffer[j * q_element_size + offset+3], stream_buffer[j * q_element_size + offset + 2],
					stream_buffer[j * q_element_size + offset + 1], stream_buffer[j * q_element_size + offset]};
				q_tmp[j] = *reinterpret_cast<float*>(quaternion);
			}

			Quaternion q;

			q.w = q_tmp[0];
			q.x = q_tmp[1];
			q.y = q_tmp[2];
			q.z = q_tmp[3];

			// roll (x-axis rotation)
			float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
			float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
			out.roll = atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			float sinp = 2 * (q.w * q.y - q.z * q.x);
			if (abs(sinp) >= 1)
				out.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			else
				out.pitch = asin(sinp);

			// yaw (z-axis rotation)
			float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
			float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
			out.yaw = atan2(siny_cosp, cosy_cosp);

		}
	}

	return out;
}

uint32_t Muse_SerialConnection::getAvailableMemory() {

	uint32_t out = 0;

	if (!checkConnectionStatus())
		return out;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetAvailableMemory);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		uint8_t memory_buffer[Muse_HW::MEMORY_BUFFER_SIZE];
		serial_connection_.Read(memory_buffer, sizeof(memory_buffer));

		if (memory_buffer[0] == 'S' && memory_buffer[1] == 'S')
		{
			uint8_t memory[4] = {memory_buffer[5], memory_buffer[4], memory_buffer[3],  memory_buffer[2] };
			out = *reinterpret_cast<uint32_t*>(memory) ;
		}
	}
 
	return out;

}

bool  Muse_SerialConnection::eraseMemory() {

	bool out = false;

	if (!checkConnectionStatus())
		return out;

	stopTransmission();

	if (serial_connection_.Write(Muse_HW::EraseMemory) > 0)
		out = true;

	return out;

}

vector<Log> Muse_SerialConnection::getLogs() {

	vector<Log> out = {};

	uint8_t log_ack_buffer[Muse_HW::LOG_ACK_BUFFER_SIZE];
	serial_connection_.Read(log_ack_buffer, sizeof(log_ack_buffer));

	if (log_ack_buffer[0] == 'P' && log_ack_buffer[1] == 'K' && log_ack_buffer[2] > 0 && log_ack_buffer[2] <= 5) {

		int dataIndex = 0;
		short temp[3];
		float tempF[3];
		uint8_t msgData[36];

		for (int j = 8; j < Muse_HW::LOG_BUFFER_SIZE; j++)
		{
			Log log;

			log.mode = log_ack_buffer[2];
			log.freq = log_ack_buffer[3];

			uint8_t timestamp[4] = { log_ack_buffer[7], log_ack_buffer[6], log_ack_buffer[5], log_ack_buffer[4] };
			log.timestamp = *reinterpret_cast<int*>(timestamp);

			if (log.mode > 0 && log.mode < 5) {

				if (j % 2 == 0)
				{

					if (log.mode == Muse_HW::LOG_QUATERNION) {
						uint8_t data[2] = { log_ack_buffer[j + 1], log_ack_buffer[j] };
						temp[dataIndex % 3] = *reinterpret_cast<short*>(data);
					}
					else {
						uint8_t data[2] = { log_ack_buffer[j + 1], log_ack_buffer[j] };
						tempF[dataIndex % 3] = *reinterpret_cast<float*>(data);
						if (log.mode == 2)
							temp[dataIndex % 3] = *reinterpret_cast<short*>(data) * 16;
						else
							temp[dataIndex % 3] = *reinterpret_cast<short*>(data);

					}

					dataIndex++;

					if (dataIndex == 3) {

						if (log.mode == Muse_HW::LOG_QUATERNION) {

							float tempQ[4];
							float tempN = 0;
							for (int k = 0; k < 3; k++)
							{
								tempQ[k + 1] = ((float)temp[k]) / 32767;
								tempN += (tempQ[k + 1] * tempQ[k + 1]);
							}

							tempQ[0] = (float)sqrt(1 - (tempN));
							tempN = 0;
							for (int k = 0; k < 4; k++)
								tempN += (tempQ[k] * tempQ[k]);
							for (int k = 0; k < 4; k++)
								tempQ[k] /= tempN;


							log.quaternion.x = tempQ[1];
							log.quaternion.y = tempQ[2];
							log.quaternion.z = tempQ[3];
							log.quaternion.w = tempQ[0];

						}
						else {
							log.linear_acceleration.x = temp[0];
							log.linear_acceleration.y = temp[1];
							log.linear_acceleration.z = temp[2];

						}

					}

					if (dataIndex == 6) {

						log.gyroscope_angular_velocity.x = tempF[0];
						log.gyroscope_angular_velocity.y = tempF[1];
						log.gyroscope_angular_velocity.z = tempF[2];

						if (log.mode == Muse_HW::LOG_HDR)
							dataIndex = 0;

					}

					if (dataIndex == 9) {

						log.magnetic_field.x = temp[0];
						log.magnetic_field.y = temp[1];
						log.magnetic_field.z = temp[2];

						if (log.mode == Muse_HW::LOG_RAW)
							dataIndex = 0;
					}

					if (dataIndex == 12) {
						float tempQ[4];
						float tempN = 0;
						for (int k = 0; k < 3; k++)
						{
							tempQ[k + 1] = ((float)temp[k]) / 32767;
							tempN += (tempQ[k + 1] * tempQ[k + 1]);
						}

						tempQ[0] = (float)sqrt(1 - (tempN));
						tempN = 0;
						for (int k = 0; k < 4; k++)
							tempN += (tempQ[k] * tempQ[k]);
						for (int k = 0; k < 4; k++)
							tempQ[k] /= tempN;

						log.quaternion.x = tempQ[1];
						log.quaternion.y = tempQ[2];
						log.quaternion.z = tempQ[3];
						log.quaternion.w = tempQ[0];

						dataIndex = 0;
					}

					out.push_back(log);

				}

			}
			if (log.mode == Muse_HW::LOG_HIGH_RESOLUTION) {

				msgData[dataIndex % 36] = log_ack_buffer[j];
				dataIndex++;

				if (dataIndex % 36 == 0)
				{

					// Accelerometer
					uint8_t acc_x[4] = { msgData[3], msgData[2], msgData[1], msgData[0] };
					log.linear_acceleration.x = *reinterpret_cast<float*>(acc_x);

					uint8_t acc_y[4] = { msgData[7], msgData[6], msgData[5], msgData[4] };
					log.linear_acceleration.y = *reinterpret_cast<float*>(acc_y);

					uint8_t acc_z[4] = { msgData[11], msgData[10], msgData[9], msgData[8] };
					log.linear_acceleration.z = *reinterpret_cast<float*>(acc_z);

					// Gyroscope
					uint8_t ang_vel_x[4] = { msgData[12 + 3], msgData[12 + 2], msgData[12 + 1], msgData[12 + 0] };
					log.gyroscope_angular_velocity.x = *reinterpret_cast<float*>(ang_vel_x);

					uint8_t ang_vel_y[4] = { msgData[12 + 7], msgData[12 + 6], msgData[12 + 5], msgData[12 + 4] };
					log.gyroscope_angular_velocity.y = *reinterpret_cast<float*>(ang_vel_y);

					uint8_t ang_vel_z[4] = { msgData[12 + 11], msgData[12 + 10], msgData[12 + 9], msgData[12 + 8] };
					log.gyroscope_angular_velocity.z = *reinterpret_cast<float*>(ang_vel_z);

					// Magnetometer
					uint8_t mag_x[2] = { msgData[24 + 1], msgData[24 + 0] };
					log.magnetic_field.x = *reinterpret_cast<float*>(mag_x);

					uint8_t mag_y[2] = { msgData[24 + 3], msgData[24 + 2] };
					log.magnetic_field.y = *reinterpret_cast<float*>(mag_y);

					uint8_t mag_z[2] = { msgData[24 + 5], msgData[24 + 4] };
					log.magnetic_field.z = *reinterpret_cast<float*>(mag_z);

					// Quaternion
					uint8_t quat_x[2] = { msgData[30 + 1], msgData[30 + 0] };
					temp[0] = *reinterpret_cast<short*>(quat_x);

					uint8_t quat_y[2] = { msgData[30 + 3], msgData[30 + 2] };
					temp[1] = *reinterpret_cast<short*>(quat_y);

					uint8_t quat_z[2] = { msgData[30 + 5], msgData[30 + 4] };
					temp[2] = *reinterpret_cast<short*>(quat_z);

					float tempQ[4];
					float tempN = 0;
					for (int k = 0; k < 3; k++)
					{
						tempQ[k + 1] = ((float)temp[k]) / 32767;
						tempN += (tempQ[k + 1] * tempQ[k + 1]);
					}
					tempQ[0] = sqrt(1 - (tempN));
					tempN = 0;
					for (int k = 0; k < 4; k++)
						tempN += (tempQ[k] * tempQ[k]);
					for (int k = 0; k < 4; k++)
						tempQ[k] /= tempN;

					log.quaternion.x = tempQ[1];
					log.quaternion.y = tempQ[2];
					log.quaternion.z = tempQ[3];
					log.quaternion.w = tempQ[0];

					dataIndex = 0;

					out.push_back(log);
				}
			}


		}
	}

	return out;

}

vector<Log> Muse_SerialConnection::readMemory() {
	vector<Log> out = {};

	if (!checkConnectionStatus())
		return out;

	stopTransmission();

	serial_connection_.Write(Muse_HW::ReadMemory);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {

		uint8_t memory_buffer[Muse_HW::MEMORY_BUFFER_SIZE];
		serial_connection_.Read(memory_buffer, sizeof(memory_buffer));

		if (memory_buffer[0] == 'N' && memory_buffer[1] == 'N') {

			uint8_t bytes[4] = {memory_buffer[5],memory_buffer[4], memory_buffer[3], memory_buffer[2] };
			int bytes_to_read = *reinterpret_cast<int*>(bytes);

			if (bytes_to_read < Muse_HW::LOG_ACK_BUFFER_SIZE)
				return out;

			int i = 0;
			int x = 1;
			int n = 1;
			while (i < bytes_to_read) {

				if ((i / Muse_HW::LOG_BUFFER_SIZE) % 20 == 0) {
					serial_connection_.Write(Muse_HW::PacketOK);
					Sleep(10);
				}

				if (n == (bytes_to_read * 25 * x) / (100 * Muse_HW::LOG_BUFFER_SIZE)) {
					printf("Advancement: %d %%", 25 * x);
					x++;
				}

				vector<Log> temp = getLogs();

				if (temp.empty()) {
					printf("Received a corrupt log.");
					return out;

				}
				else {
					out.insert(out.end(), temp.begin(), temp.end());
					i += Muse_HW::LOG_BUFFER_SIZE;
					n++;
				}

			}

		}
	}

	return out;
}

vector<pair<int, string>>  Muse_SerialConnection::getFiles() {

	vector<pair<int, string>> out = {};

	if (!checkConnectionStatus())
		return out;

	stopTransmission();

	serial_connection_.Write(Muse_HW::GetFiles);

	time_t time_begin = time(0);
	while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

	if (serial_connection_.Available() > 0) {
		
		vector<uint8_t> file_buffer;
		file_buffer.resize(serial_connection_.Available());

		serial_connection_.Read(file_buffer, file_buffer.size());

		if (!file_buffer.empty()) {

			for (int i = 0; i < file_buffer.size(); i++) {

				if (file_buffer[i] == 'F' && file_buffer[i + 1] == 'F') {

					uint8_t n_elems[2] = { file_buffer[i + 3], file_buffer[i + 2] };
					int n = *reinterpret_cast<short*>(n_elems);

					if (n > 0) {

						for (int j = 0; j < n; j++) {

							int c_element_size = 4;
							int offset = 4;

							ptime date_time = time_from_string("1970-01-01 00:00:00.000");

							uint8_t temp[4] = { file_buffer[i + j * offset + c_element_size+3],
								file_buffer[i + j * offset + c_element_size + 2],
								file_buffer[i + j * offset + c_element_size + 1],
								file_buffer[i + j * offset + c_element_size ] };
							date_time += seconds(*reinterpret_cast<int*>(temp));

							out.push_back(make_pair(j, to_simple_string(date_time)));
						}

						return out;
					}

				}

			}

		}
	}

	return out;

}

bool Muse_SerialConnection::storeLogs(string& dest_file, vector<Log>& logs) {

	bool out = false;

	struct stat buffer;
	if (stat(dest_file.c_str(), &buffer) != -1) {

		printf("The file already exists. Overwrite it? (Y/N)");
		string input;
		cin >> input;

		string y = "y";
		string yes = "yes";

		if (
			!equal(input.begin(), input.end(), y.begin(), [](char& c1, char& c2) {
				return (c1 == c2 || toupper(c1) == toupper(c2));
				}) &&
			!equal(input.begin(), input.end(), yes.begin(), [](char& c1, char& c2) {
					return (c1 == c2 || toupper(c1) == toupper(c2));
				})
					) {
			printf("Rename existing file before proceeding");
			return out;
		}

	}

	ofstream file;

	file.open(dest_file);

	if (!file) {
		printf("Error: file could not be opened.");
		out = false;
	}

	if (!logs.empty()) {

		out = true;

		int mode = 0;

		for (auto& l : logs) {

			//first line
			string first_line = "Mode\tFreq\tTimestamp\t";

			if (l.mode != mode) {
				mode = l.mode;

				if (mode == 1)
				{
					first_line += "qw\tqi\tqj\tqk";
				}
				else
				{
					first_line += "AccX\tAccY\tAccZ\tGyroX\tGyroY\tGyroZ\t";
					if (mode > 2)
					{
						first_line += "MagnX\tMagnY\tMagnZ\t";
					}
					if (mode > 3 && mode < 6)
					{
						first_line += "qw\tqi\tqj\tqk";
					}
				}

				file << first_line << endl;

			}

			//line
			string line = to_string(mode) + "\t" + to_string(l.freq) + "\t" + to_string(l.timestamp) + "\t";
			if (mode == 1)
			{
				line += to_string(l.quaternion.w) + "\t" + to_string(l.quaternion.x) + "\t" + to_string(l.quaternion.y) + "\t" + to_string(l.quaternion.z);
			}
			else
			{

				line += to_string(l.linear_acceleration.x) + "\t" + to_string(l.linear_acceleration.y) + "\t" + to_string(l.linear_acceleration.z) + "\t" +
					to_string(l.gyroscope_angular_velocity.x) + "\t" + to_string(l.gyroscope_angular_velocity.y) + "\t" + to_string(l.gyroscope_angular_velocity.z) + "\t";

				if (mode > 2)
				{
					line += to_string(l.magnetic_field.x) + "\t" + to_string(l.magnetic_field.y) + "\t" + to_string(l.magnetic_field.z) + "\t";
				}
				if (mode > 3 && mode < 6)
				{
					line += to_string(l.quaternion.w) + "\t" + to_string(l.quaternion.x) + "\t" + to_string(l.quaternion.y) + "\t" + to_string(l.quaternion.z);
				}
			}

			file << line << endl;

		}

	}

	file.close();

	return out;
}

vector<Log> Muse_SerialConnection::readFile(int file_number) {
	
	vector<Log> out = {};

	vector<pair<int, string >> file_names = getFiles();
	if (file_names.empty()) {
		printf("Files not found");
		return out;
	}

	if (file_number > 0 && file_number <= file_names.size()) {

		if (!checkConnectionStatus())
			return out;

		stopTransmission();

		// Create message
		std::stringstream s;
		s << std::setfill('0') << std::setw(3) << file_number;
		std::string number =Muse_HW::ReadFile + s.str() + "!?";

		// Send message
		serial_connection_.Write(number);

		time_t time_begin = time(0);
		while (serial_connection_.Available() == 0 && ((intmax_t)(time(0) - time_begin) < 100));

		if (serial_connection_.Available() > 0) {

			// Read message

			uint8_t memory_buffer[Muse_HW::MEMORY_BUFFER_SIZE];
			serial_connection_.Read(memory_buffer, sizeof(memory_buffer));

			if (memory_buffer[0] == 'N' && memory_buffer[1] == 'N') {

				uint8_t bytes[4] = { memory_buffer[5], memory_buffer[4], memory_buffer[3], memory_buffer[2] };
				int bytes_to_read = *reinterpret_cast<int*>(bytes);

				if (bytes_to_read < Muse_HW::LOG_BUFFER_SIZE)
					return out;

				int i = 0;
				int x = 1;
				int n = 1;
				while (i < bytes_to_read) {
					vector<Log> temp;

					if ((i / Muse_HW::LOG_BUFFER_SIZE) % 20 == 0) {
						serial_connection_.Write(Muse_HW::PacketOK);
						Sleep(10);
					}

					if (n == (bytes_to_read * 25 * x) / (100 * Muse_HW::LOG_BUFFER_SIZE)) {
						printf("Advancement: %d %%", 25 * x);
						x++;
					}

					temp = getLogs();
					if (temp.empty()) {
						printf("Received a corrupt log.");
						return out;
					}
					else {
						out.insert(out.end(), temp.begin(), temp.end());
						i += Muse_HW::LOG_BUFFER_SIZE;
						n++;
					}

				}

			}
		
		}

	}

	return out;
}