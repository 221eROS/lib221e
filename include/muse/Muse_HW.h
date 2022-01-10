#ifndef MUSE_HW_H
#define MUSE_HW_H

#include <stdint.h>

namespace Muse
{

	class Muse_HW
	{
	public:
		// Frequency and Clock
		static const uint16_t MAX_STREAM_FREQUENCY = 250;
		static const uint16_t MAX_LOG_FREQUENCY = 250;
		static const uint16_t DEFAULT_LOG_FREQUENCY = 25;

		static const uint32_t CLOCK = 24000;

		// Message Dimensions
		static const uint8_t QUATERNION_MESSAGE_SIZE = 18;
		static const uint8_t RAW_DATA_MESSAGE_SIZE = 54;
		static const uint8_t HDR_DATA_MESSAGE_SIZE = 36;

		// Buffer Dimensions
		static const uint8_t BATTERY_BUFFER_SIZE = 4;
		static const uint8_t CONFIGURATION_BUFFER_SIZE = 13;
		static const uint8_t GYROSCOPE_OFFSET_BUFFER_SIZE = 14;
		static const uint8_t CALIBRATION_BUFFER_SIZE = 50;
		static const uint8_t CALIB_PARAMS_MESSAGE_SIZE = 12;
		static const uint8_t MEMORY_BUFFER_SIZE = 6;
		static const uint8_t LOG_BUFFER_SIZE = 512;
		static const uint8_t LOG_ACK_BUFFER_SIZE = 516;

		// Operating Modes
		typedef enum
		{
			STREAM,
			LOG,
			SYNC,
			FOTA
		} op_mode_t;

		enum GyroscopeFullScale : uint16_t
		{
			GYROSCOPE_FULL_SCALE_500DPS = 500,
			GYROSCOPE_FULL_SCALE_1000DPS = 1000,
			GYROSCOPE_FULL_SCALE_2000DPS = 2000,
			GYROSCOPE_FULL_SCALE_4000DPS = 400
		};

		enum AccelerometerFullScale : uint8_t
		{
			ACCELEROMENTER_FULL_SCALE_2g = 2,
			ACCELEROMENTER_FULL_SCALE_4g = 4,
			ACCELEROMENTER_FULL_SCALE_6g = 6,
			ACCELEROMENTER_FULL_SCALE_8g = 8,
			ACCELEROMENTER_FULL_SCALE_16g = 16
		};

		enum AccelerometerHdrFullScale : uint16_t
		{
			ACCELEROMENTER_HDR_FULL_SCALE_100g = 100,
			ACCELEROMENTER_HDR_FULL_SCALE_200g = 200,
			ACCELEROMENTER_HDR_FULL_SCALE_400g = 400
		};

		enum MagnetometerFullScale : uint8_t
		{
			MAGNETOMETER_FULL_SCALE_2G = 2,
			MAGNETOMETER_FULL_SCALE_4G = 4,
			MAGNETOMETER_FULL_SCALE_8G = 8,
			MAGNETOMETER_FULL_SCALE_12G = 12
		};

		enum StreamFrequency : uint8_t
		{
			STREAM_FREQ_25HZ = 25,
			STREAM_FREQ_50HZ = 50,
			STREAM_FREQ_100HZ = 100,
			STREAM_FREQ_150HZ = 150,
			STREAM_FREQ_200HZ = 200,
			STREAM_FREQ_250HZ = 250
		};

		enum LogMode : uint8_t
		{
			LOG_NONE = 0,
			LOG_QUATERNION = 1,
			LOG_HDR = 2,
			LOG_RAW = 3,
			LOG_RAW_AND_QUATERNION = 4,
			LOG_HIGH_RESOLUTION = 5
		};

		// Calibration and Configuration
		static constexpr const char* SetAccelerometerCalibParams = "?!SETACCAL!?";
		static constexpr const char* GetAccelerometerCalibParams = "?!GETACCAL!?";
		static constexpr const char* SetMagnetometerCalibParams = "?!SETCALIB!?";
		static constexpr const char* GetMagnetometerCalibParams = "?!GETCALIB!?";
		static constexpr const char* GetGyroscopeOffset = "?!GETGBIAS!?";

		static constexpr const char* SetGyroscopeFullScale = "?!GYLFS%03u!?";
		static constexpr const char* SetAccelerometerFullScale = "?!ACLFS%03u!?";
		static constexpr const char* SetAccelerometerHDRFullScale = "?!ACHFS%03u!?";
		static constexpr const char* SetMagnetometerFullScale = "?!MAGFS%03u!?";

		static constexpr const char* GetConfigurationParams = "?!GETPARAM!?";
		static constexpr const char* StoreConfigurationParameters = "?!STOREPAR!?";

		// Streaming
		static constexpr const char* StreamQuaternion = "?!START%03u!?";
		static constexpr const char* StreamRawData = "?!CALIB%03u!?";
		static constexpr const char* StreamHDRData = "?!TXHDR%03u!?";

		// Log
		static constexpr const char* SetLogMode = "?!LOGMD%03u!?";
		static constexpr const char* SetLogFrequency = "?!LOGFS%03u!?";
		static constexpr const char* SetupBluetoothLog = "?!SETUPLOG!?";
		static constexpr const char* StartBluetoothLog = "?!LOGSTART!?";

		// Time Sync
		static constexpr const char* EnterTimeSyncMode = "?!TIMESYNC!?";
		static constexpr const char* ExitTimeSyncMode = "?!";

		static constexpr const char* ComputeClockDrift = "!!";
		static constexpr const char* SetClockDrift = "?!SETDRIFT!?";
		static constexpr const char* GetClockDrift = "?!GETDRIFT!?";

		static constexpr const char* ComputeClockOffset = "??";
		static constexpr const char* SetClockOffset = "?!SETCKOFF!?";
		static constexpr const char* GetClockOffset = "?!GETCKOFF!?";

		static constexpr const char* GetTimestamp = "!?";

		// Memory Management
		static constexpr const char* GetAvailableMemory = "?!MEMSPACE!?";
		static constexpr const char* EraseMemory = "?!ERASEMEM!?";
		static constexpr const char* ReadMemory = "?!READDATA!?";

		static constexpr const char* GetFiles = "?!GETFILES!?";
		static constexpr const char* ReadFile = "?!READF";

		// Acknowledge
		static constexpr const char* PacketOK = "?!PACKETOK!?";
		static constexpr const char* PacketKO = "?!PACKETKO!?";

		// Embedded Bluetooth Module
		static constexpr const char* BTcmdEscape = "@#@$@%";
		static constexpr const char* BTcmdReset = "AT+AB Reset\n";
		static constexpr const char* BTcmdBypass = "AT+AB Bypass\n";
		static constexpr const char* BTcmdRename = "AT+AB DefaultLocalName %s\n";

		// Fota
		static constexpr const char* StartApp = "?!STARTAPP!?";
		static constexpr const char* WriteApp = "?!WRITEAPP!?";
		static constexpr const char* GetStuff = "?!GETSTUFF!?";
		static constexpr const char* EPoint = "!";
		static constexpr const char* Trailer = "!?!?";

		// General
		static constexpr const char* StopTransmission = "?!STOPSEND!?";
		static constexpr const char* Shutdown = "?!SHUTDOWN!?";

		// Miscellaneous
		static constexpr const char* GetBatteryVoltage = "?!GETBATTV!?";
		static constexpr const char* GetBatteryCharge = "?!GETBATTQ!?";

		static constexpr const char* SetDate = "?!SETEPOCH!?";
		static constexpr const char* GetDate = "?!GETEPOCH!?";

		static constexpr const char* GetFirmwareVersion = "?!FIRMWARE!?";

		static constexpr const char* CheckHDRAvailability = "?!HDRMUSE?!?";

		static constexpr const char* EmbeddedMagnetometerCalibration = "?!MAGCALIB!?";
	};
}

#endif