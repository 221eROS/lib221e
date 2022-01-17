#ifndef MUSE_V2_SERIAL_CONNECTION_H
#define MUSE_V2_SERIAL_CONNECTION_H

#include <muse_v2/MuseV2_HW.h>
#include <serial/SerialConnection.h>

#ifdef _WIN32
#define NOMINMAX
#include <serial/impl/SerialConnectionWindows.h>
#else
#include <serial/impl/SerialConnectionUnix.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#include <chrono>
#include <array>
#include <locale>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/date_time/posix_time/posix_time.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace Connection;
using namespace std;
using namespace boost::posix_time;

namespace MuseV2
{
		struct Acceleration {
			float x, y, z;
		};

		struct AngularVelocity {
			float x, y, z;
		};

		struct Quaternion {
			float w, x, y, z;
		};

		struct EulerAngles {
			float roll, pitch, yaw;
		};

		struct Imu {
			Quaternion quaternion;
			AngularVelocity gyroscope;
			Acceleration linear_acceleration;
		};

		struct MagneticField {
			float x, y, z;
		};

		struct Log {
			short mode;
			short freq;
			int timestamp;
			Acceleration linear_acceleration;
			AngularVelocity gyroscope_angular_velocity;
			MagneticField magnetic_field;
			Quaternion quaternion;
		};

		class MuseV2_SerialConnection
		{
		private:
			SerialConnection serial_connection_;

			bool connection_status_;
			bool transmission_status_;

			std::string serial_port_;

			// Streaming utilities
			bool isFrequencyAdmissible(uint8_t frequency);

		public:

			MuseV2_SerialConnection(const std::string& port = "",
				uint32_t baudrate = 115200,
				Timeout timeout = Timeout(),
				bytesize_t bytesize = bytesize_t::eightbits,
				parity_t parity = parity_t::parity_none,
				stopbits_t stopbits = stopbits_t::stopbits_one,
				flowcontrol_t flowcontrol = flowcontrol_t::flowcontrol_none);
			~MuseV2_SerialConnection();

			// Properties

			/**
			* @brief Check the connection status
			*
			* @return true if the device is connected
			* @return false otherwise
			*/
			bool checkConnectionStatus();

			/**
			* @brief Check the transmission status
			*
			* @return true if the device is transmitting data
			* @return false otherwise
			*/
			bool checkTransmissionStatus();

			// General

			/**
			* @brief Stop data transmission
			*
			*/
			void stopTransmission();

			/**
			 * @brief Shut down the device
			 *
			 */
			void shutdown();

			/**
			 * @brief Disconnect the device
			 *
			 */
			void disconnect();

			// Battery

			/**
			* @brief Get the Battery Charge level [%]
			*
			* @return float - The percentage of battery charge
			*/
			float getBatteryCharge();

			/**
			* @brief Get the Battery Voltage [mV]
			*
			* @return float - The battery voltage
			*/
			float getBatteryVoltage();

			// Configuration

			uint16_t getGyroscopeFullScale();
			uint8_t getAccFullScale();
			uint16_t getAccHdrFullScale();
			uint8_t getMagnetometerFullScale();
			uint8_t getLogMode();
			uint8_t getLogFrequency();

			bool setGyroscopeFullScale(const uint16_t value);
			bool setAccelerometerFullScale(const uint8_t value);
			bool setAccelerometerHDRFullScale(const uint16_t value);
			bool setMagnetometerFullScale(const uint8_t value);
			bool setLogMode(const uint8_t mode);
			bool setLogFrequency(const uint8_t frequency);

			// Calibration

			vector<float> getGyroscopeOffset();
			vector<float> getAccelerometerCalibParams();
			vector<float> getMagnetometerCalibParams();

			// Streaming
			
			Acceleration getAcceleration(uint8_t frequency);
			AngularVelocity getAngularVelocity(uint8_t frequency);
			Imu getIMU(uint8_t frequency);
			MagneticField getMag(uint8_t frequency);
			Quaternion getQuaternion(uint8_t frequency);
			EulerAngles getRPY(uint8_t frequency);

			// Memory

			uint32_t getAvailableMemory();
			bool eraseMemory();
			vector<Log> getLogs();
			vector<Log> readMemory();
			vector<pair<int, string>> getFiles();
			bool storeLogs(string& dest_file, vector<Log>& logs);
			vector<Log> readFile(int file_number);
			

		};
}
#endif
