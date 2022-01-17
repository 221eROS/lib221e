#ifndef MITCH_V2_SERIAL_CONNECTION_H
#define MITCH_V2_SERIAL_CONNECTION_H

#include <mitch_v2/MitchV2_HW.h>
#include <serial/SerialConnection.h>

#ifdef _WIN32
#define NOMINMAX
#include <serial/impl/SerialConnectionWindows.h>
#else
#include <serial/impl/SerialConnectionUnix.h>
#endif

#include <chrono>
#include <array>
#include <locale>
#include <string>
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace Connection;
using namespace MitchV2;
using namespace boost::posix_time;

namespace MitchV2
{

	/**
	 * @brief Serial Connection
	 *
	 * Connect to a Mitch device via Serial Port
	 */
	class MitchV2_SerialConnection
	{
	private:
		SerialConnection serial_connection_;

		bool connection_status_;
		bool transmission_status_;

		std::string serial_port_;

		static const int ACK_OFFSET = 4;
		static const int VALUE_OFFSET = 2;

	public:
		/**
		 * @brief Simple constructor
		 *
		 * Construct a Serial Connection object and open the port if a port is specified
		 * @param port  The serial port identifier
		 * @param baudrate The baudrate, default is 115200
		 * @param timeout The timeout condition for the serial port
		 * @param bytesize The size of each byte in the data serial transmission, default is eightbits
		 * @param parity The method of parity, default is parity_none
		 * @param stopbits The number of exploited stop bits, default is stopbits_one
		 * @param flowcontrol The type of exploited flowcontrol, default is flowcontrol_none
		 */
		MitchV2_SerialConnection(const std::string &port = "",
							   uint32_t baudrate = 115200,
							   Timeout timeout = Timeout(),
							   bytesize_t bytesize = bytesize_t::eightbits,
							   parity_t parity = parity_t::parity_none,
							   stopbits_t stopbits = stopbits_t::stopbits_one,
							   flowcontrol_t flowcontrol = flowcontrol_t::flowcontrol_none);

		~MitchV2_SerialConnection();

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

		// State
		
		/**
		 * @brief Compose a command buffer to impose a certain system state
		 * 
		 * @param enable_Read Enable the reading of the system state, default is true
		 * @param state The desired system state, default is Mitch_HW::SystemState::SYS_NULL
		 * @param state_params The desired state parameters, default is NULL
		 * @return uint8_t* - The command buffer
		 */
		static uint8_t *cmdState(bool enable_Read = true, MitchV2_HW::SystemState state = MitchV2_HW::SystemState::SYS_NULL, uint8_t *state_params = NULL);
		
		/**
		 * @brief Get the current state of the device
		 * 
		 * @return Mitch_HW::SystemState - The state of the device
		 */
		MitchV2_HW::SystemState getState();

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

		// Utils

		/**
		* @brief Get the current firmware version
		*
		* @return string - The firmware version
		*/
		string getFirmwareVersion();

		/**
		* @brief Get the name of the device�s Bluetooth
		*
		* @return string - The name od the device's Bluetooth
		*/
		string getBLEName();

		/**
		* @brief Get the device ID
		*
		* @return uint16_t - The device ID
		*/
		uint16_t getDeviceId();

		// Time

		/**
		* @brief Get the time of the system
		*
		* @return time_t - The current time in the form "GMT: Wednesday 12 September 2018 07:08:51"
		*/
		ptime getTime();

		// Clock

		/*
		* @brief Get the clock offset
		* 
		* @return uint64_t - The clock offset
		*/
		uint8_t * getClockOffset();

		// Acquisition

		/**
		 * @brief Start data streaming - in case of serial connection, streams are logs
		 *
		 * @param mode A Mitch_HW::LogMode value representing the log mode
		 * @param frequency A Mitch_HW::LogFrequency value representing the log frequency
		 * @return true if the streaming is started
		 * @return false otherwise
		 */
		bool startStreaming(MitchV2_HW::LogMode mode, MitchV2_HW::LogFrequency frequency);

		/**
		 * @brief Stop data streaming
		 *
		 * @return true if the streaming is stopped
		 * @return false otherwise
		 */
		bool stopStreaming();

		/**
		* @brief Start data log
		*
		* @param mode A Mitch_HW::LogMode value representing the log mode
		* @param frequency A Mitch_HW::LogFrequency value representing the log frequency
		* @return true if the streaming is started
		* @return false otherwise
		*/
		bool startLog(MitchV2_HW::LogMode mode, MitchV2_HW::LogFrequency frequency);

		/**
		* @brief Stop data streaming
		*
		* @return true if the streaming is stopped
		* @return false otherwise
		*/
		bool stopLog();
	};
}
#endif