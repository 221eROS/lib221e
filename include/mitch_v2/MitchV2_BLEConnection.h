#ifndef MITCH_V2_BLE_CONNECTION_H
#define MITCH_V2_BLE_CONNECTION_H

#include <fstream>
#include <gattlib.h>
#include <sys/queue.h>
#include <functional>
#include <jsoncpp/json/writer.h>

#include <mitch_v2/MitchV2_HW.h>
#include <mitch_v2/MitchV2_StreamingData.h>

#define BLE_SCAN_TIMEOUT 4
#define ACK_OFFSET 4

typedef void (*ble_discovered_device_t)(const char *addr, const char *name);

namespace MitchV2
{
	/**
	 * @brief Bluetooth Low Energy (BLE) Connection
	 *
	 * Connect to a Mitch device via BLE
	 */
	class MitchV2_BLEConnection
	{
	private:

		struct connection_t
		{
			pthread_t thread;
			char *addr;
			LIST_ENTRY(connection_t)
			entries;
		};

		const char * mac_address_;
		const char * cmd_uuid_;
		const char * data_uuid_;

		bool connection_status_;
		bool transmission_status_;
		bool uuid_status_;
		bool notification_status_;

		int cmd_ret_, data_ret_;
		uuid_t g_cmd_uuid_;
		uuid_t g_data_uuid_;

		// Return values
		static float battery_charge_;
		static bool acquisition_started_;
		static bool acquisition_stopped_;
		static MitchV2_HW::SystemState current_state_;
		static float current_gyr_resolution_;
		static float current_axl_resolution_;
		static float current_mag_resolution_;

		LIST_HEAD(listhead, connection_t) g_ble_connections;
		
		// We use a mutex to make the BLE connections synchronous
		pthread_mutex_t g_mutex;

		gatt_connection_t* connection;

		/**
		 * @brief 
		 * 
		 * @param adapter 
		 * @param addr 
		 * @param name 
		 * @param user_data 
		 */
		static void BLEDiscoveredDevice(void *adapter, const char* addr, const char* name, void *user_data);	

		// Handlers

		/**
		 * @brief Start the handlers (battery, acquisition, and state)
		 * 
		 * @param uuid The data UUID
		 * @param data The data buffer
		 * @param data_length The data length
		 * @param user_data Additional user data
		 */
		static void notificationHandler(const uuid_t* uuid, const uint8_t* data, size_t data_length, void* user_data);
		
		/**
		 * @brief Handle the Battery notification from the (data) UUID
		 * 
		 * Check if the data UUID published the battery charge value
		 * 
		 * @param data The data UUID
		 * @return true if handled. In this case, the percentage of battery charge is stored in the global parameter battery_charge_
		 * @return false otherwise
		 */
		static bool handleBattery(const uint8_t *data);
		
		/**
		 * @brief Handle the Acquisition notification from the (data) UUID
		 * 
		 * Check if the data UUID published the acquisition acknowledgment
		 * 
		 * @param data The data UUID 
		 * @return true if handled. In this case, the acquisition is started and this information is stored in (bool) acquisition_started_.
		 * Moreover, the current gyroscope resolution, the current accelerometer resolution, and the magnetometer resolution are stored in 
		 * current_gyr_resolution_, current_axl_resolution_, and current_mag_resolution_
		 * @return false otherwise
		 */
		static bool handleAcquisition(const uint8_t *data);
		
		/**
		 * @brief Handle the State notification from the (data) UUID
		 * 
		 * Check if the data UUID published the device state
		 * 
		 * @param data The data UUID
		 * @return true if handled. In this case, the state of the device is stored in the current_state_
		 * @return false otherwise
		 */
		static bool handleState(const uint8_t *data);
		
		/**
		 * @brief Handle the Streaming Data notification from the (data) UUID
		 * 
		 * @param data The data UUID
		 * @return true if handled. In this case, the streamed data is stored in the predefined .json file
		 * @return false otherwise
		 */
		static bool handleStreamingData(const uint8_t *data);

	public:

		/**
		 * @brief Simple constructor
		 * 
		 * Construct a BLE Connection object and check the input UUIDs structure
		 * @param mac_address The device MAC address
		 * @param cmd_uuid The command UUID
		 * @param data_uuid The data UUID
		 * @param json_file_path The path of your .json output file containing the streamed data
		 */
		MitchV2_BLEConnection(const char * mac_address= "", const char * cmd_uuid = "", const char * data_uuid = "", const char * json_file_path = "");
		
		~MitchV2_BLEConnection();

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

		/**
		 * @brief Check the (command and data) UUIDs status
		 * 
		 * @return true if the UUID is not malformed
		 * @return false otherwise
		 */
		bool checkUUIDStatus();

		/**
		 * @brief Check if the notification is enabled on the GATT characteristic represented by the predefined (command and data) UUIDs
		 * 
		 * @return true if the notification is enabled
		 * @return false otherwise
		 */
		bool checkNotificationStatus();

		// General
	
		//void StopTransmission();
		//void Shutdown();

		/**
		 * @brief Scan the presence of BLE devices
		 * 
		 * @return true if the scan is completed 
		 * @return false if the scan failed
		 */
		bool BLEScan();

		/**
		 * @brief Connect a BLE device
		 * 
		 * @return true if the BLE device is connected
		 * @return false otherwise
		 */
		bool connect();

		/**
		 * @brief Disconnect a BLE device
		 * 
		 */
		void disconnect();

		// State

		/**
		 * @brief Write a Get State command to a GATT (command) characteristic UUID
		 * 
		 * @return true if the command has been written
		 * @return false otherwise (e.g., there has been an error while reading the GATT Characteristic with the specified UUID)
		 */
		bool sendGetStateCommand();

		/**
		 * @brief Get the State value of the device from the current_state_ parameter (if previously notified by the function handleState)
		 * 
		 * @return Mitch_HW::SystemState - The state of the device
		 */
		MitchV2_HW::SystemState getState();

		// Battery

		/**
		 * @brief Write a Battery Charge command to a GATT (command) characteristic UUID
		 * 
		 * @return true if the command has been written
		 * @return false otherwise (e.g., there has been an error while reading GATT Characteristic with the specified UUID) 
		 */
		bool sendBatteryChargeCommand();

		/**
		 * @brief Get the Battery Charge value from the battery_charge_ parameter (if previously notified by the function handleBattery)
		 * 
		 * @return float - The battery charge, default -1
		 */
		static float getBatteryCharge();
			

		// Acquisition

		/**
		 * @brief Write a Start Acquisition command to a GATT (command) characteristic UUID
		 * 
		 * @param mode A Mitch_HW::StreamMode value representing the acquisition mode
		 * @param frequency A Mitch_HW::StreamFrequency value representing the acquisition frequency
		 * @return true if the command has been written
		 * @return false otherwise
		 */
		bool sendStartAcquisitionCommand(MitchV2_HW::StreamMode mode, MitchV2_HW::StreamFrequency frequency);

		/**
		 * @brief Check if the Acquisition is started from the acquisition_started_ parameter (if previously notified by the function handleAcquisition)
		 * 
		 * @return true if the Acquisition is started 
		 * @return false otherwise
		 */
		bool acquisitionStarted(); 

		/**
		 * @brief Write a Stop Acquisition command to a GATT (command) characteristic UUID
		 * 
		 * @return if the command has been written 
		 * @return false otherwise
		 */
		bool sendStopAcquisitionCommand();

		/**
		 * @brief Check if the Acquisition has been stopped
		 * 
		 * @return true if the Acquisition has been stopped
		 * @return false otherwise
		 */
		bool acquisitionStopped();



	};
}
#endif