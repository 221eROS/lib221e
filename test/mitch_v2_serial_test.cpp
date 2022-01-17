#include <thread>
#include <mitch_v2/MitchV2_SerialConnection.h>

using namespace MitchV2;

int main()
{
	puts("Trying to Connect.");

	try {

		// Serial Connection object - input: Serial port
		//Mitch_SerialConnection my_serial("/dev/ttyACM0"); 
		MitchV2_SerialConnection my_serial("/dev/ttyACM0");

		// Check if the device is connected
		if (my_serial.checkConnectionStatus() == true) {
			puts("Connected.");

			// Check if the device is in IDLE state
			if (my_serial.getState() == MitchV2_HW::SystemState::SYS_IDLE) { 
				puts("System in Idle state.");

				// Get firmware version
				string firmware_version = my_serial.getFirmwareVersion();
				printf("firmware version : %s \n", firmware_version.c_str());

				// Get BLE name
				string ble_name = my_serial.getBLEName();
				printf("BLE name: %s \n", ble_name.c_str());

				// Get device ID
				uint16_t device_id = my_serial.getDeviceId();
				printf("Device ID: %u \n", device_id);

				// Get current time

				ptime date_time = my_serial.getTime();
				const std::string str_time = to_simple_string(date_time);
				printf("Device Time: %s \n", str_time.c_str());

				// Get clock offset
				uint8_t* clock_offset = my_serial.getClockOffset();
				for(int i = 0; i < sizeof(clock_offset); ++i)
					printf("%02X ", clock_offset[i]);
				printf("\n");

				// Get the device battery voltage
				float battery_voltage = my_serial.getBatteryVoltage();
				printf("Battery Voltage : % .2f [mV] \n", battery_voltage);

				// Get the device battery charge
				float battery_charge = my_serial.getBatteryCharge();
				printf("Battery Charge: %.2f %%\n", battery_charge);

				// Check if the battery is greater than 10%
				if(battery_charge > 10) {

					// Start data acquisition - input: acquisition mode and frequency 
					// Being the connection serial, the log mode is required
					bool acq_started = my_serial.startStreaming(MitchV2_HW::LogMode::LOG_MODE_IMU, MitchV2_HW::LogFrequency::LOG_FREQ_50HZ);
					if (acq_started) {
						puts("Acquisition Started.");

						// Sleep for 1000 ms
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));

						// Stop data acquisition
						my_serial.stopStreaming();
						puts("Acquisition Stopped.");

						// Disconnect the device
						my_serial.disconnect();
						puts("Disconnected.");
					}	
				}
			}
			
		}
		else
			puts("Error opening serial port.");
	}
	catch (SerialConnectionIOException exception) {
		std::cerr << exception.what() << std::endl;
	}

	return 0;
}