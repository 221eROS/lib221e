#include <thread>
#include <signal.h>
#include <mitch/Mitch_BLEConnection.h>

using namespace _221e_Mitch;

int main(int argc, const char *argv[])
{
	puts("Trying to Connect.");

	// BLE Connection object - input: MAC address, command UUID, data UUID, .json file path
	Mitch_BLEConnection my_ble("F2:71:C5:7A:6F:49", "d5913036-2d8a-41ee-85b9-4e361aa5c8a7", "09bf2c52-d1d9-c0b7-4145-475964544307", "/home/test/example.json");
	//MitchBLEConnection my_ble("C7:87:A9:11:CF:62", "d5913036-2d8a-41ee-85b9-4e361aa5c8a7", "09bf2c52-d1d9-c0b7-4145-475964544307");
	
	// Scan BLE devices
	my_ble.BLEScan();

	// Connect the input device
	my_ble.connect();

	// Check if the device is connected and if the input UUIDs are not malformed
	if(my_ble.checkConnectionStatus() == true && my_ble.checkUUIDStatus() == true){
		
		// Write the battery charge command
		my_ble.sendBatteryChargeCommand();

		// Get the device battery charge
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		double battery_charge =  my_ble.getBatteryCharge();
		printf("Battery Charge: %.2f %\n", battery_charge);

		// Check if the battery is greater than 10%
		if(battery_charge > 10){

			// Write the Get State command
			my_ble.sendGetStateCommand();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// Check if the device is in IDLE state
			if(my_ble.getState() == Mitch_HW::SystemState::SYS_IDLE){
				
				// Write the Start Acquisition command - input: acquisition mode and frequency
				my_ble.sendStartAcquisitionCommand(Mitch_HW::StreamMode::STREAM_MODE_6DOF, Mitch_HW::STREAM_FREQ_25Hz);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));

				// Check if the acquisition is started
				printf("Acquisition started: %s \n", my_ble.acquisitionStarted() ? "true" : "false");

				// Wait for 10000 ms
				std::this_thread::sleep_for(std::chrono::milliseconds(10000));
				
				// Stop the Acquisition
				my_ble.sendStopAcquisitionCommand();
				puts("Acquisition Stopped.");

				// Disconnect the device
				my_ble.disconnect();
				puts("Disconnected.");
			}
			
		}

	}
	else{
		puts("Not connected or UUID malformed");
		return -1;
	}

	return EXIT_SUCCESS;
}