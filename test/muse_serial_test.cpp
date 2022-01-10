#include <thread>
#include <muse/Muse_SerialConnection.h>

using namespace Muse;

int main()
{
	puts("Trying to Connect.");

	try {

		// Serial Connection object - input: Serial port
		Muse_SerialConnection *my_serial = new Muse_SerialConnection("COM5");
		//Muse_SerialConnection my_serial("COM5");

		// Check if the device is connected
		if (my_serial->checkConnectionStatus() == true) {
			puts("Connected.");

			// Get the device battery voltage
			float battery_voltage = my_serial->getBatteryVoltage();
			printf("Battery Voltage : % .2f [mV] \n", battery_voltage);

			// Get the device battery charge
			float battery_charge = my_serial->getBatteryCharge();
			printf("Battery Charge: %.2f %%\n", battery_charge);

			// Configuration

			// Set new values
			my_serial->setGyroscopeFullScale(Muse_HW::GYROSCOPE_FULL_SCALE_2000DPS);
			my_serial->setAccelerometerFullScale(Muse_HW::ACCELEROMENTER_FULL_SCALE_4g);
			my_serial->setAccelerometerHDRFullScale(Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_200g);
			my_serial->setMagnetometerFullScale(Muse_HW::MAGNETOMETER_FULL_SCALE_8G);
			my_serial->setLogMode(Muse_HW::LOG_RAW);
			my_serial->setLogFrequency(Muse_HW::STREAM_FREQ_50HZ);

			// Show

			// Get the gyroscope full scale
			uint16_t gyr_full_scale = my_serial->getGyroscopeFullScale();
			printf("Gyroscope Full Scale: %u [dps]\n", gyr_full_scale);

			// Get the acceleromenter full scale
			uint16_t acc_full_scale = my_serial->getAccFullScale();
			printf("Acceleromenter Full Scale: %u [g]\n", acc_full_scale);

			// Get the acceleromenter HDR full scale
			uint16_t acc_hdr_full_scale = my_serial->getAccHdrFullScale();
			printf("Acceleromenter HDR Full Scale: %u [g]\n", acc_hdr_full_scale);

			// Get the magnetometer full scale
			uint16_t mag_full_scale = my_serial->getMagnetometerFullScale();
			printf("Magnetometer Full Scale: %u [G]\n", mag_full_scale);

			// Get the log mode
			uint16_t log_mode = my_serial->getLogMode();
			printf("Log Mode: %u \n", log_mode);

			// Get the log frequency
			uint16_t log_freq = my_serial->getLogFrequency();
			printf("Log Frequency: %u [Hz]\n", log_freq);

			// Set again the old values
			my_serial->setGyroscopeFullScale(Muse_HW::GYROSCOPE_FULL_SCALE_4000DPS);
			my_serial->setAccelerometerFullScale(Muse_HW::ACCELEROMENTER_FULL_SCALE_16g);
			my_serial->setAccelerometerHDRFullScale(Muse_HW::ACCELEROMENTER_HDR_FULL_SCALE_100g);
			my_serial->setMagnetometerFullScale(Muse_HW::MAGNETOMETER_FULL_SCALE_4G);
			my_serial->setLogMode(Muse_HW::LOG_HIGH_RESOLUTION);
			my_serial->setLogFrequency(Muse_HW::STREAM_FREQ_200HZ);

			// Show

			// Get the gyroscope full scale
			gyr_full_scale = my_serial->getGyroscopeFullScale();
			printf("Gyroscope Full Scale: %u [dps]\n", gyr_full_scale);

			// Get the acceleromenter full scale
			acc_full_scale = my_serial->getAccFullScale();
			printf("Acceleromenter Full Scale: %u [g]\n", acc_full_scale);

			// Get the acceleromenter HDR full scale
			acc_hdr_full_scale = my_serial->getAccHdrFullScale();
			printf("Acceleromenter HDR Full Scale: %u [g]\n", acc_hdr_full_scale);

			// Get the magnetometer full scale
			mag_full_scale = my_serial->getMagnetometerFullScale();
			printf("Magnetometer Full Scale: %u [G]\n", mag_full_scale);

			// Get the log mode
			log_mode = my_serial->getLogMode();
			printf("Log Mode: %u \n", log_mode);

			// Get the log frequency
			log_freq = my_serial->getLogFrequency();
			printf("Log Frequency: %u [Hz]\n", log_freq);

			// Calibration

			// Get gyroscope offset
			vector<float> gyroscope = my_serial->getGyroscopeOffset();

			// Get accelerometer calib params
			vector<float> accelerometer = my_serial->getAccelerometerCalibParams();

			// Get magnetometer calib params
			vector<float> magnetometer = my_serial->getMagnetometerCalibParams();

			stringstream stream;
			stream.str(string());

			stream << "X" << "\t" << "Y" << "\t" << "Z" << "\n";
			stream << setprecision(3) << fixed << gyroscope[0] << "\t" << gyroscope[1] << "\t" << gyroscope[2];

			printf("Gyroscope Calibration params: \n %s \n", stream.str().c_str());

			stream.str(string());

			stream << "X" << "\t" << "Y" << "\t" << "Z" << "\t" << "offset" << "\n";
			stream << setprecision(3) << fixed << accelerometer[0] << "\t" << accelerometer[1] << "\t" << accelerometer[2] << "\t" << accelerometer[9] << "\n";
			stream << accelerometer[3] << "\t" << accelerometer[4] << "\t" << accelerometer[5] << "\t" << accelerometer[10] << "\n";
			stream << accelerometer[6] << "\t" << accelerometer[7] << "\t" << accelerometer[8] << "\t" << accelerometer[11];

			printf("Accelerometer Calibration params: \n %s \n", stream.str().c_str());

			stream.str(string());

			stream << "X" << "\t" << "Y" << "\t" << "Z" << "\t" << "offset" << "\n";
			stream << setprecision(3) << fixed << magnetometer[0] << "\t" << magnetometer[1] << "\t" << magnetometer[2] << "\t" << magnetometer[9] << "\n";
			stream << magnetometer[3] << "\t" << magnetometer[4] << "\t" << magnetometer[5] << "\t" << magnetometer[10] << "\n";
			stream << magnetometer[6] << "\t" << magnetometer[7] << "\t" << magnetometer[8] << "\t" << magnetometer[11];

			printf("Magnetometer Calibration params: \n %s \n", stream.str().c_str());
			
			// Streaming
	
			time_t time_begin = time(0);

			Acceleration acc;
			AngularVelocity ang_vel;
			Imu imu;
			MagneticField mag;
			Quaternion quat;
			EulerAngles rpy;

			while (((intmax_t)(time(0) - time_begin) < 1)) {

				// Get current acceleration
				acc = my_serial->getAcceleration(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Acceleration: (x: %.2f, y: %.2f, z: %.2f) \n", acc.x, acc.y, acc.z);

				// Get current angular velocity
				ang_vel = my_serial->getAngularVelocity(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Angular Velocity: (x: %.2f, y: %.2f, z: %.2f) \n", ang_vel.x, ang_vel.y, ang_vel.z);

				// Get current imu 
				imu = my_serial->getIMU(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Imu:  (qw: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, omega_x: %.2f, omega_y: %.2f, omega_z: %.2f, ax: %.2f, ay: %.2f, az: %.2f)\n", 
					imu.quaternion.w, imu.quaternion.x, imu.quaternion.y, imu.quaternion.z,
					imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z,
					imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);

				// Get current magnetic field 
				mag = my_serial->getMag(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Magnetic Field: (x: %.2f, y: %.2f, z: %.2f) \n", mag.x, mag.y, mag.z);

				// Get current quaternion
				quat = my_serial->getQuaternion(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Quaternion: (w: %.2f, x: %.2f, y: %.2f, z: %.2f) \n", quat.w, quat.x, quat.y, quat.z);

				// Get current roll-pitch-yaw
				rpy = my_serial->getRPY(Muse_HW::StreamFrequency::STREAM_FREQ_50HZ);
				printf("Euler Angles:(roll: %.2f, pitch: %.2f, yaw: %.2f) \n", rpy.roll, rpy.pitch, rpy.yaw);
			}

			// Memory

			// Get available memory
			uint32_t available_memory = my_serial->getAvailableMemory();
			printf("Available Memory: %.2f (kB) \n", (float)(available_memory / 1024));

			// Erase memory
			//bool eraseMemory();

			// Get files 
			//vector<pair<int, string>> files = my_serial.getFiles();
			//printf("Available files: \n");
			//for(int i = 0; i < files.size(); ++i)
			//	printf("File %i: (%i, %s) \n", i, files.at(i).first, files.at(i).second.c_str());

			//// Read and store one file
			//int random_index = rand() % files.size();
			//vector<Log> logs = my_serial.readFile(files.at(random_index).first);
			//string dest_file = "C:/Users/ElisaTosello/OneDrive - 221e srl/Documents/dev/random_file.txt";
			//my_serial.storeLogs(dest_file, logs);
			//printf("Log randome file stored in %s\n", dest_file.c_str());

			//// Read and store memory
			//vector<Log> logs = my_serial.readMemory();
			//string dest_file = "C:/Users/ElisaTosello/OneDrive - 221e srl/Documents/dev/memory.txt";
			//my_serial.storeLogs(dest_file, logs);
			//printf("All memory Log files stored in %s\n", dest_file.c_str());
			
			// Disconnect the device
			my_serial->disconnect();
			puts("Disconnected.");
			
		}
		else
			puts("Error opening serial port.");
	}
	catch (SerialConnectionIOException exception) {
		std::cerr << exception.what() << std::endl;
	}

	return 0;
}