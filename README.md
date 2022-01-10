# Mitch Driver Library 

Package available for:

 - x64 Windows 10 
 - Ubuntu 20.04 LTS

## Quick start (Ubuntu 20.04 LTS version)

### 1.  Install the BLE library dependencies: 

```sh
sudo apt install libbluetooth-dev libreadline-dev doxygen
```

### 2.  Install the Ubuntu BLE library:

Open a terminal and, after positioning in the desired folder,

```sh
clone https://github.com/labapart/gattlib
cd gattlib
mkdir build && cd build
cmake ..
make
sudo make install
```

Multiple examples are available to test the BLE library (in ```/gattlib/build```):  

- Demonstrate BLE scanning and connection:

```sh
./examples/ble_scan/ble_scan
```
- Demonstrate discovering of primary services and characteristics:

```sh
./examples/discover/discover <DEVICE_MAC_ADDRESS>
```

### 3.  Install the Mitch library:

Open a terminal and, after positioning in the desired folder,
```sh
git clone https://gitlab.com/221e-softwaredevel/rosenv/mitch_driver.git
cd mitch_driver
mkdir build
cd build
cmake ..
make
sudo make install
```

### 4.  Test both the serial and the BLE use cases:

Tests are available in ```/mitch_driver/bin```. From this location:

- Serial connection: 
```sh
./MitchSerialTest
```

- BLE connection:
```sh
./MitchBLETest
```
