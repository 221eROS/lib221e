# 221e Communication Protocol

C++ library for the 221e Communication Protocol.

Supported sensors:
- Multisensor InerTial CHamaleon (MITCH) V2
- MUltiSEnsor (MUSE) V2

Package available for:

 - x64 Windows 10 
 - Ubuntu 20.04 LTS

## Dependencies

Ubuntu:
- [Gattlib](https://github.com/labapart/gattlib) - used to access Generic Attribute Profile (GATT) protocol of BLE (Bluetooth Low Energy) devices.

Windows:
- [Microsoft Visual Studio 2019](https://docs.microsoft.com/en-us/visualstudio/releases/2019/release-notes)
  - include "Desktop development with C++" workload
  - in the Individual Components, select "Windows 10 SDK".

(*Note: if you are using this library with ROS for Windows, remember to start your installation routine from the ROS command short cut.*)

## Quick start (Ubuntu 20.04 LTS version)

### 1.  Install the Ubuntu BLE library:

- Open a terminal and install the dependencies:
```sh
sudo apt install libbluetooth-dev libreadline-dev doxygen
```
- From your terminal, after positioning in the desired folder,

```sh
git clone https://github.com/labapart/gattlib
cd gattlib
mkdir build && cd build
cmake ..
make
sudo make install
```

Multiple examples are available to test the BLE library (in ```/gattlib/build```). Examples follow. 

- Demonstrate BLE scanning and connection:

```sh
./examples/ble_scan/ble_scan
```
- Demonstrate discovering of primary services and characteristics:

```sh
./examples/discover/discover <DEVICE_MAC_ADDRESS>
```

### 2.  Install the 221e library:

Open a terminal and, after positioning in the desired folder,
```sh
git clone https://github.com/221eROS/lib221e.git
cd lib221e
mkdir build
cd build
cmake ..
make
sudo make install
```

### 3.  Test:

Tests are available in ```/lib221e/bin```. Open a terminal and locate in this directory. After connecting the desired device to your laptop,

1. Mitch V2:
- Serial connection: 
```sh
./MitchV2SerialTest
```

- BLE connection:
```sh
./MitchV2BLETest
```

2. Muse V2:
- Serial connection:
```sh
./MuseV2SerialTest
```


## Quick start (Windows 10 version)

### 1.  Install the 221e library:

- Open [GitHub Desktop](https://desktop.github.com/) or the x64 Native Tools Command Prompt for VS 2019 and clone the library repository (```https://github.com/221eROS/lib221e.git```) in a desired folder;
- Open Visual Studio 2019 and open the library folder (```File -> Open -> Folder```);
- Configure the project (```Project -> Configure Project```);
- Build the project (```Build -> Build All```);
- Install the project (```Build -> Install lib221e```).

(*Note: if you are using this library with ROS for Windows, remember to start your installation routine from the ROS command short cut.*)
### 2.  Test:

Tests are available in ```/lib221e/bin```. Details follows:

1. Mitch V2:
- Serial connection: ```MitchV2SerialTest.exe```

2. Muse V2:
- Serial connection: ```MuseV2SerialTest.exe```

After connecting the desired device to your laptop, 
- Right Click on the desired test;
- Debug.
