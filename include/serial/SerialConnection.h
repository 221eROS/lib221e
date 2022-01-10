#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H

#include <serial/v8stdint.h>

#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <exception>
#include <stdexcept>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
__LINE__, (message) )

	namespace Connection {

		/*!
		* Enumeration defines the possible bytesizes for the serial port.
		*/
		typedef enum {
			fivebits = 5,
			sixbits = 6,
			sevenbits = 7,
			eightbits = 8
		} bytesize_t;

		/*!
		 * Enumeration defines the possible parity types for the serial port.
		 */
		typedef enum {
			parity_none = 0,
			parity_odd = 1,
			parity_even = 2,
			parity_mark = 3,
			parity_space = 4
		} parity_t;

		/*!
		 * Enumeration defines the possible stopbit types for the serial port.
		 */
		typedef enum {
			stopbits_one = 1,
			stopbits_two = 2,
			stopbits_one_point_five
		} stopbits_t;

		/*!
		 * Enumeration defines the possible flowcontrol types for the serial port.
		 */
		typedef enum {
			flowcontrol_none = 0,
			flowcontrol_software,
			flowcontrol_hardware
		} flowcontrol_t;

		/*!
		 * Structure for setting the timeout [ms] of the serial port.
		 *
		 * To disable the interbyte timeout, set it to Timeout::max().
		 */
		struct Timeout 
		{
			// To manage different definitions of max function
			#ifdef max
			#undef max
			#endif

			static uint32_t max() 
			{ 
				return std::numeric_limits<uint32_t>::max(); 
			}

			/*!
			 * Generates Timeout structs given an absolute timeout.
			 * \param timeout Defines the time in milliseconds until a timeout occurs.
			 * \return Timeout struct.
			 */
			static Timeout simpleTimeout(uint32_t timeout) 
			{
				return Timeout(max(), timeout, 0, timeout, 0);
			}

			/*! Milliseconds between received bytes. */
			uint32_t inter_byte_timeout;

			/*! Milliseconds to wait after calling read. */
			uint32_t read_timeout_constant;

			/*! A multiplier against the number of requested bytes to wait after calling read. */
			uint32_t read_timeout_multiplier;
			
			/*! Milliseconds to wait after calling write. */
			uint32_t write_timeout_constant;

			/*! A multiplier against the number of requested bytes to wait after calling write. */
			uint32_t write_timeout_multiplier;

			explicit Timeout(uint32_t inter_byte_timeout_ = 0,
				uint32_t read_timeout_constant_ = 0,
				uint32_t read_timeout_multiplier_ = 0,
				uint32_t write_timeout_constant_ = 0,
				uint32_t write_timeout_multiplier_ = 0)
				: inter_byte_timeout(inter_byte_timeout_),
				read_timeout_constant(read_timeout_constant_),
				read_timeout_multiplier(read_timeout_multiplier_),
				write_timeout_constant(write_timeout_constant_),
				write_timeout_multiplier(write_timeout_multiplier_)
			{}
		};

		/*! Class that provides a portable serial port interface. */
		class SerialConnection
		{
		public:

			/*!
			 * Creates a Serial Connection object and opens the port if a port is specified,
			 * otherwise it remains closed until Pit221e::Connection::SerialConnection::Open is called.
			 *
			 * \param port Serial port identifier.
			 * \param baudrate The baudrate, default is 115200.
			 * \param timeout The timeout conditions for the serial port. \see Pit221e::Connection::Timeout.
			 * \param bytesize Size of each byte in the serial transmission of data, default is eightbits.
			 * \param parity Method of parity, default is parity_none.
			 * \param stopbits Number of stop bits used, default is stopbits_one.
			 * \param flowcontrol Type of flowcontrol used, default is flowcontrol_none.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionIOException
			 * \throw std::invalid_argument
			 */
			SerialConnection(const std::string &port = "",
							 uint32_t baudrate = 115200,
							 Timeout timeout = Timeout(),
							 bytesize_t bytesize = eightbits,
							 parity_t parity = parity_none,
							 stopbits_t stopbits = stopbits_one,
							 flowcontrol_t flowcontrol = flowcontrol_none);

			/*! Destructor */
			virtual ~SerialConnection();

			/*!
			 * Opens the serial port as long as the port is set and the port isn't already open.
			 *
			 * If the port is provided to the constructor then an explicit call to open is not needed.
			 *
			 * \throw Pit221e::Connection::SerialConnectionException
			 * \throw Pit221e::Connection::SerialConnectionIOException
			 * \throw std::invalid_argument
			 */
			void Open();

			/*! 
			 * Gets the status of the serial port.
			 *
			 * \return Returns true if the port is open, false otherwise.
			 */
			bool IsOpen() const;

			/*! Closes the serial port. */
			void Close();

			/*! Returns the number of characters in the buffer. */
			size_t Available();

			/*! 
			 * Blocks until there is serial data to read or rcTimeout_ have elapsed. 
			 *
			 * \return Returns true if the function exits with the port in a readable state, false otherwise.
			 */
			bool WaitReadable();

			/*! 
			 * Blocks for a period of time corresponding to the transmission time of
			 * count characters at present serial settings. 
			 */
			void WaitByteTimes(size_t count);

			/*! 
			 * Reads a given amount of bytes from the serial port.
			 *
			 * \param buffer Minimum requested buffer size.
			 * \param size Number of bytes to be read.
			 *
			 * \return The number of bytes read.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			size_t Read(uint8_t *buffer, size_t size);

			/*! 
			 * Reads a given amount of bytes from the serial port.
			 *
			 * \param buffer A reference to a std::vector of uint8_t.
			 * \param size Number of bytes to be read, default is 1 byte.
			 *
			 * \return The number of bytes read.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			size_t Read(std::vector<uint8_t> &buffer, size_t size = 1);

			/*! 
			 * Reads a given amount of bytes from the serial port.
			 *
			 * \param buffer A reference to a std::string.
			 * \param size Number of bytes to be read, default is 1 byte.
			 *
			 * \return The number of bytes read.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			size_t Read(std::string &buffer, size_t size = 1);

			/*! 
			 * Reads a given amount of bytes from the serial port.
			 *
			 * \param size Number of bytes to be read, default is 1 byte.
			 *
			 * \return A std::string containing the data read from the port.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			std::string Read(size_t size = 1);

			/*! 
			 * Reads a line or until a given delimiter has been processed.
			 *
			 * \param buffer A std::string reference used to store the data.
			 * \param size Maximum length of a line, default is 65536.
			 * \param eol A string representing the end-of-line delimiter.
			 *
			 * \return The number of bytes read.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			size_t Readline(std::string &buffer, size_t size = 65536, std::string eol = "\n");

			/*! 
			 * Reads a line or until a given delimiter has been processed.
			 *
			 * \param size Maximum length of a line, default is 65536.
			 * \param eol A string representing the end-of-line delimiter.
			 *
			 * \return A std::string containing the read line.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			std::string Readline(size_t size = 65536, std::string eol = "\n");

			/*! 
			 * Reads multiple lines until the serial port times out.
			 *
			 * \param size Maximum length of combined lines, default is 65536.
			 * \param eol A string representing the end-of-line delimiter.
			 *
			 * \return A std::vector<std::string> containing the read lines.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			std::vector<std::string> Readlines(size_t size = 65536, std::string eol = "\n");

			/*! 
			 * Write a string to the serial port.
			 *
			 * \param data A reference to the data to be written.			 
			 * \param size Number of bytes to be written.
			 *
			 * \return Number of bytes actually written.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 * \throw Pit221e::Connection::SerialConnectionIOException
			 */
			size_t Write(const uint8_t *data, size_t size);

			/*! 
			 * Write a string to the serial port.
			 *
			 * \param data A reference to the data to be written.
			 *
			 * \return Number of bytes actually written.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 * \throw Pit221e::Connection::SerialConnectionIOException
			 */
			size_t Write(const std::vector<uint8_t> &data);

			/*! 
			 * Write a string to the serial port.
			 *
			 * \param data A reference to the data to be written.
			 *
			 * \return Number of bytes actually written.
			 *
			 * \throw Pit221e::Connection::SerialConnectionPortNotOpenedException
			 * \throw Pit221e::Connection::SerialConnectionException
			 * \throw Pit221e::Connection::SerialConnectionIOException
			 */
			size_t Write(const std::string &data);

			/*! 
			 * Sets the serial port identifier.
			 *
			 * \param port The address of the serial port.
			 *
			 * \throw std::invalid_argument
			 */
			void SetPortName(const std::string &port);

			/*! 
			 * Gets the serial port identifier.
			 *
			 * \see Pit221e::Connection::SetPortName
			 *
			 * \throw std::invalid_argument
			 */
			std::string GetPortName() const;

			/*! 
			 * Sets the timeout for reads and writes using the Timeout struct.
			 *
			 * \param timeout A Pit221e::Connection::Timeout struct.
			 *
			 * \see Pit221e::Connection::Timeout
			 */
			void SetTimeout(Timeout timeout);

			/*! Sets the timeout for reads and writes. */
			void SetTimeout(uint32_t inter_byte_timeout,
							uint32_t read_timeout_constant,
							uint32_t read_timeout_multiplier,
							uint32_t write_timeout_constant,
							uint32_t write_timeout_multiplier)
			{
				Timeout timeout(inter_byte_timeout,
								read_timeout_constant,
								read_timeout_multiplier,
								write_timeout_constant,
								write_timeout_multiplier);
				return SetTimeout(timeout);
			}

			/*! 
			 * Gets the timeout for reads in seconds.
			 *
			 * \return A Timeout struct.
			 *
			 * \see Pit221e::Connection::SetTimeout
			 */
			Timeout GetTimeout() const;

			/*! 
			 * Sets the baudrate for the serial port.
			 *
			 * \param baudrate An unsigned integer representing the baud rate for the serial port.
			 *
			 * \throw std::invalid_argument
			 */
			void SetBaudrate(uint32_t baudrate);

			/*! 
			 * Gets the baudrate for the serial port.
			 *
			 * \return An unsigned integer representing the baud rate for the serial port.
			 *
			 * \see Pit221e::Connection::SetBaudrate
			 *
			 * \throw std::invalid_argument
			 */
			uint32_t GetBaudrate() const;

			/*! 
			 * Sets the bytesize for the serial port.
			 *
			 * \param bytesize Size of each byte in the serial transmission of data.
			 *
			 * \throw std::invalid_argument
			 */
			void SetBytesize(bytesize_t bytesize);

			/*! 
			 * Gets the bytesize for the serial port.
			 *
			 * \return Size of each byte in the serial transmission of data.
			 *
			 * \see Pit221e::Connection::SetBytesize
			 *
			 * \throw std::invalid_argument
			 */
			bytesize_t GetBytesize() const;

			/*! 
			 * Sets the parity for the serial port.
			 *
			 * \param parity Method of parity.
			 *
			 * \throw std::invalid_argument
			 */
			void SetParity(parity_t parity);

			/*! 
			 * Gets the parity for the serial port.
			 *
			 * \return Method of parity.
			 *
			 * \see Pit221e::Connection::SetParity
			 *
			 * \throw std::invalid_argument
			 */
			parity_t GetParity() const;

			/*! 
			 * Sets the stopbits for the serial port.
			 *
			 * \param stopbits Number of stop bits used.
			 *
			 * \throw std::invalid_argument
			 */
			void SetStopbits(stopbits_t stopbits);

			/*! 
			 * Gets the stopbits for the serial port.
			 *
			 * \return Number of stop bits used.
			 * 
			 * \see Pit221e::Connection::SetStopbits
			 *
			 * \throw std::invalid_argument
			 */
			stopbits_t GetStopbits() const;

			/*! 
			 * Sets the flow control for the serial port.
			 *
			 * \param flowcontrol Type of flowcontrol used.
			 *
			 * \throw std::invalid_argument
			 */
			void SetFlowcontrol(flowcontrol_t flowcontrol);

			/*! 
			 * Gets the flow control for the serial port.
			 *
			 * \return Type of flowcontrol used.
			 *
			 * \see Pit221e::Connection::SetFlowcontrol
			 *
			 * \throw std::invalid_argument
			 */
			flowcontrol_t GetFlowcontrol() const;

			/*! Flush the input and output buffers */
			void Flush();

			/*! Flush the input buffer */
			void FlushInput();

			/*! Flush the output buffer */
			void FlushOutput();

			/*! Sends the RS-232 break signal. */
			void SendBreak(int duration);

			/*! Set the break condition. Default is true. */
			void SetBreak(bool level = true);

			/*! Set the RTS handshaking line. Default is true. */
			void SetRTS(bool level = true);

			/*! Set the DTR handshaking line. Default is true. */
			void SetDTR(bool level = true);

			/*!
			 * Blocks until CTS, DSR, RI, CD changes or something interrupts it.
			 *
			 * \return Returns true if one of the lines changed, false if something else
			 * occurred.
			 *
			 * \throw Pit221e::Connection::SerialConnectionException
			 */
			bool WaitForChange();

			/*! Returns the current status of the CTS line. */
			bool GetCTS();

			/*! Returns the current status of the DSR line. */
			bool GetDSR();

			/*! Returns the current status of the RI line. */
			bool GetRI();

			/*! Returns the current status of the CD line. */
			bool GetCD();

			// Disable copy constructors

			SerialConnection(const SerialConnection&);
			SerialConnection& operator=(const SerialConnection&);

		private:

			// Pimpl idiom, d_pointer
			class SerialConnectionImpl;
			SerialConnectionImpl *pimpl_;

			// Scoped Lock Classes
			class ScopedReadLock;
			class ScopedWriteLock;

			// Generic read function
			size_t read_(uint8_t *buffer, size_t size);
			// Generic write function
			size_t write_(const uint8_t *data, size_t length);
		};


		class SerialConnectionException : public std::exception
		{
			// Disable copy constructors
			SerialConnectionException& operator=(const SerialConnectionException&);
			std::string e_what_;
			
			public:
				SerialConnectionException(const char* description) {
					std::stringstream ss;
					ss << "SerialConnectionException " << description << " failed.";
					e_what_ = ss.str();
				}
				SerialConnectionException(const SerialConnectionException& other) : e_what_(other.e_what_) {}
				virtual ~SerialConnectionException() throw() {}
				virtual const char* what() const throw () {
					return e_what_.c_str();
				}
		};

		class SerialConnectionIOException : public std::exception
		{
			// Disable copy constructors
			SerialConnectionIOException& operator=(const SerialConnectionIOException&);
			std::string file_;
			int line_;
			std::string e_what_;
			int errno_;
		
			public:
			
				explicit SerialConnectionIOException(std::string file, int line, int errnum)
					: file_(file), line_(line), errno_(errnum) {
						
					std::stringstream ss;
					#if defined(_WIN32) && !defined(__MINGW32__)
						char error_str[1024];
						strerror_s(error_str, 1024, errnum);
					#else
						char* error_str = strerror(errnum);
					#endif
				
					ss << "Serial Connection IO Exception (" << errno_ << "): " << error_str;
					ss << ", file " << file_ << ", line " << line_ << ".";
					e_what_ = ss.str();
				}

				explicit SerialConnectionIOException(std::string file, int line, const char* description)
					: file_(file), line_(line), errno_(0) {
				
					std::stringstream ss;
					ss << "Serial Connection IO Exception: " << description;
					ss << ", file " << file_ << ", line " << line_ << ".";
					e_what_ = ss.str();
				}

				virtual ~SerialConnectionIOException() throw() {}
				SerialConnectionIOException(const SerialConnectionIOException& other) : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

				int getErrorNumber() const { return errno_; }

				virtual const char* what() const throw () {
					return e_what_.c_str();
				}
		};

		class SerialConnectionPortNotOpenedException : public std::exception
		{
			// Disable copy constructors
			const SerialConnectionPortNotOpenedException& operator=(SerialConnectionPortNotOpenedException);
			std::string e_what_;
			
			public:
				SerialConnectionPortNotOpenedException(const char* description) {
					std::stringstream ss;
					ss << "SerialConnectionPortNotOpenedException " << description << " failed.";
					e_what_ = ss.str();
				}
				SerialConnectionPortNotOpenedException(const SerialConnectionPortNotOpenedException& other) : e_what_(other.e_what_) {}
					virtual ~SerialConnectionPortNotOpenedException() throw() {}
					virtual const char* what() const throw () {
						return e_what_.c_str();
					}
		};

		/*!
		 * Structure that describes a serial device.
		 */
		struct PortInfo {

			/*! Address of the serial port (this can be passed to the constructor of Serial). */
			std::string port;

			/*! Human readable description of serial device if available. */
			std::string description;

			/*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
			std::string hardware_id;

		};

		/* Lists the serial ports available on the system
		 *
		 * Returns a vector of available serial ports, each represented
		 * by a serial::PortInfo data structure:
		 *
		 * \return vector of Pit221e::Connection::PortInfo.
		 */
		std::vector<PortInfo> list_ports();
	}

#endif
