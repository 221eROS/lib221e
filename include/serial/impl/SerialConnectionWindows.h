#if defined(_WIN32)

#ifndef SERIAL_CONNECTION_WINDOWS_HPP
#define SERIAL_CONNECTION_WINDOWS_HPP

#include <serial/SerialConnection.h>

#include "Windows.h"

	namespace Connection {

		using std::string;
		using std::wstring;
		using std::invalid_argument;

		using Connection::SerialConnectionException;
		using Connection::SerialConnectionIOException;

		class SerialConnection::SerialConnectionImpl
		{
		private:
			std::wstring port_;         // Path to the file descriptor
			HANDLE fd_;

			bool is_open_;

			Timeout timeout_;           // Timeout for read operations
			uint32_t baudrate_;			// Baudrate

			parity_t parity_;           // Parity
			bytesize_t bytesize_;       // Size of the bytes
			stopbits_t stopbits_;       // Stop Bits
			flowcontrol_t flowcontrol_; // Flow Control
		
			HANDLE read_mutex_;			// Mutex used to lock the read functions
			HANDLE write_mutex_;		// Mutex used to lock the write functions

		protected:
			void ReconfigurePort();

		public:
			SerialConnectionImpl(const std::string& port,
				uint32_t baudrate,
				bytesize_t bytesize,
				parity_t parity,
				stopbits_t stopbits,
				flowcontrol_t flowcontrol);

			virtual ~SerialConnectionImpl();

			void Open();

			void Close();

			bool IsOpen() const;

			size_t Available();

			bool WaitReadable(uint32_t timeout);

			void WaitByteTimes(size_t count);

			size_t Read(uint8_t* buf, size_t size = 1);

			size_t Write(const uint8_t* data, size_t length);

			void Flush();

			void FlushInput();

			void FlushOutput();

			void SendBreak(int duration);

			void SetBreak(bool level);

			void SetRTS(bool level);

			void SetDTR(bool level);

			bool WaitForChange();

			bool GetCTS();

			bool GetDSR();

			bool GetRI();

			bool GetCD();

			void SetPortName(const std::string& port);

			std::string GetPortName() const;

			void SetTimeout(Timeout timeout);

			Timeout GetTimeout() const;

			void SetBaudrate(unsigned long baudrate);

			uint32_t GetBaudrate() const;

			void SetBytesize(bytesize_t bytesize);

			bytesize_t GetBytesize() const;

			void SetParity(parity_t parity);

			parity_t GetParity() const;

			void SetStopbits(stopbits_t stopbits);

			stopbits_t GetStopbits() const;

			void SetFlowcontrol(flowcontrol_t flowcontrol);

			flowcontrol_t GetFlowcontrol() const;

			// Scoped Lock Read/Write methods
			void ReadLock();

			void ReadUnlock();

			void WriteLock();

			void WriteUnlock();
		};
	}

#endif

#endif
