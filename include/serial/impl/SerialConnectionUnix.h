#if !defined(_WIN32)

#ifndef SERIAL_CONNECTION_UNIX_HPP
#define SERIAL_CONNNECTION_UNIX_HPP

#include "serial/SerialConnection.h"

#include <pthread.h>

    namespace Connection {

        using std::size_t;
        using std::string;
        using std::invalid_argument;

        using Pit221e::Connection::SerialConnectionException;
        using Pit221e::Connection::SerialConnectionIOException;

        class MillisecondTimer
        {
        public:
            MillisecondTimer(const uint32_t millis);
            int64_t remaining();

        private:
            static timespec timespec_now();
            timespec expiry;
        };

        class SerialConnection::SerialConnectionImpl
        {
        private:
            std::string port_;               // Path to the file descriptor
            int fd_;                    // The current file descriptor

            bool is_open_;
            bool xonxoff_;
            bool rtscts_;

            Timeout timeout_;           // Timeout for read operations
            uint32_t baudrate_;         // Baudrate
            uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

            parity_t parity_;           // Parity
            bytesize_t bytesize_;       // Size of the bytes
            stopbits_t stopbits_;       // Stop Bits
            flowcontrol_t flowcontrol_; // Flow Control

            // Mutex used to lock the read functions
            pthread_mutex_t read_mutex;
            // Mutex used to lock the write functions
            pthread_mutex_t write_mutex;

        protected:
            void ReconfigurePort();

        public:
            SerialConnectionImpl(const std::string& port,
                uint32_t baudrate,
                Pit221e::Connection::bytesize_t bytesize,
                Pit221e::Connection::parity_t parity,
                Pit221e::Connection::stopbits_t stopbits,
                Pit221e::Connection::flowcontrol_t flowcontrol);

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

            Pit221e::Connection::Timeout GetTimeout() const;

            void SetBaudrate(uint32_t baudrate);

            uint32_t GetBaudrate() const;

            void SetBytesize(bytesize_t bytesize);

            Pit221e::Connection::bytesize_t GetBytesize() const;

            void SetParity(parity_t parity);

            Pit221e::Connection::parity_t GetParity() const;

            void SetStopbits(stopbits_t stopbits);

            Pit221e::Connection::stopbits_t GetStopbits() const;

            void SetFlowcontrol(flowcontrol_t flowcontrol);

            Pit221e::Connection::flowcontrol_t GetFlowcontrol() const;

            void ReadLock();

            void ReadUnlock();

            void WriteLock();

            void WriteUnlock();
        };
    }

#endif

#endif // !defined(_WIN32)
