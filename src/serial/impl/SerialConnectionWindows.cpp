#if defined(_WIN32)

#include <sstream>

#include <serial/impl/SerialConnectionWindows.h>

using std::string;
using std::wstring;
using std::stringstream;
using std::invalid_argument;
using namespace Connection;

inline wstring _prefix_port_if_needed(const wstring& input)
{
    static wstring windows_com_port_prefix = L"\\\\.\\";
    if (input.compare(windows_com_port_prefix) != 0)
    {
        return windows_com_port_prefix + input;
    }
    return input;
}

SerialConnection::SerialConnectionImpl::SerialConnectionImpl(const string& port, uint32_t baudrate,
    bytesize_t bytesize,
    parity_t parity, stopbits_t stopbits,
    flowcontrol_t flowcontrol)
    : port_(port.begin(), port.end()), fd_(INVALID_HANDLE_VALUE), is_open_(false),
    baudrate_(baudrate), parity_(parity),
    bytesize_(bytesize), stopbits_(stopbits), flowcontrol_(flowcontrol)
{
    if (port_.empty() == false)
        Open();
    read_mutex_ = CreateMutex(NULL, false, NULL);
    write_mutex_ = CreateMutex(NULL, false, NULL);
}

SerialConnection::SerialConnectionImpl::~SerialConnectionImpl()
{
    this->Close();
    CloseHandle(read_mutex_);
    CloseHandle(write_mutex_);
}

void SerialConnection::SerialConnectionImpl::Open()
{
    if (port_.empty()) {
        throw invalid_argument("Empty port is invalid.");
    }
    if (is_open_ == true) {
        throw SerialConnectionException("Serial port already open.");
    }

    wstring port_with_prefix = _prefix_port_if_needed(port_);
    LPCWSTR lp_port = port_with_prefix.c_str();
    fd_ = CreateFileW(lp_port,
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0);

    if (fd_ == INVALID_HANDLE_VALUE) {
        DWORD create_file_err = GetLastError();
        stringstream ss;
        switch (create_file_err) {
        case ERROR_FILE_NOT_FOUND:
            // Use this->getPort to convert to a std::string
            ss << "Specified port, " << this->GetPortName() << ", does not exist.";
            THROW(SerialConnectionIOException, ss.str().c_str());
        default:
            ss << "Unknown error opening the serial port: " << create_file_err;
            THROW(SerialConnectionIOException, ss.str().c_str());
        }
    }

    ReconfigurePort();
    is_open_ = true;
}

void SerialConnection::SerialConnectionImpl::ReconfigurePort()
{
    if (fd_ == INVALID_HANDLE_VALUE) {
        // Can only operate on a valid file descriptor
        THROW(SerialConnectionIOException, "Invalid file descriptor, is the serial port open?");
    }

    DCB dcbSerialParams = { 0 };

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(fd_, &dcbSerialParams)) {
        //error getting state
        THROW(SerialConnectionIOException, "Error getting the serial port state.");
    }

    // setup baud rate
    switch (baudrate_) {
        #ifdef CBR_0
            case 0: dcbSerialParams.BaudRate = CBR_0; break;
        #endif
        #ifdef CBR_50
            case 50: dcbSerialParams.BaudRate = CBR_50; break;
        #endif
        #ifdef CBR_75
            case 75: dcbSerialParams.BaudRate = CBR_75; break;
        #endif
        #ifdef CBR_110
            case 110: dcbSerialParams.BaudRate = CBR_110; break;
        #endif
        #ifdef CBR_134
            case 134: dcbSerialParams.BaudRate = CBR_134; break;
        #endif
        #ifdef CBR_150
            case 150: dcbSerialParams.BaudRate = CBR_150; break;
        #endif
        #ifdef CBR_200
            case 200: dcbSerialParams.BaudRate = CBR_200; break;
        #endif
        #ifdef CBR_300
            case 300: dcbSerialParams.BaudRate = CBR_300; break;
        #endif
        #ifdef CBR_600
            case 600: dcbSerialParams.BaudRate = CBR_600; break;
        #endif
        #ifdef CBR_1200
            case 1200: dcbSerialParams.BaudRate = CBR_1200; break;
        #endif
        #ifdef CBR_1800
            case 1800: dcbSerialParams.BaudRate = CBR_1800; break;
        #endif
        #ifdef CBR_2400
            case 2400: dcbSerialParams.BaudRate = CBR_2400; break;
        #endif
        #ifdef CBR_4800
            case 4800: dcbSerialParams.BaudRate = CBR_4800; break;
        #endif
        #ifdef CBR_7200
            case 7200: dcbSerialParams.BaudRate = CBR_7200; break;
        #endif
        #ifdef CBR_9600
            case 9600: dcbSerialParams.BaudRate = CBR_9600; break;
        #endif
        #ifdef CBR_14400
            case 14400: dcbSerialParams.BaudRate = CBR_14400; break;
        #endif
        #ifdef CBR_19200
            case 19200: dcbSerialParams.BaudRate = CBR_19200; break;
        #endif
        #ifdef CBR_28800
            case 28800: dcbSerialParams.BaudRate = CBR_28800; break;
        #endif
        #ifdef CBR_57600
            case 57600: dcbSerialParams.BaudRate = CBR_57600; break;
        #endif
        #ifdef CBR_76800
            case 76800: dcbSerialParams.BaudRate = CBR_76800; break;
        #endif
        #ifdef CBR_38400
            case 38400: dcbSerialParams.BaudRate = CBR_38400; break;
        #endif
        #ifdef CBR_115200
            case 115200: dcbSerialParams.BaudRate = CBR_115200; break;
        #endif
        #ifdef CBR_128000
            case 128000: dcbSerialParams.BaudRate = CBR_128000; break;
        #endif
        #ifdef CBR_153600
            case 153600: dcbSerialParams.BaudRate = CBR_153600; break;
        #endif
        #ifdef CBR_230400
            case 230400: dcbSerialParams.BaudRate = CBR_230400; break;
        #endif
        #ifdef CBR_256000
            case 256000: dcbSerialParams.BaudRate = CBR_256000; break;
        #endif
        #ifdef CBR_460800
            case 460800: dcbSerialParams.BaudRate = CBR_460800; break;
        #endif
        #ifdef CBR_921600
            case 921600: dcbSerialParams.BaudRate = CBR_921600; break;
        #endif
        
        default:
        // Try to blindly assign it
        dcbSerialParams.BaudRate = baudrate_;
    }

    // setup char len
    if (bytesize_ == eightbits)
        dcbSerialParams.ByteSize = 8;
    else if (bytesize_ == sevenbits)
        dcbSerialParams.ByteSize = 7;
    else if (bytesize_ == sixbits)
        dcbSerialParams.ByteSize = 6;
    else if (bytesize_ == fivebits)
        dcbSerialParams.ByteSize = 5;
    else
        throw invalid_argument("invalid char len");

    // setup stopbits
    if (stopbits_ == stopbits_one)
        dcbSerialParams.StopBits = ONESTOPBIT;
    else if (stopbits_ == stopbits_one_point_five)
        dcbSerialParams.StopBits = ONE5STOPBITS;
    else if (stopbits_ == stopbits_two)
        dcbSerialParams.StopBits = TWOSTOPBITS;
    else
        throw invalid_argument("invalid stop bit");

    // setup parity
    if (parity_ == parity_none) {
        dcbSerialParams.Parity = NOPARITY;
    }
    else if (parity_ == parity_even) {
        dcbSerialParams.Parity = EVENPARITY;
    }
    else if (parity_ == parity_odd) {
        dcbSerialParams.Parity = ODDPARITY;
    }
    else if (parity_ == parity_mark) {
        dcbSerialParams.Parity = MARKPARITY;
    }
    else if (parity_ == parity_space) {
        dcbSerialParams.Parity = SPACEPARITY;
    }
    else {
        throw invalid_argument("invalid parity");
    }

    // setup flowcontrol
    if (flowcontrol_ == flowcontrol_none) {
        dcbSerialParams.fOutxCtsFlow = false;
        dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
        dcbSerialParams.fOutX = false;
        dcbSerialParams.fInX = false;
    }
    if (flowcontrol_ == flowcontrol_software) {
        dcbSerialParams.fOutxCtsFlow = false;
        dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
        dcbSerialParams.fOutX = true;
        dcbSerialParams.fInX = true;
    }
    if (flowcontrol_ == flowcontrol_hardware) {
        dcbSerialParams.fOutxCtsFlow = true;
        dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
        dcbSerialParams.fOutX = false;
        dcbSerialParams.fInX = false;
    }

    // activate settings
    if (!SetCommState(fd_, &dcbSerialParams)) {
        CloseHandle(fd_);
        THROW(SerialConnectionIOException, "Error setting serial port settings.");
    }

    // Setup timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = timeout_.inter_byte_timeout;
    timeouts.ReadTotalTimeoutConstant = timeout_.read_timeout_constant;
    timeouts.ReadTotalTimeoutMultiplier = timeout_.read_timeout_multiplier;
    timeouts.WriteTotalTimeoutConstant = timeout_.write_timeout_constant;
    timeouts.WriteTotalTimeoutMultiplier = timeout_.write_timeout_multiplier;
    if (!SetCommTimeouts(fd_, &timeouts)) {
        THROW(SerialConnectionIOException, "Error setting timeouts.");
    }
}

void SerialConnection::SerialConnectionImpl::Close()
{
    if (is_open_ == true) {
        if (fd_ != INVALID_HANDLE_VALUE) {
            int ret;
            ret = CloseHandle(fd_);
            if (ret == 0) {
                stringstream ss;
                ss << "Error while closing serial port: " << GetLastError();
                THROW(SerialConnectionIOException, ss.str().c_str());
            }
            else {
                fd_ = INVALID_HANDLE_VALUE;
            }
        }
        is_open_ = false;
    }
}

bool SerialConnection::SerialConnectionImpl::IsOpen() const
{
    return is_open_;
}

size_t SerialConnection::SerialConnectionImpl::Available()
{
    if (!is_open_) {
        return 0;
    }
    COMSTAT cs;
    if (!ClearCommError(fd_, NULL, &cs)) {
        stringstream ss;
        ss << "Error while checking status of the serial port: " << GetLastError();
        THROW(SerialConnectionIOException, ss.str().c_str());
    }
    return static_cast<size_t>(cs.cbInQue);
}

bool SerialConnection::SerialConnectionImpl::WaitReadable(uint32_t /*timeout*/)
{
    THROW(SerialConnectionIOException, "waitReadable is not implemented on Windows.");
    return false;
}

void SerialConnection::SerialConnectionImpl::WaitByteTimes(size_t /*count*/)
{
    THROW(SerialConnectionIOException, "waitByteTimes is not implemented on Windows.");
}

size_t SerialConnection::SerialConnectionImpl::Read(uint8_t* buf, size_t size)
{
    if (!is_open_) {
        throw SerialConnectionPortNotOpenedException("Serial::read");
    }
    DWORD bytes_read;
    if (!ReadFile(fd_, buf, static_cast<DWORD>(size), &bytes_read, NULL)) {
        stringstream ss;
        ss << "Error while reading from the serial port: " << GetLastError();
        THROW(SerialConnectionIOException, ss.str().c_str());
    }
    return (size_t)(bytes_read);
}

size_t SerialConnection::SerialConnectionImpl::Write(const uint8_t* data, size_t length)
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::write");
    }
    DWORD bytes_written;
    if (!WriteFile(fd_, data, static_cast<DWORD>(length), &bytes_written, NULL)) {
        stringstream ss;
        ss << "Error while writing to the serial port: " << GetLastError();
        THROW(SerialConnectionIOException, ss.str().c_str());
    }
    return (size_t)(bytes_written);
}

void SerialConnection::SerialConnectionImpl::SetPortName(const string& port)
{
    port_ = wstring(port.begin(), port.end());
}

string SerialConnection::SerialConnectionImpl::GetPortName() const
{
    return string(port_.begin(), port_.end());
}

void SerialConnection::SerialConnectionImpl::SetTimeout(Timeout timeout)
{
    timeout_ = timeout;
    if (is_open_) {
        ReconfigurePort();
    }
}

Timeout SerialConnection::SerialConnectionImpl::GetTimeout() const
{
    return timeout_;
}

void SerialConnection::SerialConnectionImpl::SetBaudrate(unsigned long baudrate)
{
    baudrate_ = baudrate;
    if (is_open_) {
        ReconfigurePort();
    }
}

uint32_t SerialConnection::SerialConnectionImpl::GetBaudrate() const
{
    return baudrate_;
}

void SerialConnection::SerialConnectionImpl::SetBytesize(bytesize_t bytesize)
{
    bytesize_ = bytesize;
    if (is_open_) {
        ReconfigurePort();
    }
}

bytesize_t SerialConnection::SerialConnectionImpl::GetBytesize() const
{
    return bytesize_;
}

void SerialConnection::SerialConnectionImpl::SetParity(parity_t parity)
{
    parity_ = parity;
    if (is_open_) {
        ReconfigurePort();
    }
}

parity_t SerialConnection::SerialConnectionImpl::GetParity() const
{
    return parity_;
}

void SerialConnection::SerialConnectionImpl::SetStopbits(stopbits_t stopbits)
{
    stopbits_ = stopbits;
    if (is_open_) {
        ReconfigurePort();
    }
}

stopbits_t SerialConnection::SerialConnectionImpl::GetStopbits() const
{
    return stopbits_;
}

void SerialConnection::SerialConnectionImpl::SetFlowcontrol(flowcontrol_t flowcontrol)
{
    flowcontrol_ = flowcontrol;
    if (is_open_) {
        ReconfigurePort();
    }
}

flowcontrol_t SerialConnection::SerialConnectionImpl::GetFlowcontrol() const
{
    return flowcontrol_;
}

void SerialConnection::SerialConnectionImpl::Flush()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::flush");
    }
    FlushFileBuffers(fd_);
}

void SerialConnection::SerialConnectionImpl::FlushInput()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::flushInput");
    }
    PurgeComm(fd_, PURGE_RXCLEAR);
}

void SerialConnection::SerialConnectionImpl::FlushOutput()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::flushOutput");
    }
    PurgeComm(fd_, PURGE_TXCLEAR);
}

void SerialConnection::SerialConnectionImpl::SendBreak(int /*duration*/)
{
    THROW(SerialConnectionIOException, "sendBreak is not supported on Windows.");
}

void SerialConnection::SerialConnectionImpl::SetBreak(bool level)
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::setBreak");
    }
    if (level) {
        EscapeCommFunction(fd_, SETBREAK);
    }
    else {
        EscapeCommFunction(fd_, CLRBREAK);
    }
}

void SerialConnection::SerialConnectionImpl::SetRTS(bool level)
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::setRTS");
    }
    if (level) {
        EscapeCommFunction(fd_, SETRTS);
    }
    else {
        EscapeCommFunction(fd_, CLRRTS);
    }
}

void SerialConnection::SerialConnectionImpl::SetDTR(bool level)
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::setDTR");
    }
    if (level) {
        EscapeCommFunction(fd_, SETDTR);
    }
    else {
        EscapeCommFunction(fd_, CLRDTR);
    }
}

bool SerialConnection::SerialConnectionImpl::WaitForChange()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::waitForChange");
    }
    DWORD dwCommEvent;

    if (!SetCommMask(fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD)) {
        // Error setting communications mask
        return false;
    }

    if (!WaitCommEvent(fd_, &dwCommEvent, NULL)) {
        // An error occurred waiting for the event.
        return false;
    }
    else {
        // Event has occurred.
        return true;
    }
}

bool SerialConnection::SerialConnectionImpl::GetCTS()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::getCTS");
    }
    DWORD dwModemStatus;
    if (!GetCommModemStatus(fd_, &dwModemStatus)) {
        THROW(SerialConnectionIOException, "Error getting the status of the CTS line.");
    }

    return (MS_CTS_ON & dwModemStatus) != 0;
}

bool SerialConnection::SerialConnectionImpl::GetDSR()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::getDSR");
    }
    DWORD dwModemStatus;
    if (!GetCommModemStatus(fd_, &dwModemStatus)) {
        THROW(SerialConnectionIOException, "Error getting the status of the DSR line.");
    }

    return (MS_DSR_ON & dwModemStatus) != 0;
}

bool SerialConnection::SerialConnectionImpl::GetRI()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::getRI");
    }
    DWORD dwModemStatus;
    if (!GetCommModemStatus(fd_, &dwModemStatus)) {
        THROW(SerialConnectionIOException, "Error getting the status of the RI line.");
    }

    return (MS_RING_ON & dwModemStatus) != 0;
}

bool SerialConnection::SerialConnectionImpl::GetCD()
{
    if (is_open_ == false) {
        throw SerialConnectionPortNotOpenedException("Serial::getCD");
    }
    DWORD dwModemStatus;
    if (!GetCommModemStatus(fd_, &dwModemStatus)) {
        // Error in GetCommModemStatus;
        THROW(SerialConnectionIOException, "Error getting the status of the CD line.");
    }

    return (MS_RLSD_ON & dwModemStatus) != 0;
}

void SerialConnection::SerialConnectionImpl::ReadLock()
{
    if (WaitForSingleObject(read_mutex_, INFINITE) != WAIT_OBJECT_0) {
        THROW(SerialConnectionIOException, "Error claiming read mutex.");
    }
}

void SerialConnection::SerialConnectionImpl::ReadUnlock()
{
    if (!ReleaseMutex(read_mutex_)) {
        THROW(SerialConnectionIOException, "Error releasing read mutex.");
    }
}

void SerialConnection::SerialConnectionImpl::WriteLock()
{
    if (WaitForSingleObject(write_mutex_, INFINITE) != WAIT_OBJECT_0) {
        THROW(SerialConnectionIOException, "Error claiming write mutex.");
    }
}

void SerialConnection::SerialConnectionImpl::WriteUnlock()
{
    if (!ReleaseMutex(write_mutex_)) {
        THROW(SerialConnectionIOException, "Error releasing write mutex.");
    }
}

#endif // #if defined(_WIN32)
