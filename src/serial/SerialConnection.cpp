#include <algorithm>

#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
# include <alloca.h>
#endif

#if defined (__MINGW32__)
# define alloca __builtin_alloca
#endif

#include <serial/SerialConnection.h>

#ifdef _WIN32
#include <serial/impl/SerialConnectionWindows.h>
#else
#include <serial/impl/SerialConnectionUnix.h>
#endif

using std::invalid_argument;
using std::min;
using std::numeric_limits;
using std::vector;
using std::size_t;
using std::string;

using namespace Connection;

class SerialConnection::ScopedReadLock {

	public:
		ScopedReadLock(SerialConnectionImpl* pimpl) : pimpl_(pimpl) {
			this->pimpl_->ReadLock();
		}
		~ScopedReadLock() {
			this->pimpl_->ReadUnlock();
		}
	private:
		// Disable copy constructors
		ScopedReadLock(const ScopedReadLock&);
		const ScopedReadLock& operator=(ScopedReadLock);
	
		SerialConnectionImpl* pimpl_;
};

class SerialConnection::ScopedWriteLock {
public:
	ScopedWriteLock(SerialConnectionImpl* pimpl) : pimpl_(pimpl) {
		this->pimpl_->WriteLock();
	}
	~ScopedWriteLock() {
		this->pimpl_->WriteUnlock();
	}
private:
	// Disable copy constructors
	ScopedWriteLock(const ScopedWriteLock&);
	const ScopedWriteLock& operator=(ScopedWriteLock);
	SerialConnectionImpl* pimpl_;
};

SerialConnection::SerialConnection(const string& port, uint32_t baudrate, Timeout timeout,
	bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
	flowcontrol_t flowcontrol)
	: pimpl_(new SerialConnectionImpl(port, baudrate, bytesize, parity,
		stopbits, flowcontrol))
{
	pimpl_->SetTimeout(timeout);
}


Connection::SerialConnection::~SerialConnection()
{
	delete pimpl_;
}

void SerialConnection::Open()
{
	pimpl_->Open();
}

void SerialConnection::Close()
{
	pimpl_->Close();
}

bool SerialConnection::IsOpen() const
{
	return pimpl_->IsOpen();
}

size_t SerialConnection::Available()
{
	return pimpl_->Available();
}

bool SerialConnection::WaitReadable()
{
	Timeout timeout(pimpl_->GetTimeout());
	return pimpl_->WaitReadable(timeout.read_timeout_constant);
}

void SerialConnection::WaitByteTimes(size_t count)
{
	pimpl_->WaitByteTimes(count);
}

size_t SerialConnection::Read(uint8_t* buffer, size_t size)
{
	return this->pimpl_->Read(buffer, size);
}

size_t SerialConnection::read_(uint8_t* buffer, size_t size)
{
	ScopedReadLock lock(this->pimpl_);
	return this->pimpl_->Read(buffer, size);
}

size_t SerialConnection::Read(std::vector<uint8_t>& buffer, size_t size)
{
	ScopedReadLock lock(this->pimpl_);
	uint8_t* buffer_ = new uint8_t[size];
	size_t bytes_read = 0;

	try {
		bytes_read = this->pimpl_->Read(buffer_, size);
	}
	catch (const std::exception& e) {
		delete[] buffer_;
		throw;
	}

	buffer.insert(buffer.end(), buffer_, buffer_ + bytes_read);
	delete[] buffer_;
	return bytes_read;
}

size_t SerialConnection::Read(std::string& buffer, size_t size)
{
	ScopedReadLock lock(this->pimpl_);
	uint8_t* buffer_ = new uint8_t[size];
	size_t bytes_read = 0;
	try {
		bytes_read = this->pimpl_->Read(buffer_, size);
	}
	catch (const std::exception& e) {
		delete[] buffer_;
		throw;
	}
	buffer.append(reinterpret_cast<const char*>(buffer_), bytes_read);
	delete[] buffer_;
	return bytes_read;
}

string SerialConnection::Read(size_t size)
{
	std::string buffer;
	this->Read(buffer, size);
	return buffer;
}

size_t SerialConnection::Readline(string& buffer, size_t size, string eol)
{
	ScopedReadLock lock(this->pimpl_);
	size_t eol_len = eol.length();
	uint8_t* buffer_ = static_cast<uint8_t*>
		(alloca(size * sizeof(uint8_t)));
	size_t read_so_far = 0;
	while (true)
	{
		size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
		read_so_far += bytes_read;
		if (bytes_read == 0) {
			break; // Timeout occured on reading 1 byte
		}
		if (string(reinterpret_cast<const char*>
			(buffer_ + read_so_far - eol_len), eol_len) == eol) {
			break; // EOL found
		}
		if (read_so_far == size) {
			break; // Reached the maximum read length
		}
	}
	buffer.append(reinterpret_cast<const char*> (buffer_), read_so_far);
	return read_so_far;
}

string SerialConnection::Readline(size_t size, string eol)
{
	std::string buffer;
	this->Readline(buffer, size, eol);
	return buffer;
}

vector<string> SerialConnection::Readlines(size_t size, string eol)
{
	ScopedReadLock lock(this->pimpl_);
	std::vector<std::string> lines;
	size_t eol_len = eol.length();
	uint8_t* buffer_ = static_cast<uint8_t*>
		(alloca(size * sizeof(uint8_t)));
	size_t read_so_far = 0;
	size_t start_of_line = 0;
	while (read_so_far < size) {
		size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
		read_so_far += bytes_read;
		if (bytes_read == 0) {
			if (start_of_line != read_so_far) {
				lines.push_back(
					string(reinterpret_cast<const char*> (buffer_ + start_of_line),
						read_so_far - start_of_line));
			}
			break; // Timeout occured on reading 1 byte
		}
		if (string(reinterpret_cast<const char*>
			(buffer_ + read_so_far - eol_len), eol_len) == eol) {
			// EOL found
			lines.push_back(
				string(reinterpret_cast<const char*> (buffer_ + start_of_line),
					read_so_far - start_of_line));
			start_of_line = read_so_far;
		}
		if (read_so_far == size) {
			if (start_of_line != read_so_far) {
				lines.push_back(
					string(reinterpret_cast<const char*> (buffer_ + start_of_line),
						read_so_far - start_of_line));
			}
			break; // Reached the maximum read length
		}
	}
	return lines;
}

size_t SerialConnection::Write(const string& data)
{
	ScopedWriteLock lock(this->pimpl_);
	return this->write_(reinterpret_cast<const uint8_t*>(data.c_str()),
		data.length());
}

size_t SerialConnection::Write(const std::vector<uint8_t>& data)
{
	ScopedWriteLock lock(this->pimpl_);
	return this->write_(&data[0], data.size());
}

size_t SerialConnection::Write(const uint8_t* data, size_t size)
{
	ScopedWriteLock lock(this->pimpl_);
	return this->write_(data, size);
}

size_t SerialConnection::write_(const uint8_t* data, size_t length)
{
	return pimpl_->Write(data, length);
}

void SerialConnection::SetPortName(const string& port)
{
	ScopedReadLock rlock(this->pimpl_);
	ScopedWriteLock wlock(this->pimpl_);
	bool was_open = pimpl_->IsOpen();
	if (was_open) Close();
	pimpl_->SetPortName(port);
	if (was_open) Open();
}

string SerialConnection::GetPortName() const
{
	return pimpl_->GetPortName();
}

void SerialConnection::SetTimeout(Timeout timeout)
{
	pimpl_->SetTimeout(timeout);
}

Connection::Timeout SerialConnection::GetTimeout() const {
	return pimpl_->GetTimeout();
}

void SerialConnection::SetBaudrate(uint32_t baudrate)
{
	pimpl_->SetBaudrate(baudrate);
}

uint32_t SerialConnection::GetBaudrate() const
{
	return uint32_t(pimpl_->GetBaudrate());
}

void SerialConnection::SetBytesize(bytesize_t bytesize)
{
	pimpl_->SetBytesize(bytesize);
}

bytesize_t SerialConnection::GetBytesize() const
{
	return pimpl_->GetBytesize();
}

void SerialConnection::SetParity(parity_t parity)
{
	pimpl_->SetParity(parity);
}

parity_t SerialConnection::GetParity() const
{
	return pimpl_->GetParity();
}

void SerialConnection::SetStopbits(stopbits_t stopbits)
{
	pimpl_->SetStopbits(stopbits);
}

stopbits_t SerialConnection::GetStopbits() const
{
	return pimpl_->GetStopbits();
}

void SerialConnection::SetFlowcontrol(flowcontrol_t flowcontrol)
{
	pimpl_->SetFlowcontrol(flowcontrol);
}

flowcontrol_t SerialConnection::GetFlowcontrol() const
{
	return pimpl_->GetFlowcontrol();
}

void SerialConnection::Flush()
{
	ScopedReadLock rlock(this->pimpl_);
	ScopedWriteLock wlock(this->pimpl_);
	pimpl_->Flush();
}

void SerialConnection::FlushInput()
{
	ScopedReadLock lock(this->pimpl_);
	pimpl_->FlushInput();
}

void SerialConnection::FlushOutput()
{
	ScopedWriteLock lock(this->pimpl_);
	pimpl_->FlushOutput();
}

void SerialConnection::SendBreak(int duration)
{
	pimpl_->SendBreak(duration);
}

void SerialConnection::SetBreak(bool level)
{
	pimpl_->SetBreak(level);
}

void SerialConnection::SetRTS(bool level)
{
	pimpl_->SetRTS(level);
}

void SerialConnection::SetDTR(bool level)
{
	pimpl_->SetDTR(level);
}

bool SerialConnection::WaitForChange()
{
	return pimpl_->WaitForChange();
}

bool SerialConnection::GetCTS()
{
	return pimpl_->GetCTS();
}

bool SerialConnection::GetDSR()
{
	return pimpl_->GetDSR();
}

bool SerialConnection::GetRI()
{
	return pimpl_->GetRI();
}

bool SerialConnection::GetCD()
{
	return pimpl_->GetCD();
}
