#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <paths.h>
#include <sysexits.h>
#include <termios.h>
#include <sys/param.h>
#include <pthread.h>

# include <linux/serial.h>

#include <sys/select.h>
#include <sys/time.h>
#include <time.h>

#include "serial_linux.hpp"

using std::string;
using std::stringstream;
using std::invalid_argument;
using serial::MillisecondTimer;
using serial::Serial;
using serial::SerialException;
using serial::PortNotOpenedException;
using serial::IOException;

MillisecondTimer::MillisecondTimer(const uint32_t millis) : expiry(timespec_now())
{
    int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);

    if (tv_nsec >= 1e9) {
        int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
        expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
        expiry.tv_sec += sec_diff;
    } else {
        expiry.tv_nsec = tv_nsec;
    }
}

int64_t MillisecondTimer::remaining()
{
    timespec now(timespec_now());
    int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;

    millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;

    return millis;
}

timespec MillisecondTimer::timespec_now()
{
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time;
}

timespec timespec_from_ms(const uint32_t millis)
{
    timespec time;
    time.tv_sec = millis / 1e3;
    time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

/* Serial Implementation */
Serial::SerialImpl::SerialImpl(
    const string& port, unsigned long baudrate, bytesize_t bytesize, 
    parity_t parity, stopbits_t stopbits, flowcontrol_t flowcontrol)
    : port_(port), fd_(-1), is_open_(false), xonxoff_(false), rtscts_(false),
    baudrate_(baudrate), parity_(parity), bytesize_(bytesize), stopbits_(stopbits),
    flowcontrol_(flowcontrol) 
    {
        pthread_mutex_init(&this->read_mutex, NULL);
        pthread_mutex_init(&this->write_mutex, NULL);

        if (port_.empty() == false) { open(); }
    }

Serial::SerialImpl::~SerialImpl()
{
    close();
    pthread_mutex_destroy(&this->read_mutex);
    pthread_mutex_destroy(&this->write_mutex);
}

void Serial::SerialImpl::open()
{
    if (port_.empty()) { throw invalid_argument("Empty port is invalid."); }
    if (is_open_ == true) { throw SerialException("Serial port already open."); }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ == -1) {
        switch (errno) {
        case EINTR:
            open();
            return;
        case ENFILE:
        case EMFILE:
            THROW(IOException, "Too many file handles open.");
        default:
            THROW(IOException, errno);
        }
    }

    reconfigurePort();
    is_open_ = true;
}

void Serial::SerialImpl::reconfigurePort()
{
    if (fd_ == -1) { THROW(IOException, "Invalid file descriptor, is the serial port open?"); }

    struct termios options; // options for the file descriptor

    if (tcgetattr(fd_, &options) == -1) { THROW(IOException, "::tcgetattr"); }

    options.c_cflag |= (tcflag_t) (CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);

    options.c_oflag &= (tcflag_t) ~IUCLC;
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

#ifdef IUCLC
    options.c_iflag &= (tcflag_t) ~IUCLC;
#endif

#ifdef PARMRK
    options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

    bool custom_baud = false;
    speed_t baud;

    switch (baudrate_) {
#ifdef B9600
    case 9600: 
        baud = B9600; 
        break;
#endif
#ifdef B115200
    case 115200:
        baud = B115200;
        break;
#endif
#ifdef B921600
    case 921600:
        baud = B921600;
        break;
#endif
    default:
        custom_baud = true;
        struct serial_struct ser;
        if (-1 == ioctl(fd_, TIOCGSERIAL, &ser)) { THROW(IOException, errno); }

        ser.custom_divisor = ser.baud_base / static_cast<int>(baudrate_);
        ser.flags &= ~ASYNC_SPD_MASK;
        ser.flags |= ASYNC_SPD_CUST;

        if (-1 == ioctl(fd_, TIOCSSERIAL, &ser)) { THROW(IOException, errno); }
    }

    if (custom_baud == false) {
#ifdef _BSD_SOURCE
        ::cfsetspeed(&options, baud);
#else
        ::cfsetispeed(&options, baud);
        ::cfsetospeed(&options, baud);
#endif
    }

    options.c_cflag &= (tcflag_t) ~CSIZE;

    /* Character length */
    if (bytesize_ == eightbits) { options.c_cflag |= CS8; }
    else if (bytesize_ == sevenbits) { options.c_cflag |= CS7; }
    else if (bytesize_ == sixbits) { options.c_cflag |= CS6; }
    else if (bytesize_ == fivebits) { options.c_cflag |= CS5; }
    else { throw invalid_argument("invalid char length"); }

    /* Stopbits */
    if (stopbits_ == stopbits_one) { options.c_cflag &= (tcflag_t) ~(CSTOPB); }
    else if (stopbits_ == stopbits_one_point_five) { options.c_cflag |= (CSTOPB); }
    else if (stopbits_ == stopbits_two) { options.c_cflag |= (CSTOPB); }
    else { throw invalid_argument("invalid stop bit"); }

    /* Parity bit */
    options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
    if (parity_ == parity_none) {
        options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
    } else if (parity_ == parity_even) {
        options.c_cflag &= (tcflag_t) ~(PARODD);
        options.c_cflag |= (PARENB);
    } else if (parity_ == parity_odd) {
        options.c_cflag |= (PARENB | PARODD);
    }
#ifdef CMSPAR
    else if (parity_ == parity_mark) {
        options.c_cflag |= (PARENB | CMSPAR | PARODD);
    } else if (parity_ == parity_space) {
        options.c_cflag |= (PARENB | CMSPAR);
        options.c_cflag &= (tcflag_t) ~(PARODD);
    }
#else
    else if (parity_ == parity_mark || parity_ == parity_space) {
        throw invalid_argument("OS does not support mark or space parity");
    }
#endif
    else {
        throw invalid_argument("invalid parity");
    }

    /* Flow control */
    if (flowcontrol_ == flowcontrol_none) {
        xonxoff_ = false;
        rtscts_ = false;
    }
    if (flowcontrol_ == flowcontrol_sw) {
        xonxoff_ = true;
        rtscts_ = false;
    }
    if (flowcontrol_ == flowcontrol_hw) {
        xonxoff_ = false;
        rtscts_ = true;
    }

    /* xonxoff */
#ifdef IXANY
    if (xonxoff_) { options.c_iflag |= (IXON | IXOFF); }
    else { options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY); }
#else
    if (xonxoff_) { options.c_iflag |= (IXON | IXOFF); }
    else { options.c_iflag &= (tcflag_t) ~(IXON | IXOFF); }
#endif

    /* RTSCTS */
#ifdef CRTSCTS
    if (rtscts_) { options.c_cflag |= (CRTSCTS); }
    else { options.c_cflag &= (unsigned long) ~(CRTSCTS); }
#elif defined CNEW_RTSCTS
    if (rtscts_) { options.c_cflag |= (CNEW_RTSCTS); }
    else { options.c_clfag &= (unsigned long) ~(CNEW_RTSCTS); }
#else
#error "OS Support seems wrong."
#endif

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    /* activate setting */
    ::tcsetattr(fd_, TCSANOW, &options);

    uint32_t bit_time_ns = 1e9 / baudrate_;
    byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

    if (stopbits_ == stopbits_one_point_five) {
        byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
    }
}

void Serial::SerialImpl::close()
{
    if (is_open_ == true) {
        if (fd_ != -1) {
            int ret;
            ret = ::close(fd_);
            if (ret == 0) { fd_ = -1; }
            else { THROW(IOException, errno); }
        }
        is_open_ = false;
    }
}

bool Serial::SerialImpl::isOpen() const { return is_open_; }

size_t Serial::SerialImpl::available()
{
    if (!is_open_) { return 0; }
    int count = 0;

    if (-1 == ioctl(fd_, TIOCINQ, &count)) { THROW(IOException, errno); }
    else { return static_cast<size_t>(count); }
}

bool Serial::SerialImpl::waitReadable(uint32_t timeout)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    timespec timeout_ts (timespec_from_ms(timeout));
    int r = pselect(fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

    if (r < 0) {
        if (errno == EINTR) { return false; }
        THROW(IOException, errno);
    }

    if (r == 0) { return false; }

    if (!FD_ISSET(fd_, &readfds)) {
        THROW(IOException, "Select reports ready to read, but our fd isn't.");
    }

    return true;
}

void Serial::SerialImpl::waitByteTimes(size_t count)
{
    timespec wait_time = { 0, static_cast<long>(byte_time_ns_ * count) };
    pselect(0, NULL, NULL, NULL, &wait_time, NULL);
}

size_t Serial::SerialImpl::read(uint8_t* buf, size_t size)
{
    if (!is_open_) { throw PortNotOpenedException("Serial::read"); }
    size_t bytes_read = 0;

    long total_timeout_ms = timeout_.read_timeout_constant;
    total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long>(size);
    MillisecondTimer total_timeout(total_timeout_ms);

    {
        ssize_t current_bytes = ::read(fd_, buf, size);
        if (current_bytes > 0) { bytes_read = current_bytes; }
    }

    while (bytes_read < size) {
        int64_t timeout_remaining_ms = total_timeout.remaining();
        if (timeout_remaining_ms <= 0) { break; }

        uint32_t timeout = std::min(static_cast<uint32_t>(timeout_remaining_ms), timeout_.inter_byte_timeout);

        if (waitReadable(timeout)) {
            if (size > 1 && timeout_.inter_byte_timeout == Timeout::max()) {
                size_t bytes_available = available();
                if (bytes_available + bytes_read < size) {
                    waitByteTimes(size - (bytes_available + bytes_read));
                }
            }

            ssize_t current_bytes = ::read(fd_, buf + bytes_read, size - bytes_read);
            if (current_bytes < 1) {
                throw SerialException("Device reports readiness to read but returned no data");
            }

            bytes_read += static_cast<size_t>(current_bytes);
            if (bytes_read == size) { break; }
            if (bytes_read < size) { continue; }
            if (bytes_read > size) { throw SerialException("read overhead, too many bytes wher read."); }
        }
    }

    return bytes_read;
}

size_t Serial::SerialImpl::write(const uint8_t* data, size_t length)
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::write"); }

    fd_set writefds;
    size_t bytes_written = 0;

    long total_timeout_ms = timeout_.write_timeout_constant;
    total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long>(length);
    MillisecondTimer total_timeout(total_timeout_ms);

    bool first_iteration = true;
    while (bytes_written < length) {
        int64_t timeout_remaining_ms = total_timeout.remaining();

        if (!first_iteration && (timeout_remaining_ms <= 0)) { break; }
        first_iteration = false;

        timespec timeout(timespec_from_ms(timeout_remaining_ms));

        FD_ZERO(&writefds);
        FD_SET(fd_, &writefds);

        int r = pselect(fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

        if (r < 0) {
            if (errno == EINTR) { continue; }
            THROW(IOException, errno);
        }

        if (r == 0) { break; }

        /* Port ready to write */
        if (r > 0) {
            if (FD_ISSET(fd_, &writefds)) {
                ssize_t current_bytes = ::write(fd_, data + bytes_written, length - bytes_written);
                if (current_bytes < 1) {
                    throw SerialException("Device reports readiness to write but returned no data.");
                }

                bytes_written += static_cast<size_t>(current_bytes);

                if (bytes_written == length) { break; }
                if (bytes_written < length) { continue; }
                if (bytes_written > length) { throw SerialException("Write overwrote."); }
            }

            THROW(IOException, "Select reports ready to write, but our fd isn't.");
        }
    }

    return bytes_written;
}

void Serial::SerialImpl::setPort(const string& port) { port_ = port; }
string Serial::SerialImpl::getPort() const { return port_; }

void Serial::SerialImpl::setTimeout(serial::Timeout& timeout) { timeout_ = timeout; }
serial::Timeout Serial::SerialImpl::getTimeout() const { return timeout_; }

void Serial::SerialImpl::setBaudrate(unsigned long baudrate) {
    baudrate_ = baudrate;
    if (is_open_) { reconfigurePort(); }
}
unsigned long Serial::SerialImpl::getBaudrate() const { return baudrate_; }

void Serial::SerialImpl::setBytesize(serial::bytesize_t bytesize)
{
    bytesize_ = bytesize;
    if (is_open_) { reconfigurePort(); }
}
serial::bytesize_t Serial::SerialImpl::getBytesize() const { return bytesize_; }

void Serial::SerialImpl::setParity(serial::parity_t parity)
{
    parity_ = parity;
    if (is_open_) { reconfigurePort(); }
}
serial::parity_t Serial::SerialImpl::getParity() const { return parity_; }

void Serial::SerialImpl::setStopbits (serial::stopbits_t stopbits)
{
    stopbits_ = stopbits;
    if (is_open_) { reconfigurePort (); }
}
serial::stopbits_t Serial::SerialImpl::getStopbits () const { return stopbits_; }

void Serial::SerialImpl::setFlowcontrol(serial::flowcontrol_t flowcontrol)
{
    flowcontrol_ = flowcontrol;
    if (is_open_) { reconfigurePort(); }
}
serial::flowcontrol_t Serial::SerialImpl::getFlowcontrol() const { return flowcontrol_; }

void Serial::SerialImpl::flush()
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::flush"); }
    tcdrain(fd_);
}

void Serial::SerialImpl::flushInput()
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::flushInput"); }
    tcflush(fd_, TCIFLUSH);
}

void Serial::SerialImpl::flushOutput()
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::flushOutput"); }
    tcflush(fd_, TCOFLUSH);
}

void Serial::SerialImpl::sendBreak(int duration)
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::sendBreak"); }
    tcsendbreak(fd_, static_cast<int>(duration / 4));
}

void Serial::SerialImpl::setBreak(bool level)
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::setBreak"); }
    if (level) {
        if (-1 == ioctl(fd_, TIOCSBRK)) {
            stringstream ss;
            ss << "setBreak failed on a call to ioctl(TIOCSBRK): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    } else {
        if (-1 == ioctl(fd_, TIOCCBRK)) {
            stringstream ss;
            ss << "setBreak failed on a call to ioctl(TIOCCBRK): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    }
}

void Serial::SerialImpl::setRTS(bool level)
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::setRTS"); }
    int command = TIOCM_RTS;

    if (level) {
        if (-1 == ioctl(fd_, TIOCMBIS, &command)) {
            stringstream ss;
            ss << "setRTS failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    } else {
        if (-1 == ioctl(fd_, TIOCMBIC, &command)) {
            stringstream ss;
            ss << "setRTS failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    }
}

void Serial::SerialImpl::setDTR(bool level)
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::setDTR"); }
    int command = TIOCM_DTR;

    if (level) {
        if (-1 == ioctl(fd_, TIOCMBIS, &command)) {
            stringstream ss;
            ss << "setDTR failed on call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    } else {
        if (-1 == ioctl(fd_, TIOCMBIC, &command))
        {
            stringstream ss;
            ss << "setDTR failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
            throw(SerialException(ss.str().c_str()));
        }
    }
}

bool Serial::SerialImpl::waitForChange()
{
    int command = (TIOCM_CD | TIOCM_DSR | TIOCM_RI | TIOCM_CTS);
    if (-1 == ioctl(fd_, TIOCMIWAIT, &command)) {
        stringstream ss;
        ss << "waitForDSR failed on a call to ioctl(TIOCMIWAIT): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    }
    return true;
}

bool Serial::SerialImpl::getCTS()
{
    if (is_open_ == false) { throw PortNotOpenedException("Serial::getCTS"); }
    int status;

    if (-1 == ioctl(fd_, TIOCMGET, &status)) {
        stringstream ss;
        ss << "getCTS failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    } else {
        return 0 != (status & TIOCM_CTS);
    }
}

bool Serial::SerialImpl::getDSR ()
{
    if (is_open_ == false) { throw PortNotOpenedException ("Serial::getDSR"); }
    int status;

    if (-1 == ioctl (fd_, TIOCMGET, &status)) {
        stringstream ss;
        ss << "getDSR failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    } else {
        return 0 != (status & TIOCM_DSR);
    }
}

bool Serial::SerialImpl::getRI () { 
    if (is_open_ == false) { throw PortNotOpenedException ("Serial::getRI"); }
    int status;

    if (-1 == ioctl (fd_, TIOCMGET, &status)) {
        stringstream ss;
        ss << "getRI failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    } else {
        return 0 != (status & TIOCM_RI);
    }
}

bool Serial::SerialImpl::getCD ()
{ 
    if (is_open_ == false) { throw PortNotOpenedException ("Serial::getCD"); }
    int status;
    if (-1 == ioctl (fd_, TIOCMGET, &status)) {
        stringstream ss;
        ss << "getCD failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    } else {
        return 0 != (status & TIOCM_CD);
    }
}

void Serial::SerialImpl::readLock() 
{
    int result = pthread_mutex_lock(&this->read_mutex);
    if (result) { THROW(IOException, result); }    
}

void Serial::SerialImpl::readUnlock()
{
    int result = pthread_mutex_unlock(&this->read_mutex);
    if (result) { THROW(IOException, result); }
}

void Serial::SerialImpl::writeLock()
{
    int result = pthread_mutex_lock(&this->write_mutex);
    if (result) { THROW(IOException, result); }
}

void Serial::SerialImpl::writeUnlock()
{
    int result = pthread_mutex_unlock(&this->write_mutex);
    if (result) { THROW(IOException, result); }
}