#ifndef SERIAL_H_
#define SERIAL_H_

#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <exception>
#include <stdexcept>

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, __LINE__, (message))

namespace serial {
    typedef enum {
        fivebits = 5,
        sixbits = 6,
        sevenbits = 7,
        eightbits = 8
    } bytesize_t;

    typedef enum {
        parity_none = 0,
        parity_odd = 1,
        parity_even = 2,
        parity_mark = 3,
        parity_space = 4
    } parity_t;

    typedef enum {
        stopbits_one = 1,
        stopbits_two = 2,
        stopbits_one_point_five
    } stopbits_t;

    typedef enum {
        flowcontrol_none = 0,
        flowcontrol_sw,
        flowcontrol_hw
    } flowcontrol_t;

    struct Timeout {
    #ifdef max
    #undef max
    #endif
        static uint32_t max() { return std::numeric_limits<uint32_t>::max(); }
        static Timeout simpleTimeout(uint32_t timeout) { return Timeout(max(), timeout, 0, timeout, 0); }
        
        uint32_t inter_byte_timeout;
        uint32_t read_timeout_constant;
        uint32_t read_timeout_multiplier;
        uint32_t write_timeout_constant;
        uint32_t write_timeout_multiplier;

        explicit Timeout(
            uint32_t inter_byte_timeout_ = 0,
            uint32_t read_timeout_constant_ = 0,
            uint32_t read_timeout_multiplier_ = 0,
            uint32_t write_timeout_constant_ = 0,
            uint32_t write_timeout_multiplier_ = 0): 
            inter_byte_timeout(inter_byte_timeout_),
            read_timeout_constant(read_timeout_constant_),
            read_timeout_multiplier(read_timeout_multiplier_),
            write_timeout_constant(write_timeout_constant_),
            write_timeout_multiplier(write_timeout_multiplier_) {}
    };

    class Serial {
    public:
        Serial(
            const std::string &port = "",
            uint32_t baudrate = 9600,
            Timeout timeout = Timeout(),
            bytesize_t bytesize = eightbits,
            parity_t parity = parity_none,
            stopbits_t stopbits = stopbits_one,
            flowcontrol_t flowcontrol = flowcontrol_none);
        virtual ~Serial();

        void open();
        bool isOpen() const;
        void close();
        size_t available();
        bool waitReadable();
        void waitByteTimes(size_t count);

        size_t read(uint8_t* buffer, size_t size);
        size_t read(std::vector<uint8_t>& buffer, size_t size = 1);
        size_t read(std::string &buffer, size_t size = 1);
        std::string read(size_t size = 1);

        size_t readline(std::string& buffer, size_t size = 65536, std::string eol = "\n");
        std::string readline (size_t size = 65536, std::string eol = "\n");
        std::vector<std::string> readlines (size_t size = 65536, std::string eol = "\n");

        size_t write (const uint8_t* data, size_t size);
        size_t write (const std::vector<uint8_t>& data);
        size_t write (const std::string& data);

        void setPort(const std::string& port);
        std::string getPort() const;

        void setTimeout(Timeout& timeout);
        void setTimeout(
            int32_t inter_byte_timeout, 
            uint32_t read_timeout_constant,
            uint32_t read_timeout_multiplier,
            uint32_t write_timeout_constant,
            uint32_t write_timeout_multiplier) {
                Timeout timeout(
                    inter_byte_timeout, read_timeout_constant, read_timeout_multiplier,
                    write_timeout_constant, write_timeout_multiplier);

                return setTimeout(timeout);
            }

        Timeout getTimeout() const;
        void setBaudrate(uint32_t baudrate);
        uint32_t getBaudrate() const;
        void setBytesize(bytesize_t bytesize);
        bytesize_t getBytesize() const;
        void setParity(parity_t parity);
        parity_t getParity() const;
        void setStopbits(stopbits_t stopbits);
        stopbits_t getStopbits() const;
        void setFlowcontrol(flowcontrol_t flowcontrol);
        flowcontrol_t getFlowcontrol() const;

        void flush();
        void flushInput();
        void flushOutput();

        void sendBreak(int duration);
        void setBreak(bool level = true);
        void setRTS(bool level = true);
        void setDTR(bool level = true);

        bool waitForChange();
        bool getCTS();
        bool getDSR();
        bool getRI();
        bool getCD();

    private:
        Serial(const Serial&);
        Serial& operator=(const Serial&);
        class SerialImpl;
        SerialImpl* pimpl_;

        class ScopedReadLock;
        class ScopedWriteLock;

        size_t read_(uint8_t* buffer, size_t size);
        size_t write_(const uint8_t* data, size_t length);
    };

    class SerialException: public std::exception
    {
        SerialException& operator=(const SerialException&);
        std::string e_what_;

    public:
        SerialException(const char* description) {
            std::stringstream ss;
            ss << "SerialException " << description << " failed.";
            e_what_ = ss.str();
        }

        SerialException(const SerialException& other): e_what_(other.e_what_) {}
        virtual ~SerialException() throw() {}
        virtual const char* what() const throw() { return e_what_.c_str(); }
    };

    class IOException: public std::exception
    {
    public:
        explicit IOException(std::string file, int line, int errnum)
            : file_(file), line_(line), errno_(errnum) {
                std::stringstream ss;
                char* error_str = strerror(errnum);
                ss << "IO Exception (" << errno_ << "): " << error_str;
                ss << ", file " << file_ << ", line " << line_ << ".";
                e_what_ = ss.str(); 
            }
        
        explicit IOException(std::string file, int line, const char* description)
            : file_(file), line_(line), errno_(0) {
                std::stringstream ss;
                ss << "IO Exception: " << description;
                ss << ", file " << file_ << ", line " << line_ << ".";
                e_what_ = ss.str();
            }
        
        virtual ~IOException() throw() {}
        IOException(const IOException& other)
            : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}
        
        int getErrorNumber() const { return errno_; }
        virtual const char* what() const throw() { return e_what_.c_str(); }

    protected:
        IOException& operator=(const IOException&);
        std::string file_;
        int line_;
        std::string e_what_;
        int errno_;
    };

    class PortNotOpenedException: public std::exception
    {        
    public:
        PortNotOpenedException(const char* description) {
            std::stringstream ss;
            ss << "PortNotOpenedException " << description << " failed.";
            e_what_ = ss.str();
        }
        PortNotOpenedException(const PortNotOpenedException& other): e_what_(other.e_what_) {}
        virtual ~PortNotOpenedException() throw() {}
        virtual const char* what() const throw() { return e_what_.c_str(); }

    protected:
        const PortNotOpenedException& operator=(PortNotOpenedException);
        std::string e_what_;
    };

    struct PortInfo
    {
        std::string port;
        std::string description;
        std::string hardware_id;
    };

    std::vector<PortInfo> list_ports();
} /* namespace serial */

#endif /* SERIAL_H_ */