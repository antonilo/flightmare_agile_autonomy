#include <rpgq_common/serial_port/serial_port.h>

// standard library
#include <sys/poll.h>
#include <termios.h>

using namespace RPGQ;

// constructor & destructor
SerialPort::SerialPort(void)
{
    // general variables
    fd_ = -1;
}

SerialPort::~SerialPort()
{
    close(fd_);
}


// public set up & get functions
bool SerialPort::Initialize(const std::string port, const int baud)
{
    // open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); // read/write, ignore specials chars, do not consider DCD line, non-blocking mode 
    ROS_INFO("[%s] Opening serial port %s.", ros::this_node::getName().c_str(), port.c_str());

    if (fd_ == -1)
    {
        ROS_ERROR("[%s] Failed to open serial port %s.", ros::this_node::getName().c_str(), port.c_str());
        return false;
    }

    // clear config
    fcntl(fd_, F_SETFL, 0);
    // read non blocking
    fcntl(fd_, F_SETFL, FNDELAY);

    // create serial port config
    struct termios uartConfig;
    tcgetattr(fd_, &uartConfig);

    // set baud rate
    int bBaud;
    switch (baud)
    {
        case 115200:
        bBaud = B115200;
        break;

        case 230400:
        bBaud = B230400;
        break;

        case 460800:
        bBaud = B460800;
        break;

        case 921600:
        bBaud = B921600;
        break;

        default:
        ROS_ERROR("[%s] Invalid baud rate (%d). Setting default baud rate 115200.", ros::this_node::getName().c_str(), baud);
        bBaud = B115200;
        break;
    }
    if (cfsetispeed(&uartConfig, bBaud) < 0 || cfsetospeed(&uartConfig, bBaud) < 0) // set input/output baudrate
    {
        ROS_ERROR("[%s] Could not set desired baud rate (%d).", ros::this_node::getName().c_str(), baud);
        return false;
    }

    // output flags
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    uartConfig.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    // input flags
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    uartConfig.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // line flags
    // echo off
    // echo newline off
    // canonical mode off,
    // extended input processing off,
    // signal chars off,
    uartConfig.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // 8N1 settings
    // turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    uartConfig.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    uartConfig.c_cflag |= CS8;

    // apply configuration
    if (tcsetattr(fd_, TCSAFLUSH, &uartConfig) < 0)
    {
        ROS_ERROR("[%s] Could not set serial port configuration.", ros::this_node::getName().c_str());
        return false;
    }
    tcflush(fd_, TCOFLUSH);
    
    return true;
}


// public read/write functions
unsigned int SerialPort::Write(const void* buffer, const unsigned int length)
{
    unsigned int lengthWritten = write(fd_, buffer, length);
    if (lengthWritten != length)
    {
        ROS_ERROR("[%s] Could only write %d of %d bytes.", ros::this_node::getName().c_str(), lengthWritten, (int) length);
    }

    return lengthWritten;
}

unsigned int SerialPort::Read(void* buffer, const unsigned int bufferLength)
{
    return read(fd_, buffer, bufferLength);
}
