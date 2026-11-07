#pragma once

#include <string>
#include <iostream>
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

// OS-agnostic port definition
enum class Port: uint8_t {
    COM3
};


class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int serialFd;
#endif
    bool connected;

constexpr const char* getPortName(Port port) {
    switch (port) {
        case Port::COM3:
#ifdef _WIN32
            return "COM3";
#elif __APPLE__
            return "/dev/cu.usbserial-0001";
#else
            return "/dev/ttyUSB0";
#endif
        
        default:
            // Note: throwing exceptions in constexpr depends on your C++ standard 
            // (fully supported in C++20). Alternatively, return a fallback or "UNKNOWN".
            return "UNKNOWN";
    }
}

public:
    SerialPort(const Port port, int baudRate = 115200) : connected(false) {
        const char* portName = this->getPortName(port);
#ifdef _WIN32
        hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Failed to open port: " << portName << std::endl;
            return;
        }
        DCB dcb = { 0 };
        dcb.DCBlength = sizeof(dcb);
        GetCommState(hSerial, &dcb);
        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        SetCommState(hSerial, &dcb);
        
        COMMTIMEOUTS timeouts = { 50, 50, 50, 10, 10 };
        SetCommTimeouts(hSerial, &timeouts);
#else
        serialFd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serialFd == -1) {
            std::cerr << "Failed to open port: " << portName << std::endl;
            return;
        }
        struct termios options;
        tcgetattr(serialFd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        tcsetattr(serialFd, TCSANOW, &options);
#endif
        connected = true;
        std::cout << "Connected to serial device on " << portName << std::endl;
    }

    ~SerialPort() {
        if (connected) {
#ifdef _WIN32
            CloseHandle(hSerial);
#else
            close(serialFd);
#endif
        }
    }

    void writeData(const std::string& data) {
        if (!connected) return;
#ifdef _WIN32
        DWORD bytesWritten;
        WriteFile(hSerial, data.c_str(), data.length(), &bytesWritten, NULL);
#else
        write(serialFd, data.c_str(), data.length());
#endif
    }

    std::string readData() {
        if (!connected) return "";
        char buffer[128];
#ifdef _WIN32
        DWORD bytesRead;
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
            buffer[bytesRead] = '\0';
            return std::string(buffer);
        }
#else
        int bytesRead = read(serialFd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            return std::string(buffer);
        }
#endif
        return "";
    }

    bool isConnected() const { return connected; }
};