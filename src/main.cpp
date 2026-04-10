#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string_view>
#include <system_error>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rei/hard/serial/serial.hpp"

namespace {

struct PosixSerialContext {
    int fd{-1};
};

struct PosixSerialDevice {
    int fd{-1};

    PosixSerialDevice() = default;

    ~PosixSerialDevice() {
        if (fd >= 0) {
            (void)close(fd);
        }
    }

    PosixSerialDevice(const PosixSerialDevice&) = delete;
    PosixSerialDevice& operator=(const PosixSerialDevice&) = delete;
};

bool writeByte(void* context, std::uint8_t data) {
    auto* serialContext = static_cast<PosixSerialContext*>(context);
    if ((serialContext == nullptr) || (serialContext->fd < 0)) {
        return false;
    }

    const auto bytesWritten = write(serialContext->fd, &data, 1U);
    if (bytesWritten == 1) {
        return true;
    }

    if ((bytesWritten < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR))) {
        return false;
    }

    return false;
}

bool readByte(void* context, std::uint8_t& data) {
    auto* serialContext = static_cast<PosixSerialContext*>(context);
    if ((serialContext == nullptr) || (serialContext->fd < 0)) {
        return false;
    }

    const auto bytesRead = read(serialContext->fd, &data, 1U);
    if (bytesRead == 1) {
        return true;
    }

    if ((bytesRead < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR))) {
        return false;
    }

    return false;
}

bool configureSerial(int fd, speed_t baudrate) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    if (cfsetispeed(&tty, baudrate) != 0 || cfsetospeed(&tty, baudrate) != 0) {
        return false;
    }

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }

    return tcflush(fd, TCIOFLUSH) == 0;
}

bool printBytes(std::string_view label, const std::uint8_t* data, std::size_t length) {
    std::cout << label;
    for (std::size_t index = 0; index < length; ++index) {
        std::cout << ' ' << static_cast<unsigned int>(data[index]);
    }
    std::cout << '\n';
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    constexpr auto kBaudrate = B115200;
    constexpr auto kPollInterval = std::chrono::milliseconds(10);
    constexpr auto kTimeout = std::chrono::seconds(2);
    const char* devicePath = argc > 1 ? argv[1] : "/dev/serial0";

    PosixSerialDevice serialDevice;
    serialDevice.fd = open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serialDevice.fd < 0) {
        std::cerr << "open serial failed: " << devicePath << ", error=" << std::strerror(errno) << '\n';
        std::cerr << "usage: " << argv[0] << " [serial_device]\n";
        return 1;
    }

    if (!configureSerial(serialDevice.fd, kBaudrate)) {
        std::cerr << "configure serial failed: " << devicePath << ", error=" << std::strerror(errno) << '\n';
        return 1;
    }

    PosixSerialContext context{serialDevice.fd};
    venthmi::SerialPort serialPort;
    const auto initStatus = serialPort.init({writeByte, readByte, &context});
    if (initStatus != venthmi::SerialStatus::kOk) {
        std::cerr << "serial port init failed: " << venthmi::toString(initStatus) << '\n';
        return 1;
    }

    const std::array<std::uint8_t, 8> txData{0x31U, 0x32U, 0x33U, 0x34U, 0xA5U, 0x5AU, 0x0DU, 0x0AU};
    std::array<std::uint8_t, txData.size()> rxData{};

    const auto queuedBytes = serialPort.send(txData);
    if (queuedBytes != txData.size()) {
        std::cerr << "queue tx failed: queued=" << queuedBytes << ", expected=" << txData.size() << '\n';
        return 1;
    }

    const auto startTime = std::chrono::steady_clock::now();
    std::size_t totalReceived = 0U;
    while (totalReceived < txData.size()) {
        const auto processResult = serialPort.process(venthmi::SerialPort::kUnlimited, txData.size());
        if (processResult.stateError) {
            std::cerr << "serial process state error\n";
            return 1;
        }

        totalReceived += serialPort.receive(
            rxData.data() + totalReceived,
            static_cast<std::uint32_t>(rxData.size() - totalReceived));

        if (std::chrono::steady_clock::now() - startTime > kTimeout) {
            break;
        }

        if (totalReceived < txData.size()) {
            std::this_thread::sleep_for(kPollInterval);
        }
    }

    printBytes("tx bytes:", txData.data(), txData.size());
    printBytes("rx bytes:", rxData.data(), totalReceived);

    if (totalReceived != txData.size()) {
        std::cerr << "loopback timeout: received=" << totalReceived << ", expected=" << txData.size() << '\n';
        std::cerr << "check loopback wiring: connect UART TX to UART RX on the Raspberry Pi header.\n";
        return 1;
    }

    if (rxData != txData) {
        std::cerr << "loopback mismatch\n";
        return 1;
    }

    std::cout << "serial loopback test passed on " << devicePath << " at 115200 8N1\n";
    return 0;
}