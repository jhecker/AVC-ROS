#include "USBSerial.h"

USBSerial::USBSerial() {

}

USBSerial::~USBSerial() {
    closeUSBPort();
}

int USBSerial::openUSBPort(std::string devicePath, int baud) {
    memset(&ioStruct, 0, sizeof (ioStruct));
    ioStruct.c_iflag = 0;
    ioStruct.c_oflag = 0;
    ioStruct.c_cflag = CS8 | CREAD | CLOCAL;
    ioStruct.c_lflag = 0;
    ioStruct.c_cc[VMIN] = 1;
    ioStruct.c_cc[VTIME] = 5;

    usbFileDescriptor = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (usbFileDescriptor > 0) {
        cfsetospeed(&ioStruct, B115200);
        cfsetispeed(&ioStruct, B115200);
        tcsetattr(usbFileDescriptor, TCSANOW, &ioStruct);
    }
    return usbFileDescriptor;
}

void USBSerial::sendData(char data[]) {
    sprintf(dataOut, "%s", data);
    write(usbFileDescriptor, dataOut, sizeof (dataOut));
    memset(&dataOut, '\0', sizeof (dataOut));
}

std::string USBSerial::readData() {
    if (read(usbFileDescriptor, &serialDataIn, sizeof (serialDataIn)) > 0) {
        read(usbFileDescriptor, &serialDataIn, sizeof (serialDataIn));
    }
    std::string str(serialDataIn);
    tcflush(usbFileDescriptor, TCIOFLUSH);
    memset(&serialDataIn, '\0', sizeof (serialDataIn));
    return str;
}

void USBSerial::closeUSBPort() {
    close(usbFileDescriptor);
}
