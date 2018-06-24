#ifndef USBSERIAL_H
#define	USBSERIAL_H

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <unistd.h>  
#include <fcntl.h>   
#include <termios.h> 

class USBSerial {
public:
    
    USBSerial();
    virtual ~USBSerial();
  
    int openUSBPort(std::string devicePath, int baud);
    void sendData(char data[]);
    std::string readData();
    void closeUSBPort();

private:

    struct termios ioStruct;
    int usbFileDescriptor;
    char serialDataIn[200];
    char dataOut[16];

};

#endif	/* USBSERIAL_H */

