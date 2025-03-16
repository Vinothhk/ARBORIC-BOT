#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

int main() {
    int serial_port = open("/dev/ttyACM0", O_WRONLY);

    if (serial_port < 0) {
        std::cerr << "Error opening serial port!" << std::endl;
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_port, TCSANOW, &tty);

    const char* data = "A";  // Data to send
    while (true)
    {
        /* code */
        write(serial_port, data, 1);
        sleep(1);
    }
    
   

    close(serial_port);
    return 0;
}
