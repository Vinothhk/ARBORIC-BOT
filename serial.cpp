#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

int main() {
    int serial_port = open("/dev/ttyACM0", O_WRONLY | O_NOCTTY);

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

    while (true) {
        std::string command;
        std::cout << "Enter Command (1 1, -1 -1, 1 0, 0 1, 0 0): ";
        std::getline(std::cin, command);

        command += "\n";  // Ensure newline for Arduino
        write(serial_port, command.c_str(), command.length()); 
        std::cout << "Sent: " << command;
        usleep(100000);  // Small delay to avoid overwhelming Arduino
    }

    close(serial_port);
    return 0;
}
