#include "sound_module/SoundPlayer.hpp"
#include <string.h>
#include <unistd.h>

namespace DY {
    Player::Player(char * uart_device ) {
        this -> port = uart_device;
    }

    int Player::begin() {
	UART_filestream = open(port, O_RDWR | O_NOCTTY);
        if (UART_filestream < 0) {
            return 1;
        } 
        //This block setups up the uart_filestream object for UART comm.
        tcgetattr(UART_filestream, &UART_options);
        UART_options.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        UART_options.c_iflag = IGNPAR;
        UART_options.c_oflag = 0;
        UART_options.c_lflag = 0;
        UART_options.c_cc[VTIME] = 0;  // Disable timeout
        UART_options.c_cc[VMIN] = 1;   // Read at least 1 character before returning
        tcflush(UART_filestream, TCIFLUSH);
        tcsetattr(UART_filestream, TCSANOW, &UART_options);

        return 0; //successfully setup UART comm.
    }

    void Player::serialWrite(uint8_t * tx_buffer, uint8_t tx_len) {
        write(UART_filestream, tx_buffer,tx_len);
    }

    bool Player::serialRead(uint8_t * rx_buffer, uint8_t rx_len) 
    {
        int rx_count = 0;
        rx_count = read(UART_filestream, rx_buffer, rx_len);

        if (rx_count > 0) 
        {
            return true;
        } 
        else 
        {
            return false;
        }
    }

    void Player::close_connection() {
        close(UART_filestream);
    }
}
