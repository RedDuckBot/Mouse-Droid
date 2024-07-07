#include "DYPlayer.hpp"
#include <iostream>
#include <fcntl.h>  
#include <termios.h>

#define BAUDRATE B9600

namespace DY {
    class Player: public DYPlayer
    {
        public:
            char * port;  
            struct termios UART_options;
            int UART_filestream = -1;

            Player(char * uart_device);
            int begin();
            void serialWrite(uint8_t *buffer, uint8_t len);
            bool serialRead(uint8_t * buffer, uint8_t len);
            void close_connection();
    };

}
