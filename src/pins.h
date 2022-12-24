#ifndef PINS_H
#define PINS_H

constexpr int PIN_INT = 39;
constexpr int PIN_SDA = 40;
constexpr int PIN_SCL = 41;

constexpr int PIN_TXD = 1;
constexpr int PIN_RXD = 2;

constexpr int UART_NUM = 1;
constexpr int UART_BAUD = 115200;
constexpr int UART_RX_BUFFER_LEN = 256;
constexpr int UART_RX_PAT_CHR = 0xA5;
constexpr int UART_RX_PAT_COUNT = 4;

#endif
