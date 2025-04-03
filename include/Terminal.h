#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>

namespace TerminalColor {
    enum Color { Black = 30, Red = 31, Green = 32, Yellow = 33, Blue = 34, Magenta = 35, Cyan = 36, White = 37 };
}

class TerminalClass {
public:
    void begin(Stream& stream) {
        _stream = &stream;
    }

    void setTextColor(TerminalColor::Color color) {
        _stream->printf("\033[%dm", color);
    }

    void print(const char* text) {
        _stream->print(text);
    }

    void println(const char* text) {
        _stream->println(text);
    }

    void printf(const char* format, ...) {
        char    buf[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);
        _stream->print(buf);
    }

private:
    Stream* _stream;
};

extern TerminalClass Terminal;

#endif  // TERMINAL_H