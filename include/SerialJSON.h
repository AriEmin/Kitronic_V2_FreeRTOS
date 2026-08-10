// include/SerialJSON.h
#pragma once

// JSON-only mod aç/kapat: platformio.ini -> -DSERIAL_JSON_ONLY
#ifndef KEEP_REAL_SERIAL
  #ifdef SERIAL_JSON_ONLY
    // Basit / güvenli Null Serial
    struct NullSerial {
      template<typename... Args> void print(Args...) {}
      template<typename... Args> void println(Args...) {}
      void printf(const char*, ...) {}
      int available() { return 0; }
      int read() { return -1; }
      size_t readBytesUntil(char, char*, size_t) { return 0; }
      void setTimeout(uint32_t) {}
    };
    static NullSerial __NullSerial__;
    #define Serial __NullSerial__
  #endif
#endif
