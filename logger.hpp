#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <ArduinoJson.h>
#include <PicoMQTT.h>

#define LOG_BUFFER_SIZE 4

class Logger {
    public:
      Logger();
      Logger(Stream *serial, PicoMQTT::Server *mqtt, String topic);
      void setSerial(Stream *serial);
      void setMqtt(PicoMQTT::Server *mqtt, String topic);
      void log(String msg);
      String get(int index);
      String get();
      String last();
      String next();
      String tail();
      uint8_t currentIndex();

    private :
        CircularBuffer<String, LOG_BUFFER_SIZE> log_buffer;
        Stream *serial;
        PicoMQTT::Server *mqtt;
        String topic;
        uint8_t current_index;
};

#endif // LOGGER_HPP