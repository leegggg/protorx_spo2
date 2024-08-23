#include "logger.hpp"

Logger::Logger() {
    this->serial = NULL;
    this->mqtt = NULL;
}

Logger::Logger(Stream *serial, PicoMQTT::Server *mqtt, String topic) {
    this->serial = serial;
    this->mqtt = mqtt;
    this->topic = topic;
}

void Logger::setSerial(Stream *serial) { this->serial = serial; }

void Logger::setMqtt(PicoMQTT::Server *mqtt, String topic) {
    this->mqtt = mqtt;
    this->topic = topic;
}

String Logger::get() { 
    this->current_index = this->current_index % LOG_BUFFER_SIZE; 
    return this->log_buffer[this->current_index]; 
}

String Logger::next() {
    this->current_index++;
    return get();
}

String Logger::last() {
    this->current_index--;
    return get();
}

String Logger::tail() {
    this->current_index = this->log_buffer.size() - 1;
    return get();
}

u8_t Logger::currentIndex() { return this->current_index % LOG_BUFFER_SIZE; }

String Logger::get(int index) {
    this->current_index = index % LOG_BUFFER_SIZE;
    return get();
}

void Logger::log(String msg) {
    this->log_buffer.push(String(millis()) + ": \n" + msg);
    JsonDocument doc;
    doc["ts"] = millis();
    doc["log"] = msg;
    String res;
    serializeJson(doc, res);

    if (serial) {
        serial->println(res);
    }

    if (mqtt) {
        mqtt->publish(this->topic, res);
    }
}