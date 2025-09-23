#pragma once
#include <Arduino.h>
#include <STM32SD.h>

namespace finware {

class SDCard {
public:
    SDCard() = default;

    bool begin() {  mounted_ = SD.begin(); return mounted_;  }

    bool open(const char* path) {
        if (!mounted_ && !begin()) return false;
        close(); 
        file_ = SD.open(path, FILE_WRITE);
        return (opened_ = (bool)file_);
    }

    size_t write(const void* data, size_t len) {
        if (!opened_) return 0;
        return file_.write(reinterpret_cast<const uint8_t*>(data), len);
    }

    void close() {
        if (opened_) {  file_.close(); opened_ = false;  };
    }

    size_t print(const char* s) { return write(s, strlen(s)); }
    size_t println(const char* s) { 
        size_t n = print(s); 
        n += print("\r\n"); 
        return n; 
    }

    bool ok() const { return mounted_ && opened_; }

    private:
    bool mounted_ = false;
    bool opened_ = false;
    File file_;
};

} // namespace finwar