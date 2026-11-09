#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unistd.h>

#include "api.h"
#include "pros/apix.h"

// Byte transports between the brain and the host computer. Both expose the
// same two calls so the bridge loop doesn't care which one is in use:
//   size_t read_available(uint8_t* buf, size_t max);  // never blocks
//   void   write_all(const uint8_t* buf, size_t len);
namespace bridge {

// USB user serial (the brain's second CDC port, usually /dev/ttyACM1 on Linux).
// PROS COBS-encodes stdout by default for its terminal; that is turned off so
// the host sees raw packets. stdin reads block in PROS, so a background task
// drains it into a queue that read_available() pulls from.
class UsbTransport {
  public:
    UsbTransport() : reader_([this] { reader_loop(); }, "usb_rx") {
        pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
    }

    size_t read_available(uint8_t* buf, size_t max) {
        std::lock_guard<pros::Mutex> lock(mutex_);
        size_t n = 0;
        while (n < max && !queue_.empty()) {
            buf[n++] = queue_.front();
            queue_.pop_front();
        }
        return n;
    }

    void write_all(const uint8_t* buf, size_t len) {
        while (len > 0) {
            ssize_t written = write(STDOUT_FILENO, buf, len);
            if (written <= 0) return;
            buf += written;
            len -= static_cast<size_t>(written);
        }
    }

  private:
    // Bounds memory if the bridge loop stalls; oldest bytes are dropped first.
    static constexpr size_t MAX_QUEUED = 4096;

    void reader_loop() {
        while (true) {
            uint8_t c;
            // One byte at a time: a larger read can block until it is full.
            if (read(STDIN_FILENO, &c, 1) == 1) {
                std::lock_guard<pros::Mutex> lock(mutex_);
                if (queue_.size() >= MAX_QUEUED) queue_.pop_front();
                queue_.push_back(c);
            } else {
                pros::delay(1);
            }
        }
    }

    pros::Mutex mutex_;
    std::deque<uint8_t> queue_;
    pros::Task reader_; // declared last so it starts after mutex_/queue_ exist
};

// Smart-port generic serial through the comms board (Jetson /dev/ttyTHS1).
// The original wiring used separate ports for each direction.
class SmartPortTransport {
  public:
    SmartPortTransport(uint8_t rx_port, uint8_t tx_port, int32_t baud)
        : rx_(rx_port, baud), tx_(tx_port, baud) {}

    size_t read_available(uint8_t* buf, size_t max) {
        int32_t avail = rx_.get_read_avail();
        if (avail <= 0 || avail == PROS_ERR) return 0;
        int32_t n = rx_.read(buf, static_cast<int32_t>(avail < static_cast<int32_t>(max) ? avail : max));
        return n > 0 && n != PROS_ERR ? static_cast<size_t>(n) : 0;
    }

    void write_all(const uint8_t* buf, size_t len) {
        while (len > 0) {
            int32_t written = tx_.write(const_cast<uint8_t*>(buf), static_cast<int32_t>(len));
            if (written == PROS_ERR) return;
            if (written == 0) {
                pros::delay(1); // TX buffer full
                continue;
            }
            buf += written;
            len -= static_cast<size_t>(written);
        }
    }

  private:
    pros::Serial rx_;
    pros::Serial tx_;
};

} // namespace bridge
