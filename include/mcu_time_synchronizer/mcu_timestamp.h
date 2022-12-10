//
// Created by GUAN Tongfan on 2/12/2022.
//

#ifndef COMMUNICATION__MCU_TIMESTAMP_H
#define COMMUNICATION__MCU_TIMESTAMP_H

#include "usb_com.h"
#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <ctime>

namespace curi {

void _cbf_wrapper(struct libusb_transfer *_transfer) {
  auto *temp =
      reinterpret_cast<USB_COM_UTC *>(_transfer->user_data);
  temp->USB_In_CBF(_transfer);
}

// Receive timestamps of device external triggers
class TimestampReceiver {
  using scoped_m = std::unique_lock<std::mutex>;

  std::shared_ptr<USB_COM_UTC> usb_dev_;
  bool verbose_;

  // thread and locking variables
  std::mutex mutex_;
  std::condition_variable cond_;
  std::shared_ptr<std::thread> worker_;

  // use to terminate the receiving thread
  std::atomic_bool done_;

  std::queue<usb_utc_rx_data_t> timestamps_;

 public:
  TimestampReceiver()
      : usb_dev_{},
        worker_{},
        timestamps_{},
        mutex_{},
        cond_{},
        done_{false},
        verbose_{false} {}

  TimestampReceiver(const uint16_t vendor_id,
                    const uint16_t product_id,
                    bool verbose) :
      usb_dev_{},
      verbose_{verbose},
      timestamps_{},
      mutex_{},
      cond_{},
      worker_{},
      done_{false} {
    usb_dev_ = std::make_shared<USB_COM_UTC>(
        USB_RX_WORDS_PER_MESSAGE,
        4, Vendor_id_Hex(vendor_id),
        Product_id_Hex(product_id),
        endpoint_1, 0x00);

    scoped_m lock(mutex_);
    worker_ = std::make_shared<std::thread>([this] { run(); });
  }

  ~TimestampReceiver() {
    done_ = true;
    if (worker_ && worker_->joinable())
      worker_->join();
  }

  // prohibit copy
  TimestampReceiver(const TimestampReceiver&) = delete;
  TimestampReceiver& operator=(const TimestampReceiver&) = delete;

  /// Not thread safe
  void reset(const uint16_t vendor_id,
             const uint16_t product_id,
             bool verbose) {
    if (worker_ && worker_->joinable()) {
      done_ = true;
      worker_->join();
    }

    usb_dev_ = std::make_shared<USB_COM_UTC>(
        USB_RX_WORDS_PER_MESSAGE,
        4, Vendor_id_Hex(vendor_id),
        Product_id_Hex(product_id),
        endpoint_1, 0x00);
    verbose_ = verbose;
    done_ = false;
    worker_ = std::make_shared<std::thread>([this] { run(); });
  }

  bool retrieveTimestamp(usb_utc_rx_data_t &t, bool start, size_t duration_ms) {
    scoped_m lock(mutex_);
    if (start)
      cond_.wait_for(lock, std::chrono::milliseconds(duration_ms), [this] {
        return !timestamps_.empty();
      });
    else
      cond_.wait(lock, [this] { return !timestamps_.empty(); });
    if (timestamps_.empty()) return false;
    t = timestamps_.front();
    timestamps_.pop();
    return true;
  }

 public:
  static const unsigned char endpoint_1 = 0x81;
  static const int USB_RX_WORDS_PER_MESSAGE = 20;
  static const int MAX_QUEUE_SIZE = 1000;

 private:
  void run() {
    // start transmit
    usb_dev_->USB_Com_Start_Trans_Asy(_cbf_wrapper);

    if (verbose_)
      usb_dev_->turn_on_print_data();

    // cache address
    const usb_utc_rx_data_t *utc_ptr = usb_dev_->get_usb_received_data();
    usb_utc_rx_data_t last_utc = *utc_ptr;
    usb_utc_rx_data_t cur_utc;
    bool done = false;
    // timeout for usb handler pending
    struct timeval tv{};
    tv.tv_sec = 1;

    while (!done) {
      libusb_handle_events_timeout(usb_dev_->get_usb_ctx(), &tv);

      usb_dev_->lock_mutex_rx();
      // try-catch block to avoid usb receiver deadlock
      try {
        cur_utc = *utc_ptr;
      }
      catch (...) {
        usb_dev_->unlock_mutex_rx();
        throw;
      }
      usb_dev_->unlock_mutex_rx();

      if ((cur_utc.date_utc != last_utc.date_utc) ||
          (cur_utc.real_utc != last_utc.real_utc) ||
          (cur_utc.st_time != last_utc.st_time)) {
        last_utc = cur_utc;

        scoped_m lock(mutex_);
        if (timestamps_.size() == MAX_QUEUE_SIZE)
          timestamps_.pop();
        timestamps_.push(cur_utc);
        lock.unlock();
        cond_.notify_all();
      }
      done = done_;
    }
  }
};

const unsigned char TimestampReceiver::endpoint_1;
const int TimestampReceiver::USB_RX_WORDS_PER_MESSAGE;
const int TimestampReceiver::MAX_QUEUE_SIZE;

/// Convert a NMEA RMC date and time to UNIX epoch time.
void parse_mcu_time(const usb_utc_rx_data_t *data,
                    std::time_t *unix_secs,
                    int *nano_secs) {
  int date = data->date_utc; // NMEA UTC date, formatted as DDMMYY
  int time_point = data->real_utc; // NMEA UTC time, formatted as HHMMSS
  double extra = data->st_time; // in seconds

  int year = date % 10 + date / 10 % 10 * 10;
  date /= 100;
  int mon = date % 10 + date / 10 % 10 * 10;
  date /= 100;
  int day = date % 10 + date / 10 % 10 * 10;

  int sec = time_point % 10 + time_point / 10 % 10 * 10;
  time_point /= 100;
  int min = time_point % 10 + time_point / 10 % 10 * 10;
  time_point /= 100;
  int hour = time_point % 10 + time_point / 10 % 10 * 10;
  printf("hour: %d\n", hour);

  /// Resolve the ambiguity of century
  /// example 1: utc_year = 99, pc_year = 2100
  /// years = 2100 + int((2100 % 100 - 99) / 50.0) = 2099
  /// example 2: utc_year = 00, pc_year = 2099
  /// years = 2099 + int((2099 % 100 - 00) / 50.0) = 2100
  std::time_t now = std::time(nullptr);
  std::tm *gmtm = gmtime(&now);  // convert now to tm struct for UTC
  int pc_year = gmtm->tm_year + 1900;
  year = pc_year + int((pc_year % 100 - year) / 50.0);

  std::tm tm{};
  tm.tm_year = year - 1900;
  tm.tm_mon = mon - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = min;
  tm.tm_sec = sec;
  tm.tm_isdst = 0;
#if defined(_WIN32)
  std::time_t t = _mkgmtime(&tm);
#else
  std::time_t t = timegm(&tm);
#endif
  auto from = std::chrono::system_clock::from_time_t(t);
  from += std::chrono::seconds(int(extra));
  *unix_secs = std::chrono::system_clock::to_time_t(from);
  *nano_secs = int((extra - int(extra)) * 1e9);
}

inline bool mcu_sync(const usb_utc_rx_data_t *utc_timestamp) {
  return (utc_timestamp->real_utc != 0) || (utc_timestamp->date_utc != 0);
}

} // namespace curi

#endif //COMMUNICATION__MCU_TIMESTAMP_H
