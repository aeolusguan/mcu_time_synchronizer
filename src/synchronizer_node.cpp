//
// Created by GUAN Tongfan on 12/4/22.
//
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include "spinnaker_sdk_camera_driver/SpinnakerImageNames.h"
#include "mcu_time_synchronizer/mcu_timestamp.h"

int getch(void) {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

class Synchronizer {
  using scoped_m = std::unique_lock<std::mutex>;

  std::vector<double> tm_cam_utc_;
  std::vector<double> tm_cam_hw_;

  ros::NodeHandle nh_pvt_;
  ros::Subscriber cam_subscriber_object_;
  ros::Publisher cam_pub_object_;
  curi::TimestampReceiver recv_;
  std::shared_ptr<std::thread> worker_;
  std::mutex mutex_;
  std::atomic_bool done_;

 public:
  Synchronizer()
      : tm_cam_hw_{},
        tm_cam_utc_{},
        nh_pvt_{"~"},
        done_{false},
        mutex_{} {
    std::string cam_topic_name;
    ROS_FATAL_COND(!nh_pvt_.getParam("cam_tm_topic_name", cam_topic_name),
                   "\"cam_tm_topic_name\" is not specified");
    cam_subscriber_object_ = nh_pvt_.subscribe(cam_topic_name,
                                               1000,
                                               &Synchronizer::camTimestampCallback,
                                               this);
    std::string cam_offset_topic_name;
    ROS_FATAL_COND(!nh_pvt_.getParam("cam_offset_topic_name",
                                     cam_offset_topic_name),
                   "\"cam_offset_topic_name\" is not specified");
    cam_pub_object_ =
        nh_pvt_.advertise<std_msgs::Float64>(cam_offset_topic_name, 1000);
    std::string vendor_id, product_id;
    ROS_FATAL_COND(!nh_pvt_.getParam("vendor_id", vendor_id),
                   "\"vendor_id\" of usb device is not specified");
    ROS_FATAL_COND(!nh_pvt_.getParam("product_id", product_id),
                   "\"product_id\" of usb device is not specified");
    recv_.reset((uint16_t) stoi(vendor_id, nullptr, 16),
                (uint16_t) stoi(product_id, nullptr, 16),
                false);
    worker_ = std::make_shared<std::thread>([this] { run(); });
  }

  ~Synchronizer() {
    done_ = true;
    if (worker_->joinable())
      worker_->join();
  }

 private:
  void add_utc_time(double tm) { tm_cam_utc_.push_back(tm); }

  void clear() {
    tm_cam_utc_.clear();
    scoped_m lk;
    tm_cam_hw_.clear();
  }

  void camTimestampCallback(const spinnaker_sdk_camera_driver::SpinnakerImageNames &msg) {
    double tm = msg.header.stamp.toSec() - msg.time.toSec();
    scoped_m lk;
    tm_cam_hw_.push_back(tm);
  }

  bool Execute(double &offset) {
    using namespace boost::accumulators;
    if (tm_cam_utc_.size() < 5) {
      ROS_WARN("number of UTC timestamp is not enough, at least 5 is required!");
      return false;
    }
    scoped_m lk;
    if (tm_cam_utc_.size() != tm_cam_hw_.size()) {
      ROS_WARN("number of timestamps mismatch: %lu (utc) vs %lu (hw)",
               tm_cam_utc_.size(), tm_cam_hw_.size());
      return false;
    }

    ROS_INFO("offset list:");
    accumulator_set<double, stats<tag::variance(lazy)>> acc;
    for (int i = 0; i < tm_cam_utc_.size(); ++i) {
      double tmp = tm_cam_utc_[i] - tm_cam_hw_[i];
      acc(tmp);
      printf("\t%.6lf\n", tmp);
    }
    double std_dev = sqrt(variance(acc)) * 1e6;
    ROS_INFO(
        "standard deviation: %lf us, please press 'y' or 'Y' if you accept "
        "this result, otherwise to calibrate again",
        std_dev);
    int ch = getch();
    if ((ch == 'Y') || (ch == 'y')) {
      offset = tm_cam_utc_.back() - tm_cam_hw_.back();
      return true;
    }
    return false;
  }

  bool sync(double offset_old, double &offset_new) {
    scoped_m lk;
    double latest_utc = tm_cam_utc_.back();
    if (tm_cam_hw_.empty()) return false;
    int idx = tm_cam_hw_.size();
    for (auto it = tm_cam_hw_.rbegin(); it != tm_cam_hw_.rend(); ++it) {
      if (abs(*it + offset_old - latest_utc) < 1e-3) {
        offset_new = latest_utc - *it;
        tm_cam_hw_.erase(tm_cam_hw_.begin(), tm_cam_hw_.begin() + idx);
        tm_cam_utc_.clear();
        return true;
      }
      idx--;
    }

    double latest_hw = tm_cam_hw_.back();
    idx = tm_cam_utc_.size();
    for (auto it = tm_cam_utc_.rbegin(); it != tm_cam_utc_.rend(); ++it) {
      if (abs(*it - latest_hw - offset_old) < 1e-3) {
        offset_new = *it - latest_hw;
        tm_cam_utc_.erase(tm_cam_utc_.begin(), tm_cam_utc_.begin() + idx);
        tm_cam_hw_.clear();
        return true;
      }
      idx--;
    }

    return false;
  }

  void run() {
    bool ok = false;
    curi::usb_utc_rx_data_t timestamp;
    time_t unix_secs;
    int nano_secs;
    double offset = 0.;
    while (!ok) {
      clear();
      ROS_INFO(
          "Please press the user key to calibrate clock offset between MCU and camera. "
          "You are suggested to press the keys 5-10 times to trigger the camera. "
          "Note: the time interval between press action must not be larger than 15 seconds, "
          "otherwise the key press process are regarded as finished and begin to calculate offset.");
      bool start = false;
      bool done = false;
      while (!done) {
        bool received = recv_.retrieveTimestamp(timestamp, start, 15000);
        if (received) {
          start = true;
          if (curi::mcu_sync(&timestamp)) {
            curi::parse_mcu_time(&timestamp, &unix_secs, &nano_secs);
            double tm = unix_secs + ((double) nano_secs) / 1e9;
            add_utc_time(tm);
          } else {
            add_utc_time(timestamp.st_time);
          }
        }
        done = !received;
      }

      // calculate the clock offset between mcu and camera
      ok = Execute(offset);
    }

    ROS_INFO("Finish clock offset initial calibration");
    clear();

    ros::Time last_offset_time = ros::Time::now();
    std_msgs::Float64 offset_pub;
    double offset_new = offset;
    while (!done_) {
      if ((ros::Time::now() - last_offset_time).toSec() > 60) {
        last_offset_time = ros::Time::now();
        ROS_INFO("Camera-MCU clock offset: %.6lf", offset);
      }

      // receive new timestamp from MCU
      if (recv_.retrieveTimestamp(timestamp, true, 1000)) {
        if (curi::mcu_sync(&timestamp)) {
          curi::parse_mcu_time(&timestamp, &unix_secs, &nano_secs);
          double tm = unix_secs + ((double) nano_secs) / 1e9;
          add_utc_time(tm);
        } else {
          add_utc_time(timestamp.st_time);
        }
        if (sync(offset, offset_new)) {
          offset = offset_new;
          offset_pub.data = offset;
          cam_pub_object_.publish(offset_pub);
        }
      }
    }
  }
};

int main(int argc, char **argv) {
  // Initializing the ros node
  ros::init(argc, argv, "mcu_synchronizer_node");

  Synchronizer sync;

  ros::spin();
  return 0;
}