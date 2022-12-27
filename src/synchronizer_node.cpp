//
// Created by GUAN Tongfan on 12/4/22.
//
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
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

  std::vector<double> tm_utc_;
  std::vector<double> tm_hw_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_pvt_;
  std::string imu_topic_name_;
  std::string cam_topic_name_;
  ros::Subscriber cam_subscriber_object_;
  ros::Subscriber imu_subscriber_object_;
  ros::Publisher cam_pub_object_;
  ros::Publisher imu_pub_object_;
  curi::TimestampReceiver recv_;
  std::shared_ptr<std::thread> worker_;
  std::shared_ptr<std::thread> key_worker_;
  std::mutex mutex_;
  std::atomic_bool done_;
  std::atomic_bool key_done_;
  std::mutex key_mutex_;
  std::condition_variable key_cond_;
  bool key_press_;
  int key_;

 public:
  Synchronizer()
      : tm_hw_{},
        tm_utc_{},
        nh_{},
        nh_pvt_{"~"},
        done_{false},
        key_done_{false},
        mutex_{},
        key_mutex_{},
        key_cond_{},
        key_press_{false},
        cam_topic_name_{"camera"},
        imu_topic_name_{"imu/data"} {
    ROS_FATAL_COND(!nh_pvt_.getParam("cam_tm_topic_name", cam_topic_name_),
                   "\"cam_tm_topic_name\" is not specified");
    ROS_FATAL_COND(!nh_pvt_.getParam("imu_tm_topic_name", imu_topic_name_),
                   "\"imu_tm_topic_name\" is not specified");

    std::string cam_offset_topic_name;
    ROS_FATAL_COND(!nh_pvt_.getParam("cam_offset_topic_name",
                                     cam_offset_topic_name),
                   "\"cam_offset_topic_name\" is not specified");
    cam_pub_object_ =
        nh_.advertise<std_msgs::Float64>(cam_offset_topic_name, 1000);
    std::string imu_offset_topic_name;
    ROS_FATAL_COND(!nh_pvt_.getParam("imu_offset_topic_name",
                                     imu_offset_topic_name),
                   "\"imu_offset_topic_name\" is not specified");
    imu_pub_object_ =
        nh_.advertise<std_msgs::Float64>(imu_offset_topic_name, 1000);
    std::string vendor_id, product_id;
    ROS_FATAL_COND(!nh_pvt_.getParam("vendor_id", vendor_id),
                   "\"vendor_id\" of usb device is not specified");
    ROS_FATAL_COND(!nh_pvt_.getParam("product_id", product_id),
                   "\"product_id\" of usb device is not specified");
    recv_.reset((uint16_t) stoi(vendor_id, nullptr, 16),
                (uint16_t) stoi(product_id, nullptr, 16),
                false);
    key_worker_ = std::make_shared<std::thread>([this] { keyLoop(); });
    worker_ = std::make_shared<std::thread>([this] { run(); });
  }

  ~Synchronizer() {
    key_done_ = true;
    if (key_worker_->joinable())
      key_worker_->join();
    done_ = true;
    if (worker_->joinable())
      worker_->join();
  }

 private:
  void add_utc_time(double tm) { tm_utc_.push_back(tm); }

  void clear() {
    tm_utc_.clear();
    scoped_m lk(mutex_);
    tm_hw_.clear();
  }

  void camTimestampCallback(const spinnaker_sdk_camera_driver::SpinnakerImageNames &msg) {
    double tm = msg.header.stamp.toSec() - msg.time.toSec();
    scoped_m lk(mutex_);
    tm_hw_.push_back(tm);
  }

  void imuTimestampCallback(const sensor_msgs::Imu::ConstPtr &imu_data) {
    /// 3.19 ms: time difference between trigger and first inertial data available
    double tm = imu_data->header.stamp.toSec() - 0.00319;
    scoped_m lk(mutex_);
    bool valid = tm_hw_.empty();
    if (!valid)
      valid = (tm - tm_hw_.back()) > 0.015;
    if (valid)
      tm_hw_.push_back(tm);
  }

  bool Execute(double &offset) {
    using namespace boost::accumulators;
    if (tm_utc_.size() < 5) {
      ROS_WARN("number of UTC timestamp is not enough, at least 5 is required!");
      return false;
    }
    scoped_m lk(mutex_);
    if (tm_utc_.size() != tm_hw_.size()) {
      ROS_WARN("number of timestamps mismatch: %lu (utc) vs %lu (hw)",
               tm_utc_.size(), tm_hw_.size());
      return false;
    }

    ROS_INFO("offset list:");
    accumulator_set<double, stats<tag::variance(lazy)>> acc;
    for (int i = 0; i < tm_utc_.size(); ++i) {
      double tmp = tm_utc_[i] - tm_hw_[i];
      acc(tmp);
      printf("\t%.6lf\n", tmp);
    }
    double std_dev = sqrt(variance(acc));
    ROS_INFO(
        "standard deviation: %.6f s, please press 'y' or 'Y' if you accept "
        "this result, otherwise (except 'C', 'c', 'I', 'i') to calibrate again",
        std_dev);
    scoped_m lk2(key_mutex_);
    key_cond_.wait(lk2, [this] { return key_press_; });
    key_press_ = false;
    if ((key_ == 'Y') || (key_ == 'y')) {
      offset = tm_utc_.back() - tm_hw_.back();
      return true;
    }
    return false;
  }

  bool sync(double offset_old, double &offset_new) {
    scoped_m lk;
    if (tm_hw_.empty() || tm_utc_.empty()) return false;

    double latest_utc = tm_utc_.back();
    int idx = tm_hw_.size();
    for (auto it = tm_hw_.rbegin(); it != tm_hw_.rend(); ++it) {
      if (abs(*it + offset_old - latest_utc) < 1e-3) {
        offset_new = latest_utc - *it;
        tm_hw_.erase(tm_hw_.begin(), tm_hw_.begin() + idx);
        tm_utc_.clear();
        return true;
      }
      idx--;
    }

    double latest_hw = tm_hw_.back();
    idx = tm_utc_.size();
    for (auto it = tm_utc_.rbegin(); it != tm_utc_.rend(); ++it) {
      if (abs(*it - latest_hw - offset_old) < 1e-3) {
        offset_new = *it - latest_hw;
        tm_utc_.erase(tm_utc_.begin(), tm_utc_.begin() + idx);
        tm_hw_.clear();
        return true;
      }
      idx--;
    }

    return false;
  }

  double calibrate(const char *tag) {
    bool ok = false;
    curi::usb_utc_rx_data_t timestamp;
    time_t unix_secs;
    int nano_secs;
    double offset = 0.;
    const char *keys = nullptr;
    if (strcmp(tag, "imu") == 0)
      keys = "'I' or 'i'";
    else
      keys = "'C' or 'c'";
    while (!ok) {
      clear();
      ROS_INFO(
          "Please press %s to calibrate clock offset between MCU and %s. "
          "You are suggested to press the keys 5-10 times to trigger the %s. "
          "Note: the time interval between press action must not be larger than 10 seconds, "
          "otherwise the key press process are regarded as finished and begin to calculate offset.",
          keys, tag, tag);
      bool start = false;
      bool done = false;
      while (!done) {
        bool received = recv_.retrieveTimestamp(timestamp, start, 10000);
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
      // calculate the clock offset between mcu and sensor
      ok = Execute(offset);
    }
    return offset;
  }

  void run() {
    /// calibrate IMU and MCU clock offset
    imu_subscriber_object_ = nh_.subscribe(imu_topic_name_,
                                           1000,
                                           &Synchronizer::imuTimestampCallback,
                                           this);
    double imu_offset = calibrate("imu");
    std_msgs::Float64 imu_offset_pub;
    imu_offset_pub.data = imu_offset;
    imu_pub_object_.publish(imu_offset_pub);
    imu_subscriber_object_.shutdown();
    clear();
    ROS_INFO("Finish IMU clock offset calibration");

    /// calibrate camera and MCU clock offset
    cam_subscriber_object_ = nh_.subscribe(cam_topic_name_,
                                           1000,
                                           &Synchronizer::camTimestampCallback,
                                           this);
    double cam_offset = calibrate("camera");
    ROS_INFO("Finish camera clock offset initial calibration");

    /// finish key event handler and enable camera auto trigger
    key_done_ = true;
    recv_.sendCMD(2);

    /// continuously update the clock offset between camera and MCU
    std_msgs::Float64 cam_offset_pub;
    ros::Time last_offset_time = ros::Time::now();
    double cam_offset_new = cam_offset;
    while (!done_) {
      if ((ros::Time::now() - last_offset_time).toSec() > 60) {
        last_offset_time = ros::Time::now();
        ROS_INFO("Camera-MCU clock offset: %.6lf", cam_offset);
      }

      // receive new timestamp from MCU
      curi::usb_utc_rx_data_t timestamp;
      time_t unix_secs;
      int nano_secs;
      if (recv_.retrieveTimestamp(timestamp, true, 1000)) {
        if (curi::mcu_sync(&timestamp)) {
          curi::parse_mcu_time(&timestamp, &unix_secs, &nano_secs);
          double tm = unix_secs + ((double) nano_secs) / 1e9;
          add_utc_time(tm);
        } else {
          add_utc_time(timestamp.st_time);
        }
        if (sync(cam_offset, cam_offset_new)) {
          cam_offset = cam_offset_new;
          cam_offset_pub.data = cam_offset;
          cam_pub_object_.publish(cam_offset_pub);
        }
      }
    } // while (!done_)
  }

  void keyLoop() {
    while (!key_done_) {
      int ch = getch();
      if (ch == 'C' || ch == 'c') {  // trigger camera
        std::cout << "send 1" << std::endl;
        recv_.sendCMD(1);
      } else if (ch == 'I' || ch == 'i') {  // trigger IMU
        recv_.sendCMD(0);
      } else {  // forward key event
        scoped_m lk(key_mutex_);
        key_press_ = true;
        key_ = ch;
        lk.unlock();
        key_cond_.notify_all();
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