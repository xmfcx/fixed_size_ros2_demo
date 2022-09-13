// Copyright (c) 2019 by Robert Bosch GmbH. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BURGER_SUBSCRIBER_HPP_
#define BURGER_SUBSCRIBER_HPP_

#include <inttypes.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

template<class MsgT, size_t width = MsgT::WIDTH, size_t height = MsgT::HEIGHT>
class BurgerSubscriber {
 public:
  BurgerSubscriber(std::shared_ptr<rclcpp::Node> node, std::string topic, bool show_gui = true)
      : sub_(node->create_subscription<MsgT>(
      topic,
      9,
      std::bind(&BurgerSubscriber<MsgT>::show_image, this, std::placeholders::_1))),
        node_(node),
        show_gui_(show_gui),
        k_(0),
        average_round_time_(0),
        init_time_{false},
        count_{0} {}

  virtual ~BurgerSubscriber() {
    auto logger = rclcpp::get_logger("image_transport_subscriber");
    RCLCPP_INFO(logger, "Received %" PRId64 " messages", k_);
    RCLCPP_INFO(
        logger, "Average round time %f milliseconds", static_cast<float>(average_round_time_) / 1e6);
  }

  void show_image(std::shared_ptr<MsgT> image) {
    rclcpp::Time time_message(image->timestamp);

    if (!init_time_) {
      last_time_ = time_message;
      init_time_ = true;
    } else {
      count_++;
      rclcpp::Duration duration = time_message - last_time_;
      if (duration.seconds() > 3.0) {
        RCLCPP_INFO(rclcpp::get_logger("image_transport_subscriber"),
                    "Received %d images in %f secs; %f hz.",
                    count_,
                    duration.seconds(),
                    static_cast<double>(count_) / duration.seconds());
        count_ = 0;
        last_time_ = time_message;
      }
    }

    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    auto logger = rclcpp::get_logger("image_transport_subscriber");
    auto msg_timestamp = image.get()->timestamp;

    auto diff = now - msg_timestamp;

    rclcpp::Duration duration_msg(now - msg_timestamp);
    RCLCPP_INFO(
        logger,
        "Received message %zu on address %p delay: %f ms.", image->step, image.get(), duration_msg.seconds());

    ++k_;
    average_round_time_ = ((k_ - 1) * average_round_time_ + diff) / k_;
    if (show_gui_) {
      cv::Mat frame(height, width, CV_8UC3, image.get()->data.data());
      cv::imshow("burger_subscriber", frame);
      cv::waitKey(1);
    }
  }

  void run() {
    rclcpp::spin(node_);
  }

 private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::shared_ptr<rclcpp::Node> node_;
  bool show_gui_;
  int64_t k_;
  int64_t average_round_time_;

  bool init_time_;
  rclcpp::Time last_time_;
  int count_;
};

#endif  // BURGER_SUBSCRIBER_HPP_
