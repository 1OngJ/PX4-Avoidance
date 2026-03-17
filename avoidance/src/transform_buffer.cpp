#include "avoidance/transform_buffer.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace avoidance {

namespace tf_buffer {

TransformBuffer::TransformBuffer(float buffer_size_s) 
    : buffer_size_(std::chrono::nanoseconds(static_cast<int64_t>(buffer_size_s * 1e9))) {
  startup_time_ = clock_.now();
}

std::string TransformBuffer::getKey(const std::string& source_frame, const std::string& target_frame) const {
  return source_frame + "_to_" + target_frame;
}

bool TransformBuffer::interpolateTransform(const geometry_msgs::msg::TransformStamped& tf_earlier,
                                           const geometry_msgs::msg::TransformStamped& tf_later,
                                           geometry_msgs::msg::TransformStamped& transform) const {
  // check if the requested timestamp lies between the two given transforms
  rclcpp::Time tf_later_time(tf_later.header.stamp);
  rclcpp::Time tf_earlier_time(tf_earlier.header.stamp);
  rclcpp::Time transform_time(transform.header.stamp);
  
  if (transform_time > tf_later_time || transform_time < tf_earlier_time) {
    return false;
  }

  rclcpp::Duration timeBetween = tf_later_time - tf_earlier_time;
  rclcpp::Duration timeAfterEarlier = transform_time - tf_earlier_time;
  float tau = static_cast<float>(timeAfterEarlier.nanoseconds()) / timeBetween.nanoseconds();

  // Interpolate translation
  transform.transform.translation.x = 
      tf_earlier.transform.translation.x * (1.f - tau) + tf_later.transform.translation.x * tau;
  transform.transform.translation.y = 
      tf_earlier.transform.translation.y * (1.f - tau) + tf_later.transform.translation.y * tau;
  transform.transform.translation.z = 
      tf_earlier.transform.translation.z * (1.f - tau) + tf_later.transform.translation.z * tau;

  // Interpolate rotation using slerp
  tf2::Quaternion q_earlier, q_later, q_result;
  tf2::fromMsg(tf_earlier.transform.rotation, q_earlier);
  tf2::fromMsg(tf_later.transform.rotation, q_later);
  q_result = q_earlier.slerp(q_later, tau);
  transform.transform.rotation = tf2::toMsg(q_result);

  return true;
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      geometry_msgs::msg::TransformStamped transform) {
  std::lock_guard<std::mutex> lck(mutex_);
  auto iterator = buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    std::deque<geometry_msgs::msg::TransformStamped> empty_deque;
    buffer_[getKey(source_frame, target_frame)] = empty_deque;
    iterator = buffer_.find(getKey(source_frame, target_frame));
  }

  rclcpp::Time transform_time(transform.header.stamp);
  
  // check if the given transform is newer than the last buffered one
  if (iterator->second.size() == 0 || 
      rclcpp::Time(iterator->second.back().header.stamp) < transform_time) {
    iterator->second.push_back(transform);
    // remove transforms which are outside the buffer size
    while ((transform_time - rclcpp::Time(iterator->second.front().header.stamp)) > buffer_size_) {
      iterator->second.pop_front();
    }
    return true;
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame,
                                   const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& transform) const {
  std::lock_guard<std::mutex> lck(mutex_);
  auto iterator = buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    print(log_level::error, "TF Buffer: could not retrieve requested transform from buffer, unregistered");
    return false;
  } else if (iterator->second.size() == 0) {
    print(log_level::warn, "TF Buffer: could not retrieve requested transform from buffer, buffer is empty");
    return false;
  } else {
    rclcpp::Time back_time(iterator->second.back().header.stamp);
    rclcpp::Time front_time(iterator->second.front().header.stamp);
    
    if (back_time < time) {
      print(log_level::debug, "TF Buffer: could not retrieve requested transform from buffer, tf has not yet arrived");
      return false;
    } else if (front_time > time) {
      print(log_level::warn,
            "TF Buffer: could not retrieve requested transform from buffer, tf has already been dropped from buffer");
      return false;
    } else {
      const geometry_msgs::msg::TransformStamped* previous = &iterator->second.back();
      for (auto it = ++iterator->second.rbegin(); it != iterator->second.rend(); ++it) {
        if (rclcpp::Time(it->header.stamp) <= time) {
          const geometry_msgs::msg::TransformStamped& tf_earlier = *it;
          const geometry_msgs::msg::TransformStamped& tf_later = *previous;
          transform.header.stamp = time;
          if (interpolateTransform(tf_earlier, tf_later, transform)) {
            return true;
          } else {
            print(log_level::warn, "TF Buffer: could not interpolate transform");
            return false;
          }
        }
        previous = &(*it);
      }
    }
  }
  return false;
}

void TransformBuffer::print(const log_level& level, const std::string& msg) const {
  if ((clock_.now() - startup_time_) > rclcpp::Duration(3, 0)) {
    switch (level) {
      case error: {
        RCLCPP_ERROR(rclcpp::get_logger("transform_buffer"), "%s", msg.c_str());
        break;
      }
      case warn: {
        RCLCPP_WARN(rclcpp::get_logger("transform_buffer"), "%s", msg.c_str());
        break;
      }
      case info: {
        RCLCPP_INFO(rclcpp::get_logger("transform_buffer"), "%s", msg.c_str());
        break;
      }
      case debug: {
        RCLCPP_DEBUG(rclcpp::get_logger("transform_buffer"), "%s", msg.c_str());
        break;
      }
    }
  }
}
}  // namespace tf_buffer
}  // namespace avoidance
