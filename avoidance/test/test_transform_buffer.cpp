#include <gtest/gtest.h>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "avoidance/transform_buffer.h"

using namespace avoidance;

class TransformBufferTests : public ::testing::Test, public tf_buffer::TransformBuffer {
 protected:
  void SetUp() override {
    // ROS2 doesn't need explicit time initialization like ROS1
  }
  
  geometry_msgs::msg::TransformStamped createTransform(const rclcpp::Time& time,
                                                        double tx, double ty, double tz,
                                                        double qx = 0.0, double qy = 0.0,
                                                        double qz = 0.0, double qw = 1.0) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time;
    tf.transform.translation.x = tx;
    tf.transform.translation.y = ty;
    tf.transform.translation.z = tz;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    return tf;
  }
};

TEST(TransformBuffer, GetTransformAPI) {
  // GIVEN: a transform Buffer and source/target frames
  tf_buffer::TransformBuffer tf_buffer(10.0f);
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time time1 = clock.now();
  rclcpp::Time time2 = time1 - rclcpp::Duration(2, 0);
  rclcpp::Time time3 = time1 - rclcpp::Duration(4, 0);
  rclcpp::Time time_between = time1 - rclcpp::Duration(1, 500000000);
  rclcpp::Time time_before = time1 - rclcpp::Duration(6, 0);
  rclcpp::Time time_after = time1 + rclcpp::Duration(1, 0);

  geometry_msgs::msg::TransformStamped transform1, transform2, transform3, retrieved_transform;
  
  transform1.header.stamp = time1;
  transform1.transform.translation.x = 0.0;
  transform1.transform.translation.y = 0.0;
  transform1.transform.translation.z = 0.0;
  transform1.transform.rotation.x = 0.0;
  transform1.transform.rotation.y = 0.0;
  transform1.transform.rotation.z = 0.0;
  transform1.transform.rotation.w = 1.0;
  
  transform2.header.stamp = time2;
  transform2.transform.translation.x = 0.0;
  transform2.transform.translation.y = 0.0;
  transform2.transform.translation.z = 0.0;
  transform2.transform.rotation.x = 0.0;
  transform2.transform.rotation.y = 0.0;
  transform2.transform.rotation.z = 0.0;
  transform2.transform.rotation.w = 1.0;
  
  transform3.header.stamp = time3;
  transform3.transform.translation.x = 0.0;
  transform3.transform.translation.y = 0.0;
  transform3.transform.translation.z = 0.0;
  transform3.transform.rotation.x = 0.0;
  transform3.transform.rotation.y = 0.0;
  transform3.transform.rotation.z = 0.0;
  transform3.transform.rotation.w = 1.0;

  // WHEN: we insert the 3 transforms into the buffer
  ASSERT_TRUE(tf_buffer.insertTransform(source_frame, target_frame, transform3));
  ASSERT_TRUE(tf_buffer.insertTransform(source_frame, target_frame, transform2));
  ASSERT_TRUE(tf_buffer.insertTransform(source_frame, target_frame, transform1));

  // THEN: we should be able to retrieve transforms at different times
  // time1 should get transform1
  ASSERT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time1, retrieved_transform));

  // time2 should get transform2
  ASSERT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time2, retrieved_transform));

  // time3 should get transform3
  ASSERT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time3, retrieved_transform));

  // time in between should give the timestamp of what we ask for
  ASSERT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time_between, retrieved_transform));

  // outside of the buffer should not give a transform
  EXPECT_FALSE(tf_buffer.getTransform(source_frame, target_frame, time_before, retrieved_transform));
  EXPECT_FALSE(tf_buffer.getTransform(source_frame, target_frame, time_after, retrieved_transform));
}

TEST_F(TransformBufferTests, insertTransform) {
  // GIVEN: a transform Buffer and source/target frames
  std::string source_frame = "frame1";
  std::string target_frame1 = "frame2";
  std::string target_frame2 = "frame3";
  std::string target_frame3 = "frame4";

  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time time1 = clock.now();
  rclcpp::Time time2 = time1 - rclcpp::Duration(2, 0);
  rclcpp::Time time3 = time1 - rclcpp::Duration(4, 0);

  auto transform1 = createTransform(time1, 0.0, 0.0, 0.0);
  auto transform2 = createTransform(time2, 0.0, 0.0, 0.0);
  auto transform3 = createTransform(time3, 0.0, 0.0, 0.0);

  // THEN: inserting transforms should work depending on timestamps
  EXPECT_TRUE(insertTransform(source_frame, target_frame1, transform3));
  EXPECT_TRUE(insertTransform(source_frame, target_frame1, transform2));
  EXPECT_FALSE(insertTransform(source_frame, target_frame1, transform2));
  EXPECT_TRUE(insertTransform(source_frame, target_frame1, transform1));
  EXPECT_FALSE(insertTransform(source_frame, target_frame1, transform1));
  EXPECT_FALSE(insertTransform(source_frame, target_frame1, transform2));

  EXPECT_TRUE(insertTransform(source_frame, target_frame2, transform3));
  EXPECT_TRUE(insertTransform(source_frame, target_frame2, transform2));

  EXPECT_TRUE(insertTransform(source_frame, target_frame3, transform2));

  // AND: the buffer should contain 3 transforms for the first target, and 2 for
  // the second
  EXPECT_EQ(buffer_[getKey(source_frame, target_frame1)].size(), 3u);
  EXPECT_EQ(buffer_[getKey(source_frame, target_frame2)].size(), 2u);
}

TEST_F(TransformBufferTests, interpolateTransform) {
  // GIVEN: two transforms with translations and rotations
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time time1 = clock.now();
  rclcpp::Time time_half = time1 + rclcpp::Duration(1, 0);
  rclcpp::Time time2 = time1 + rclcpp::Duration(2, 0);

  tf2::Quaternion rotation2, rotation_half;
  rotation2.setRPY(1.0, 0.0, 0.0);
  rotation_half.setRPY(0.5, 0.0, 0.0);

  auto transform1 = createTransform(time1, 0.0, 0.0, 0.0);
  auto transform2 = createTransform(time2, 0.0, 0.0, 2.0, 
                                     rotation2.x(), rotation2.y(), 
                                     rotation2.z(), rotation2.w());

  geometry_msgs::msg::TransformStamped retrieved_transform1, retrieved_transform2, retrieved_transform3;
  retrieved_transform1.header.stamp = time_half;
  retrieved_transform2.header.stamp = time1;
  retrieved_transform3.header.stamp = time2;

  // WHEN: we interpolate to a time exactly between the two given transforms
  ASSERT_TRUE(interpolateTransform(transform1, transform2, retrieved_transform1));
  ASSERT_TRUE(interpolateTransform(transform1, transform2, retrieved_transform2));
  ASSERT_TRUE(interpolateTransform(transform1, transform2, retrieved_transform3));

  // THEN: we should get half the translation for the time in between
  EXPECT_NEAR(retrieved_transform1.transform.translation.z, 1.0, 0.001);
}
