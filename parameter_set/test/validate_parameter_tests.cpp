// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <parameter_set/validate_parameter.hpp>

using namespace parameter_set::validate;

TEST(ValidateParameterTests, AlwaysReject) {
  // GIVEN a default rclcpp::Parameter
  rclcpp::Parameter parameter;

  // WHEN we call always_reject
  auto result = always_reject(parameter);

  // THEN we expect the success flag to be false
  EXPECT_FALSE(result.successful);
}

TEST(ValidateParameterTests, AlwaysAccept) {
  // GIVEN a default rclcpp::Parameter
  rclcpp::Parameter parameter;

  // WHEN we call always_accept
  auto result = always_accept(parameter);

  // THEN we expect the success flag to be true
  EXPECT_TRUE(result.successful);
}

TEST(ValidateParameterTests, ValidTopic) {
  // GIVEN a rclcpp::Parameter with a valid topic name
  rclcpp::Parameter parameter("topic", "/joint_states");

  // WHEN we call topic_name
  auto result = topic_name(parameter);

  // THEN we expect the success flag to be true
  EXPECT_TRUE(result.successful);
}

TEST(ValidateParameterTests, InvalidTopic) {
  // GIVEN a rclcpp::Parameter with a invalid topic name
  rclcpp::Parameter parameter("topic", "123456^&*(");

  // WHEN we call topic_name
  auto result = topic_name(parameter);

  // THEN we expect the success flag to be false
  EXPECT_FALSE(result.successful);
}

TEST(ValidateParameterTests, NotStringTopic) {
  // GIVEN a rclcpp::Parameter with a non-string type
  rclcpp::Parameter parameter("topic", 10.4);

  // WHEN we call topic_name
  // THEN we expect it to throw
  EXPECT_THROW(topic_name(parameter), std::exception);
}

TEST(ValidateParameterTests, EmptyString) {
  // GIVEN a rclcpp::Parameter with a empty string
  rclcpp::Parameter parameter("param", "");

  // WHEN we call not_empty_string
  auto result = not_empty_string(parameter);

  // THEN we expect the success flag to be false
  EXPECT_FALSE(result.successful);
}

TEST(ValidateParameterTests, NotEmptyString) {
  // GIVEN a rclcpp::Parameter with a not empty string
  rclcpp::Parameter parameter("param", "not");

  // WHEN we call not_empty_string
  auto result = not_empty_string(parameter);

  // THEN we expect the success flag to be false
  EXPECT_TRUE(result.successful);
}

TEST(ValidateParameterTests, InStringSet) {
  // GIVEN a rclcpp::Parameter with a string, and a set of strings with that
  // string
  rclcpp::Parameter parameter("param", "value1");
  std::set<std::string> values = {"value0", "value1", "value2"};

  // WHEN we call in_string_set
  auto result = in_string_set(parameter, values);

  // THEN we expect the success flag to be true
  EXPECT_TRUE(result.successful);
}

TEST(ValidateParameterTests, NotInStringSet) {
  // GIVEN a rclcpp::Parameter with a string, and a set of strings with out that
  // string
  rclcpp::Parameter parameter("param", "value1");
  std::set<std::string> values = {"value0", "value2"};

  // WHEN we call in_string_set
  auto result = in_string_set(parameter, values);

  // THEN we expect the success flag to be false
  EXPECT_FALSE(result.successful);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
