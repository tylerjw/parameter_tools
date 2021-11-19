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

#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std::literals;

TEST(ExampleIntegrationTests, ListParameters) {
  // GIVEN connection to node using AsyncParametersClient
  std::string test_server_name = "example_node";
  auto node = std::make_shared<rclcpp::Node>("list_parameters_test");
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node, test_server_name);
  if (!parameters_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  // WHEN we get the parameter names and prefixes
  auto result = parameters_client->list_parameters({"robot1"}, 1);
  rclcpp::spin_until_future_complete(node, result);
  auto parameters_and_prefixes = result.get();

  // THEN we expect the names to be robot1.joint_state_topic and
  // robot1.robot_description and the prefix to be robot1
  for (auto& name : parameters_and_prefixes.names) {
    EXPECT_TRUE(name == "robot1.joint_state_topic" ||
                name == "robot1.robot_description")
        << "unexpected parameter: " << name;
  }
  for (auto& prefix : parameters_and_prefixes.prefixes) {
    EXPECT_TRUE(prefix == "robot1");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
