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

#include <parameter_set/parameter_descriptor_builder.hpp>

using parameter_set::ParameterDescriptorBuilder;

TEST(ParameterDescriptorBuilderTests, MsgConversion) {
  // GIVEN a ParameterDescriptorBuilder
  ParameterDescriptorBuilder builder;

  // THEN we expect to be able to cast it to
  // rcl_interfaces::msg::ParameterDescriptor
  rcl_interfaces::msg::ParameterDescriptor msg;
  EXPECT_NO_THROW(msg = builder);
}

TEST(ParameterDescriptorBuilderTests, DefaultMsg) {
  // GIVEN a rcl_interfaces::msg::ParameterDescriptor created with
  // ParameterDescriptorBuilder
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder();

  // THEN we expect it be equal to a default constructed message
  rcl_interfaces::msg::ParameterDescriptor default_msg;
  EXPECT_EQ(built_msg, default_msg);
}

TEST(ParameterDescriptorBuilderTests, SettingType) {
  // GIVEN a type
  // WHEN we build a message with type
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().type(
          rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

  // THEN we expect the type in that message to be the same
  EXPECT_EQ(built_msg.type,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
}

TEST(ParameterDescriptorBuilderTests, SettingDescription) {
  // GIVEN a description
  // WHEN we build a message with description
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().description("example");

  // THEN we expect the description in that message to be the same
  EXPECT_EQ(built_msg.description, "example");
}

TEST(ParameterDescriptorBuilderTests, SettingAditionalConstraints) {
  // GIVEN a additional_constraints string
  // WHEN we build a message with additional_constraints
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().additional_constraints("constraint");

  // THEN we expect the additional_constraints in that message to be the same
  EXPECT_EQ(built_msg.additional_constraints, "constraint");
}

TEST(ParameterDescriptorBuilderTests, SettingReadOnly) {
  // GIVEN a read_only setting
  // WHEN we build a message with that read_only
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().read_only(true);

  // THEN we expect the read_only in that message to be the same
  EXPECT_EQ(built_msg.read_only, true);
}

TEST(ParameterDescriptorBuilderTests, SettingFloatingPointRange) {
  // GIVEN a floating_point_range setting
  // WHEN we build a message with that floating_point_range
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().floating_point_range(1., 3., 0.1);

  // THEN we expect the floating_point_range to be set with a size of 1
  EXPECT_EQ(built_msg.floating_point_range.size(), 1);
  EXPECT_EQ(built_msg.floating_point_range[0].from_value, 1.);
  EXPECT_EQ(built_msg.floating_point_range[0].to_value, 3.);
  EXPECT_EQ(built_msg.floating_point_range[0].step, 0.1);
}

TEST(ParameterDescriptorBuilderTests, SettingIntegerPointRange) {
  // GIVEN a integer_range setting
  // WHEN we build a message with that integer_range
  rcl_interfaces::msg::ParameterDescriptor built_msg =
      ParameterDescriptorBuilder().integer_range(1, 10, 2);

  // THEN we expect the integer_range to be set with a size of 1
  EXPECT_EQ(built_msg.integer_range.size(), 1);
  EXPECT_EQ(built_msg.integer_range[0].from_value, 1);
  EXPECT_EQ(built_msg.integer_range[0].to_value, 10);
  EXPECT_EQ(built_msg.integer_range[0].step, 2);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
