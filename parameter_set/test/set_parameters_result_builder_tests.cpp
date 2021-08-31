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

#include <parameter_set/set_parameters_result_builder.hpp>

using parameter_set::SetParametersResultBuilder;

TEST(SetParametersResultBuilderTests, MsgConversion) {
  // GIVEN a SetParametersResultBuilder
  auto builder = SetParametersResultBuilder(true);

  // THEN we expect to be able to cast it to
  // rcl_interfaces::msg::SetParametersResult
  rcl_interfaces::msg::SetParametersResult msg;
  EXPECT_NO_THROW(msg = builder);
}

TEST(SetParametersResultBuilderTests, SuccessMsg) {
  // GIVEN a success rcl_interfaces::msg::SetParametersResult created with
  // SetParametersResultBuilder
  rcl_interfaces::msg::SetParametersResult msg =
      SetParametersResultBuilder(true);

  // THEN we expect the success flag to be true
  EXPECT_TRUE(msg.successful);
}

TEST(SetParametersResultBuilderTests, FailureMsg) {
  // GIVEN a failure rcl_interfaces::msg::SetParametersResult created with
  // SetParametersResultBuilder
  rcl_interfaces::msg::SetParametersResult msg =
      SetParametersResultBuilder(false);

  // THEN we expect the success flag to be true
  EXPECT_FALSE(msg.successful);
}

TEST(SetParametersResultBuilderTests, SetReason) {
  // GIVEN a failure rcl_interfaces::msg::SetParametersResult created with
  // SetParametersResultBuilder with a reason
  rcl_interfaces::msg::SetParametersResult msg =
      SetParametersResultBuilder(false).reason("example");

  // THEN we expect the reason in the message to be the same
  EXPECT_EQ(msg.reason, "example");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
