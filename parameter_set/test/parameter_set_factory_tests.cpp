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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <parameter_set/parameter_set.hpp>
#include <parameter_set/validate_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mock_node_parameter_interface.hpp"

using parameter_set::ParameterSetFactory;
using rclcpp::node_interfaces::NodeParametersInterface;

using namespace std::chrono_literals;
using namespace parameter_set::validate;

namespace {

// Globals for use by CountingParameterSet
static int declare_count = 0;
static int get_count = 0;

struct CountingParameterSet : public parameter_set::ParameterSet {
  using ParameterSet::ParameterSet;

  absl::Status declare(
      ParameterSetFactory* parameter_set_factory,
      const NodeParametersInterface::SharedPtr& node_parameters) override {
    declare_count++;
    return absl::OkStatus();
  }

  absl::Status get(
      const NodeParametersInterface::SharedPtr& node_parameters) override {
    get_count++;
    return absl::OkStatus();
  }
};

struct GetErrorParameterSet : public parameter_set::ParameterSet {
  using ParameterSet::ParameterSet;

  absl::Status declare(
      ParameterSetFactory* parameter_set_factory,
      const NodeParametersInterface::SharedPtr& node_parameters) override {
    return absl::OkStatus();
  }

  absl::Status get(
      const NodeParametersInterface::SharedPtr& node_parameters) override {
    return absl::UnknownError("error!");
  }
};

// For suppressing -Wunused-result
template <class T>
void ignore(const T&) {}

bool waitFor(std::chrono::seconds timeout, std::function<bool()> done) {
  const auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout) {
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

}  // namespace

TEST(ParameterSetFactoryTests,
     FactoryConstructorCallsAddOnSetParametersCallback) {
  // GIVEN a mocked middleware handle
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();

  // THEN we expect it to call this method on the MockNodeParameterInterface
  EXPECT_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .Times(1);

  // WHEN we construct the occupancy map monitor
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};
}

TEST(ParameterSetFactoryTests, DeclareCallsDeclare) {
  // GIVEN a parameter_set_factory and current declare count
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  auto pre_declare_count = declare_count;

  // WHEN we call declare on the factory
  ignore(parameter_set_factory.declare<CountingParameterSet>("ns"));

  // THEN we expect the declare count to have increased
  EXPECT_GT(declare_count, pre_declare_count);
}

TEST(ParameterSetFactoryTests, GetCallsGet) {
  // GIVEN a parameter_set_factory, declared set, and current get count
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  ignore(parameter_set_factory.declare<CountingParameterSet>("ns"));
  auto pre_get_count = get_count;

  // WHEN we call get on the factory
  ignore(parameter_set_factory.get<CountingParameterSet>("ns"));

  // THEN we expect the get count to have increased
  EXPECT_GT(get_count, pre_get_count);
}

TEST(ParameterSetFactoryTests, GetIsOk) {
  // GIVEN a parameter_set_factory with a declared set
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  ignore(parameter_set_factory.declare<CountingParameterSet>("ns"));

  // WHEN we call get on the factory
  auto result = parameter_set_factory.get<CountingParameterSet>("ns");

  // THEN we expect the result to be ok
  EXPECT_TRUE(result.ok());
}

TEST(ParameterSetFactoryTests, GetIsNotOk) {
  // GIVEN a parameter_set_factory with a declared set
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  ignore(parameter_set_factory.declare<GetErrorParameterSet>("ns"));

  // WHEN we call get on the factory
  auto result = parameter_set_factory.get<GetErrorParameterSet>("ns");

  // THEN we expect the result to be ok
  EXPECT_FALSE(result.ok());
}

TEST(ParameterSetFactoryTests, GetNotDeclared) {
  // GIVEN a parameter_set_factory
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};

  // WHEN we call get on the factory for a set not yet declared
  auto result = parameter_set_factory.get<CountingParameterSet>("ns");

  // THEN we expect the result to be a error status
  EXPECT_FALSE(result.ok());
}

TEST(ParameterSetFactoryTests, DeclareAndGetCount) {
  // GIVEN a parameter_set_factory and current declare and get count
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  auto pre_declare_count = declare_count;
  auto pre_get_count = get_count;

  // WHEN we call declare_and_get on the factory
  ignore(parameter_set_factory.declare_and_get<CountingParameterSet>("ns"));

  // THEN we expect the declare and get count to have increased
  EXPECT_GT(declare_count, pre_declare_count);
  EXPECT_GT(get_count, pre_get_count);
}

TEST(ParameterSetFactoryTests, DeclareAndGetOk) {
  // GIVEN a parameter_set_factory
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};
  auto pre_declare_count = declare_count;
  auto pre_get_count = get_count;

  // WHEN we call declare_and_get on the factory
  auto result =
      parameter_set_factory.declare_and_get<CountingParameterSet>("ns");

  // THEN we expect the result to be ok
  EXPECT_TRUE(result.ok());
}

TEST(ParameterSetFactoryTests, SplitParameterName) {
  // GIVEN a parameter name with a namespace
  auto name = "ns.name";

  // WHEN we call split_parameter_name
  auto result = parameter_set::split_parameter_name(name);

  // THEN we expect the result be a pair of namespace, parameter name
  EXPECT_EQ(result.first, "ns");
  EXPECT_EQ(result.second, "name");
}

TEST(ParameterSetFactoryTests, SplitParameterNameNoNamespace) {
  // GIVEN a parameter name without a namespace
  auto name = "name";

  // WHEN we call split_parameter_name
  auto result = parameter_set::split_parameter_name(name);

  // THEN we expect the result be a pair of "", parameter name
  EXPECT_EQ(result.first, "");
  EXPECT_EQ(result.second, "name");
}

TEST(ParameterSetFactoryTests, RegisterValidateFunctionNoThrow) {
  // GIVEN a parameter_set_factory
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};

  // WHEN we call registerValidateFunction
  // THEN it doesn't throw
  EXPECT_NO_THROW(parameter_set_factory.registerValidateFunction(
      "ns", "test", always_reject));
}

TEST(ParameterSetFactoryTests, RegisterSetChangedCallbackNoThrow) {
  // GIVEN a parameter_set_factory
  ParameterSetFactory parameter_set_factory{
      std::make_shared<MockNodeParameterInterface>()};

  // WHEN we call registerSetChangedCallback
  // THEN it doesn't throw
  EXPECT_NO_THROW(
      parameter_set_factory.registerSetChangedCallback("ns", []() {}));
}

TEST(ParameterSetFactoryTests, CallSetParametersCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};

  // WHEN we call on_parameters_set callback with an empty parameter vector
  rcl_interfaces::msg::SetParametersResult result = on_parameters_set({});

  // THEN we expect a successful result
  EXPECT_TRUE(result.successful);
}

TEST(ParameterSetFactoryTests, CallSetParametersNotDeclaredCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};

  // WHEN we call on_parameters_set callback with an undeclared parameter
  rcl_interfaces::msg::SetParametersResult result =
      on_parameters_set({rclcpp::Parameter{"ns.name", "value"}});

  // THEN we expect a successful result
  EXPECT_TRUE(result.successful);
}

TEST(ParameterSetFactoryTests, CallSetParametersFailValidateCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback with
  // registered parameter validate
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};
  parameter_set_factory.registerValidateFunction("ns", "name", always_reject);

  // WHEN we call on_parameters_set callback with the parameter we registered
  rcl_interfaces::msg::SetParametersResult result =
      on_parameters_set({rclcpp::Parameter{"ns.name", "value"}});

  // THEN we expect a failed result
  EXPECT_FALSE(result.successful);
}

TEST(ParameterSetFactoryTests, CallSetParametersSuccessValidateCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback with
  // registered parameter validate
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};
  parameter_set_factory.registerValidateFunction("ns", "name", always_accept);

  // WHEN we call on_parameters_set callback with the parameter we registered
  rcl_interfaces::msg::SetParametersResult result =
      on_parameters_set({rclcpp::Parameter{"ns.name", "value"}});

  // THEN we expect a sucessful result
  EXPECT_TRUE(result.successful);
}

TEST(ParameterSetFactoryTests, CallSetParametersValidateInvalidTypeCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback with
  // registered parameter validate requiring a string type
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};
  parameter_set_factory.registerValidateFunction("ns", "name",
                                                 not_empty_string);

  // WHEN we call on_parameters_set callback with an parameter with different
  // type but same name
  rcl_interfaces::msg::SetParametersResult result =
      on_parameters_set({rclcpp::Parameter{"ns.name", 12345.321}});

  // THEN we expect a false result
  EXPECT_FALSE(result.successful);
}

TEST(ParameterSetFactoryTests, CallSetParametersCallSetChangedCallback) {
  // GIVEN a mocked middleware handle and on_parameters_set callback with
  // registered parameter validate requiring a string type
  auto mock_parameter_set_interface =
      std::make_shared<MockNodeParameterInterface>();
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType
      on_parameters_set;
  ON_CALL(*mock_parameter_set_interface, add_on_set_parameters_callback)
      .WillByDefault([&](auto callback) {
        on_parameters_set = callback;
        return std::make_shared<
            rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
      });
  ParameterSetFactory parameter_set_factory{mock_parameter_set_interface};
  std::atomic<bool> callback_called{false};
  parameter_set_factory.registerSetChangedCallback(
      "ns", [&]() { callback_called = true; });

  // WHEN we call on_parameters_set callback with a parameter
  rcl_interfaces::msg::SetParametersResult result =
      on_parameters_set({rclcpp::Parameter{"ns.name", 12345.321}});

  // THEN we expect the callback was called
  EXPECT_TRUE(waitFor(10s, [&]() { return callback_called == true; }));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
