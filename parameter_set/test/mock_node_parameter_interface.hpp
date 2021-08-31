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

#pragma once

#include <gmock/gmock.h>

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using rclcpp::node_interfaces::OnSetParametersCallbackHandle;

struct MockNodeParameterInterface
    : public rclcpp::node_interfaces::NodeParametersInterface {
  using rclcpp::node_interfaces::NodeParametersInterface::
      OnParametersSetCallbackType;
  MOCK_METHOD(const rclcpp::ParameterValue &, declare_parameter,
              (const std::string &name), (override));
  MOCK_METHOD(
      const rclcpp::ParameterValue &, declare_parameter,
      (const std::string &name, const rclcpp::ParameterValue &default_value,
       const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
       bool ignore_override),
      (override));
  MOCK_METHOD(
      const rclcpp::ParameterValue &, declare_parameter,
      (const std::string &, rclcpp::ParameterType,
       const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
       bool ignore_override),
      (override));
  MOCK_METHOD(void, undeclare_parameter, (const std::string &name), (override));
  MOCK_METHOD(bool, has_parameter, (const std::string &name),
              (const, override));
  MOCK_METHOD(std::vector<rcl_interfaces::msg::SetParametersResult>,
              set_parameters,
              (const std::vector<rclcpp::Parameter> &parameters), (override));
  MOCK_METHOD(rcl_interfaces::msg::SetParametersResult,
              set_parameters_atomically,
              (const std::vector<rclcpp::Parameter> &parameters), (override));
  MOCK_METHOD(std::vector<rclcpp::Parameter>, get_parameters,
              (const std::vector<std::string> &names), (const, override));
  MOCK_METHOD(rclcpp::Parameter, get_parameter, (const std::string &name),
              (const override));
  MOCK_METHOD(bool, get_parameter,
              (const std::string &name, rclcpp::Parameter &parameter),
              (const override));
  MOCK_METHOD(bool, get_parameters_by_prefix,
              (const std::string &prefix,
               (std::map<std::string, rclcpp::Parameter> &)parameters),
              (const, override));
  MOCK_METHOD(std::vector<rcl_interfaces::msg::ParameterDescriptor>,
              describe_parameters, (const std::vector<std::string> &names),
              (const, override));
  MOCK_METHOD(std::vector<uint8_t>, get_parameter_types,
              (const std::vector<std::string> &names), (const, override));
  MOCK_METHOD(rcl_interfaces::msg::ListParametersResult, list_parameters,
              (const std::vector<std::string> &prefixes, uint64_t depth),
              (const, override));
  MOCK_METHOD(OnSetParametersCallbackHandle::SharedPtr,
              add_on_set_parameters_callback,
              (OnParametersSetCallbackType callback), (override));
  MOCK_METHOD(void, remove_on_set_parameters_callback,
              (const OnSetParametersCallbackHandle *const handler), (override));
  MOCK_METHOD((const std::map<std::string, rclcpp::ParameterValue> &),
              get_parameter_overrides, (), (const, override));
};
