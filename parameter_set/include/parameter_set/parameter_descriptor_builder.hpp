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

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <string>

namespace parameter_set {

class ParameterDescriptorBuilder {
  rcl_interfaces::msg::ParameterDescriptor msg_;

 public:
  /**
   * @brief      Rcl_interfaces::msg::parameterdescriptor conversion operator.
   */
  operator rcl_interfaces::msg::ParameterDescriptor() const {
    return move(msg_);
  }

  /**
   * @brief      Set the type
   *
   * @param[in]  type  The type
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& type(uint8_t type);

  /**
   * @brief      Set the description string
   *
   * @param[in]  description  The description
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& description(std::string description);

  /**
   * @brief      Set the aditional constraints string
   *
   * @param[in]  additional_constraints  The additional constraints
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& additional_constraints(
      std::string additional_constraints);

  /**
   * @brief      Sets the read only flag
   *
   * @param[in]  read_only  The read only flag
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& read_only(bool read_only);
  // Rolling only

  /**
   * @brief      Set the dynamic typing flag (rolling only)
   *
   * @param[in]  dynamic_typing  The dynamic typing flag
   *
   * @return     Reference to this object
   */
  // ParameterDescriptorBuilder& dynamic_typing(bool dynamic_typing);

  /**
   * @brief      Set floating point range
   *
   * @param[in]  from_value  The from value
   * @param[in]  to_value    To value
   * @param[in]  step        The step
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& floating_point_range(
      double from_value = std::numeric_limits<double>::min(),
      double to_value = std::numeric_limits<double>::max(), double step = 0);

  /**
   * @brief      Set the integer range
   *
   * @param[in]  from_value  The from value
   * @param[in]  to_value    To value
   * @param[in]  step        The step
   *
   * @return     Reference to this object
   */
  ParameterDescriptorBuilder& integer_range(
      int64_t from_value = std::numeric_limits<int64_t>::min(),
      int64_t to_value = std::numeric_limits<int64_t>::max(), int64_t step = 0);
};

}  // namespace parameter_set
