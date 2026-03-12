//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include "rclcpp/rclcpp.hpp"

#include "K_MappingRos.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Check for use_sim_time in command line arguments
  bool use_sim_time = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.find("use_sim_time:=true") != std::string::npos) {
      use_sim_time = true;
      break;
    }
  }

  // Create node with use_sim_time option
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", use_sim_time}});
  auto node = std::make_shared<rclcpp::Node>("hector_slam", options);

  RCLCPP_INFO(node->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");

  K_MappingRos sm(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return(0);
}

