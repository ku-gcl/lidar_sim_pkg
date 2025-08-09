/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

#include <cstdio>

std::string topic_pub = "/cmd_vel";
ignition::transport::Node node;
auto pub = node.Advertise<ignition::msgs::Twist>(topic_pub);

void cb(const ignition::msgs::LaserScan &_msg) {
    ignition::msgs::Twist data;
    bool allMore = true;
    for (int i = 0; i < _msg.ranges_size(); i++) {
        if (_msg.ranges(i) < 1.0) {
            allMore = false;
            break;
        }
    }
    if (allMore) {
        data.mutable_linear()->set_x(0.5);
        data.mutable_angular()->set_z(0.0);
    } else {
        data.mutable_linear()->set_x(0.0);
        data.mutable_angular()->set_z(-0.5);
    }
    pub.Publish(data);
}

int main(int argc, char **argv) {
    std::string topic_sub = "/lidar";
    if (!node.Subscribe(topic_sub, cb)) {
        std::cerr << "Error subscribing to topic [" << topic_sub << "]"
                  << std::endl;
        return -1;
    }

    ignition::transport::waitForShutdown();

    return 0;
}
