# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Mavic 2 Pro driver."""

import math
import random
import rclpy
from wasp_autonomous_systems_interfaces.msg import Thrust


K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.
K_YAW_P = 2.0
K_X_VELOCITY_P = 1
K_Y_VELOCITY_P = 1


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')

        self.__node.create_subscription(
            Thrust, 'thrust', self.__thrust_callback, 1)
        self.__m1 = 0.0
        self.__m2 = 0.0
        self.__m3 = 0.0
        self.__m4 = 0.0
        self.__thrust_offset = random.uniform(-0.5, 0.5)
        self.__step = 0

    def __thrust_callback(self, msg: Thrust):
        self.__m1 = msg.m1
        self.__m2 = msg.m2
        self.__m3 = msg.m3
        self.__m4 = msg.m4

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__step += 1
        if 0 == self.__step % 100:
            self.__thrust_offset += random.uniform(-0.01, 0.0)

        # Read sensors
        roll, pitch, _ = self.__imu.getRollPitchYaw()
        x, y, vertical = self.__gps.getValues()
        roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
        velocity = self.__gps.getSpeed()
        if math.isnan(velocity):
            return

        pitch_ref = 0
        roll_ref = 0
        yaw_ref = 0

        # if 0.2 < vertical:
        #     # Calculate velocity
        #     velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
        #     velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

        #     linear_x_error = clamp(-x, -3, 3) - velocity_x
        #     linear_y_error = clamp(y, -3, 3) - velocity_y

        #     pitch_ref = K_X_VELOCITY_P * linear_x_error
        #     roll_ref = K_Y_VELOCITY_P * linear_y_error

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
        pitch_input = K_PITCH_P * \
            clamp(pitch, -1, 1) + pitch_velocity + pitch_ref
        # yaw_input = K_YAW_P * (yaw_ref - twist_yaw)
        yaw_input = 0

        m1 = self.__thrust_offset + self.__m1
        m2 = self.__thrust_offset + self.__m2
        m3 = self.__thrust_offset + self.__m3
        m4 = self.__thrust_offset + self.__m4

        m1 = m1 + yaw_input + pitch_input + roll_input
        m2 = m2 - yaw_input + pitch_input - roll_input
        m3 = m3 - yaw_input - pitch_input + roll_input
        m4 = m4 + yaw_input - pitch_input - roll_input

        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)
