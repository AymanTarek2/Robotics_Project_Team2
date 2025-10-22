#!/usr/bin/env python3
import os
import rospkg

if __name__ == "__main__":
    rp = rospkg.RosPack()
    p = rp.get_path('my_robot_gazebo')
    print("my_robot_gazebo path:", p)
    print("URDF default path:", os.path.join(p, 'urdf', 'robot.urdf'))
