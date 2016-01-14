#!/bin/bash
rosrun baxter_examples send_urdf_fragment.py -f /home/nxr-baxter/indigows/src/baxter_common/baxter_description/urdf/left_end_effector.urdf.xacro -l left_hand -j left_gripper_base & 
pidleft=$!
rosrun baxter_examples send_urdf_fragment.py -f /home/nxr-baxter/indigows/src/baxter_common/baxter_description/urdf/right_end_effector.urdf.xacro -l right_hand -j right_gripper_base &
pidright=$!
echo "Sleeping"
sleep 5
echo "Waking up"
echo "Nodes started:"
echo $(rosnode list |grep --color=always configure_urdf)
kill -2 $pidleft
kill -2 $pidright
echo "Nodes after kill:" 
echo $(rosnode list |grep --color=always configure_urdf)

