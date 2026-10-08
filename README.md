ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
        { positions: [0, 0.523599, 0, 1.5708, 0, .785398] , time_from_start: { sec: 5 } },
    ]
    }" 

ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
points: [
    { positions: [0, 0.0, 0, 0., 0, .0] , time_from_start: { sec: 5 } },
]
}" 

ros2 action send_goal --feedback \
  /gen3_lite_2f_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['right_finger_bottom_joint'], position: [0.5]}}"
  