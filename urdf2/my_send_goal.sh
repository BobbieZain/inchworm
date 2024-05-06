ros2 action send_goal /top_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Slider_2, Slider_6],
    points: [
      { positions: [0.135,0.135], time_from_start: { sec: 0, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 5, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 10, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 15, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 20, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 25, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 30, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 35, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 40, nanosec: 0 } }
    ]
  }
}"

ros2 action send_goal /mid_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Slider_9],
    points: [
      { positions: [0, 0], time_from_start: { sec: 0, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 5, nanosec: 0 } },
      { positions: [0.15,0.15], time_from_start: { sec: 10, nanosec: 0 } },
      { positions: [0.15,0.15], time_from_start: { sec: 15, nanosec: 0 } },
      { positions: [0.15,0.15], time_from_start: { sec: 20, nanosec: 0 } },
      { positions: [[0, 0], time_from_start: { sec: 25, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 30, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 35, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 40, nanosec: 0 } }
    ]
  }
}"

ros2 action send_goal /bot_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Slider_7, Slider_8],
    points: [
      { positions: [0.135,0.135], time_from_start: { sec: 0, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 5, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 10, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 15, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 20, nanosec: 0 } },
      { positions: [0, 0], time_from_start: { sec: 25, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 30, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 35, nanosec: 0 } },
      { positions: [0.135,0.135], time_from_start: { sec: 40, nanosec: 0 } }
    ]
  }
}"

