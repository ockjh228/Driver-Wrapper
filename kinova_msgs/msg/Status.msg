# Status.msg
# Overall status of the robot including operating mode, pose, and actuator feedback

OperatingMode operating_mode            # Operating and servoing mode of the robot
ServoingMode servoing_mode              # Servoing mode of the robot
CurrentPose cartesian_pose              # Cartesian pose of the tool
ActuatorFeedback[] actuator_feedback    # Array of feedback from each actuator
