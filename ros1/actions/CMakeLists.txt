find_package(catkin REQUIRED COMPONENTS
 actionlib
 genmsg
 actionlib_msgs
 rospy
)

catkin_install_python(PROGRAMS
    scripts/countdown_server.py
    scripts/countdown_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Before catkin_package()
add_action_files(DIRECTORY action FILES Countdown.action)
generate_messages(DEPENDENCIES actionlib_msgs)

# catkin_package()
