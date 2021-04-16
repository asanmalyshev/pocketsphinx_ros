cmake_minimum_required(VERSION 3.0.2)
project(pocketsphinx_ros)
# cmake_policy(NEW)
# project(pocketsphinx VERSION 0.1)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    message_generation
)

add_message_files(
  FILES
  DecodedPhrase.msg  
)

add_service_files(
  FILES
  PocketsphinxControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(DEPENDS)

# catkin_package()

# install(DIRECTORY demo
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/decoder.py scripts/send_audio.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
