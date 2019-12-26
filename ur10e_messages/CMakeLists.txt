cmake_minimum_required(VERSION 2.8.3)
project(ur10e_messages)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  State.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   GetState.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


catkin_package(
 CATKIN_DEPENDS message_runtime actionlib_msgs
)
