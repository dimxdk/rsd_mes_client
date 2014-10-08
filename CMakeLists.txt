cmake_minimum_required(VERSION 2.8.3)
project(rsd_mes_client)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs message_generation 
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   mes_mobile_command.msg
   mes_mobile_status.msg
   mes_sorting_command.msg
   mes_sorting_status.msg
   mes_order.msg
   lego_brick.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   DEPENDS message_runtime std_msgs
)
