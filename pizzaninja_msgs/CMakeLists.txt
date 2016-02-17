cmake_minimum_required(VERSION 2.8.3)
project(pizzaninja_msgs)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MSG_DEPS
  std_msgs
)

set(MSG_FILES
  SegGoal.msg
)

set(SRV_FILES
  SetEnable.srv
)

set(ACT_FILES
)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEPS}
  message_generation
)

#add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS} message_runtime)

