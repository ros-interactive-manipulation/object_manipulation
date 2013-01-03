cmake_minimum_required(VERSION 2.8.3)
project(household_objects_database_msgs)

find_package(catkin REQUIRED COMPONENTS arm_navigation_msgs geometry_msgs sensor_msgs message_generation)

add_message_files(
  DIRECTORY msg
  FILES 
  DatabaseModelPoseList.msg
  DatabaseModelPose.msg
  DatabaseReturnCode.msg
  DatabaseScan.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetModelDescription.srv
  GetModelList.srv
  GetModelMesh.srv
  GetModelScans.srv
  SaveScan.srv
  TranslateRecognitionId.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs arm_navigation_msgs sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime arm_navigation_msgs geometry_msgs sensor_msgs
)
