cmake_minimum_required(VERSION 2.8.3)
project(nao_dcm_msgs)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES
    std_msgs
)

#Declare build dependencies
find_package(catkin REQUIRED 
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

add_message_files(DIRECTORY msg
	FILES
	FSRs.msg
	Bumper.msg
	Tactile.msg
)

add_service_files(DIRECTORY srv
	FILES
	BoolService.srv
)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#Declare package run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})