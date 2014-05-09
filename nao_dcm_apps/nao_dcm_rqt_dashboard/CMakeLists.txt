cmake_minimum_required(VERSION 2.8.3)
project(nao_dcm_rqt_dashboard)

find_package(catkin REQUIRED)
find_package(Qt4 REQUIRED)


catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/nao_dcm_rqt_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
