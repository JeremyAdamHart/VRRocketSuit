cmake_minimum_required(VERSION 3.20)
project(rocketsuit)

add_executable(rocketsuit_exe
    ${CMAKE_CURRENT_SOURCE_DIR}/source/HeatParticleGeometry.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/HeatParticleMat.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/HeatParticleShader.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/HeatParticleSystem.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/Physics.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/VRCamera.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/VRController.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/VRRocketSuit.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/VRWindow.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/source/VRTools.cpp
)

target_include_directories(rocketsuit_exe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source/)

find_package(jorl REQUIRED COMPONENTS jorl::exts)
find_package(openvr REQUIRED)

# Packages
target_link_libraries(rocketsuit_exe PUBLIC jorl::core jorl::exts openvr::openvr)
