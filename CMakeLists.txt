cmake_minimum_required(VERSION 3.10)

project(MyCppApp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
	venthmi
	src/main.cpp
	rei/hard/serial/serial.cpp
	rei/tools/ringbuffer/ringbuffer.cpp
)

target_include_directories(venthmi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})