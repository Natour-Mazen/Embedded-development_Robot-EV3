cmake_minimum_required(VERSION 3.7)
project(TP_Robot C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_executable(TP_Robot
        communication.c
        communication.h
        main.c
        mdd.c
        mdd.h
        myev3.c
        myev3.h
        time_util.c
        time_util.h
        workers.c
        workers.h)
