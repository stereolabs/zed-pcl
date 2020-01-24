SET(execName ZED_with_PCL)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(PCL REQUIRED)

if(PCL_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${ZED_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    link_directories(${ZED_LIBRARY_DIR})
    link_directories(${CUDA_LIBRARY_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})

    FILE(GLOB_RECURSE SRC_FILES src/*.cpp) 
    ADD_EXECUTABLE(${execName} ${SRC_FILES})

    add_definitions(-std=c++14 -g -O3)
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)

    if (LINK_SHARED_ZED)
        SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
    else()
        SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
    endif()
    
    TARGET_LINK_LIBRARIES(${execName} 
                            ${ZED_LIBS} 
                            ${OpenCV_LIBRARIES} 
                            ${PCL_LIBRARIES})                            
endif()
