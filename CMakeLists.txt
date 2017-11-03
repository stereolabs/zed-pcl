SET(execName ZED_with_PCL)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")
SET(VERSION_REQ_CUDA "8.0")

IF(WIN32) # Windows
    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
    SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
ELSE() # Linux
    find_package(ZED 2 REQUIRED)
  
    ##For Jetson, OpenCV4Tegra is based on OpenCV2.4
    exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
    if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
        SET(VERSION_REQ_CUDA "")
        unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
        option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
        message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
        ##to prevent from opencv_dep_cudart dependencies error...
        ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.
    endif()
ENDIF(WIN32)

find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(PCL)

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

	add_definitions(-std=c++0x -g -O3) # -m64)
	list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)

	TARGET_LINK_LIBRARIES(${execName}
		                ${ZED_LIBRARIES}
		                ${OpenCV_LIBRARIES}
		                ${PCL_LIBRARIES}
		                ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY} )
else()
	message(WARNING "PCL not found, sample not build")
endif()
