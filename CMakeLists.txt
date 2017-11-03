SET(execName ZED_with_PCL)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2 REQUIRED)
find_package(CUDA REQUIRED)
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

	add_definitions(-std=c++11 -g -O3)
	list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)

	TARGET_LINK_LIBRARIES(${execName}
		                ${ZED_LIBRARIES}
		                ${OpenCV_LIBRARIES}
		                ${PCL_LIBRARIES}
		                ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} )
else()
	message(WARNING "PCL not found, sample not build")
endif()
