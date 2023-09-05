# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

# ===================================
#      PROJECT SETUP
# ===================================
project(TestDemo)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ===================================
#      CONFIGURE ALL EXAMPLES
# ===================================
# examples bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Tool) 

# list of examples
set(TOOL_LIST
  
  display_robot_states
  CollectData
)

foreach(Tool ${TOOL_LIST})
  # create executable from source
  add_executable(${Tool} Tool/${Tool}.cpp)
  # link dependencies
  target_include_directories(${Tool} 
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen3
  )

  target_link_libraries(${Tool} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.x86_64-linux-gnu.a
      anl
      # fmt
      Threads::Threads
  )
endforeach()



