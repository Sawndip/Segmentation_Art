CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(SegmentByThreeDiff)
SET(DEMO_VERSION_MAJOR 1)
SET(DEMO_VERSION_MINOR 0)
SET(CMAKE_VERBOSE_MAKEFILE off)

# we make binary
SET(segthree segthree.out)
SET(testPso pso.out)
SET(testVector vector.out)

# get compile time
EXECUTE_PROCESS(
    COMMAND date "+%Y.%m.%d %H:%M:%S"
    #COMMAND awk "{print $2 \"-\"$3-\"-\"$4}"
    OUTPUT_VARIABLE compileTime
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# get svn version
EXECUTE_PROCESS(
    COMMAND svn info ../
    COMMAND grep -i Revision
    COMMAND cut -d " " -f 2
    OUTPUT_VARIABLE svnVersion
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

MESSAGE("CMakeLists Foler: " ${CMAKE_CURRENT_SOURCE_DIR})
MESSAGE("Compile Time: " ${compileTime})
MESSAGE("Svn Version: " ${svnVersion})
get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} PATH)
MESSAGE("Pontus Folder: " ${PARENT_DIR})

# add macros
ADD_DEFINITIONS(-DCOMPILE_TIME="${compileTime}")
ADD_DEFINITIONS(-DSVN_VER="${svnVersion}")

FIND_PACKAGE (Boost REQUIRED)
LINK_DIRECTORIES (${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})

# included directories
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../pontus/vpcore/tools/opticalFlow 
                   )
# add linking directory
LINK_DIRECTORIES (${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_CXX_FLAGS ${CMAKE_C_FLAGS} "-g -O2 -Wall -std=c++0x")
ADD_EXECUTABLE(${segthree} ${CMAKE_CURRENT_SOURCE_DIR}/segControl.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/threeDiff.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/boundaryScan.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/psoBook.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/contourTrack.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/CompressiveTracker.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/../pontus/vpcore/tools/opticalFlow/VarFlowWA.cpp 
                           ${CMAKE_CURRENT_SOURCE_DIR}/testThree.cpp)
                         
ADD_EXECUTABLE(${testPso} ${CMAKE_CURRENT_SOURCE_DIR}/psoBook.cpp
                          ${CMAKE_CURRENT_SOURCE_DIR}/testPso.cpp)

ADD_EXECUTABLE(${testVector} ${CMAKE_CURRENT_SOURCE_DIR}/testVector.cpp)

SET(bins ${testVector} ${testPso} ${segthree})
foreach(bin ${bins})
  TARGET_LINK_LIBRARIES(${bin} opencv_calib3d opencv_contrib opencv_core opencv_features2d
                               opencv_flann opencv_highgui opencv_imgproc 
                               opencv_legacy opencv_ml opencv_objdetect 
                               opencv_photo opencv_stitching opencv_superres
                               opencv_video opencv_videostab)
  TARGET_LINK_LIBRARIES(${bin} boost_system boost_thread)
  TARGET_LINK_LIBRARIES(${bin} numa pthread m dl lzma bz2 rt)    
endforeach()

## options we can use
#CMAKE_EXE_LINKER_FLAGS
#CMAKE_MODULE_LINKER_FLAGS
#CMAKE_SHARED_LINKER_FLAGS
#CMAKE_STATIC_LINKER_FLAGS

##################### END OF THE FILE #########################
###############################################################
