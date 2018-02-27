#-----------------------------------------------------------------------------------------------
# CMake file for the KUNS library:  /KuObstAvoidancePr
#
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(LibraryName KuObstAvoidancePr)
SET(PRJ_NAME "${LibraryName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})

# These commands are needed by modern versions of CMake:
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# ---------------------------------------------------------------------------
# Set the output directory 
# ---------------------------------------------------------------------------
SET(LIBRARY_OUTPUT_PATH "${KUNS_LIB_DIR}")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the library target:
ADD_LIBRARY(${LibraryName}
 KuObstAvoidancePr.h
 KuObstAvoidancePr.cpp ) 

SET_TARGET_PROPERTIES(
	${LibraryName} 
	PROPERTIES 
	PROJECT_LABEL "(KUNS_PROCESS_LIB) ${LibraryName}"
	DEBUG_POSTFIX "_dbg"	
					
	)

IF( CMAKE_BUILD_TYPE OR MSVC) 		#for (M)icro(S)oft (V)isual (C)++ 
	IF( MSVC )
        set( CMAKE_FIND_LIBRARY_PREFIXES "" )
        set( CMAKE_FIND_LIBRARY_SUFFIXES ".lib" )
		
		# Add the required libraries for linking:
		TARGET_LINK_LIBRARIES(${LibraryName} 
								KuMap 
								KuPose 
								KuMath 
								KuUtil
								KuLaserMapBuilderPr
								KuGradientPathPlannerbasedPOMapPr
								KuPathSmoothingPr KuGradientPathPlannerPr
								debug ${KUNS_EXTRA_WIN32_LIBS_DIR}/newmat/Debug/newmat11d.lib							
								optimized ${KUNS_EXTRA_WIN32_LIBS_DIR}/newmat/Release/newmat11.lib
								)		
				
    ENDIF( MSVC )
ELSE(CMAKE_BUILD_TYPE OR MSVC)
        set( CMAKE_FIND_LIBRARY_PREFIXES "lib" )
    set( CMAKE_FIND_LIBRARY_SUFFIXES ".a" )

      # set(LIBRARY_OUTPUT_PATH ../../../lib)
    #  FIND_LIBRARY(KUNSUtil_LIB NAMES  KuUtil  PATHS ${LIBRARY_OUTPUT_PATH})
    #  FIND_LIBRARY(KUNSPOSE_LIB NAMES  KuPose  PATHS ${LIBRARY_OUTPUT_PATH})
    #   FIND_LIBRARY(KUNSMATH_LIB NAMES  KuMath  PATHS ${LIBRARY_OUTPUT_PATH})
    #    FIND_LIBRARY(KUNSMAP_LIB NAMES  KuMap  PATHS ${LIBRARY_OUTPUT_PATH})

    #   TARGET_LINK_LIBRARIES(${LibraryName} ${KUNSUtil_LIB})
    #   TARGET_LINK_LIBRARIES(${LibraryName}  ${KUNSPOSE_LIB})
    #   TARGET_LINK_LIBRARIES(${LibraryName}  ${KUNSMATH_LIB})
    #   TARGET_LINK_LIBRARIES(${LibraryName}  ${KUNSMAP_LIB})

        # Add the required libraries for linking:
        TARGET_LINK_LIBRARIES(${LibraryName} KuMap KuPose KuMath KuUtil
         KuLaserMapBuilderPr
        KuGradientPathPlannerbasedPOMapPr KuPathSmoothingPr KuGradientPathPlannerPr)
        					  	  
ENDIF(CMAKE_BUILD_TYPE OR MSVC)




# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
