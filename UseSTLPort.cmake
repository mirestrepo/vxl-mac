# STLPort settings

OPTION(USE_STLPORT "Use STLPort?" NO) 

OPTION(LINK_DYNAMIC_RUNTIMES "Link to dynamic runtimes?" NO) 

IF(USE_STLPORT)

  IF(USE_STLPORT)
    SET(STLPORT $ENV{STLPORT} CACHE STRING "STLPORT")
    INCLUDE_DIRECTORIES( ${STLPORT}/stlport )
    LINK_DIRECTORIES ( ${STLPORT}/lib )
  ENDIF(USE_STLPORT)
  
  IF(WIN32)
    IF(CYGWIN)
      IF(USE_STLPORT)
          LINK_LIBRARIES( debug ${STLPORT}/lib/libstlport_cygwin_stldebug.a )
          LINK_LIBRARIES( optimized ${STLPORT}/lib/libstlport_cygwin.a )
          SET (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_STLP_DEBUG")
  #        SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_STLP_DO_IMPORT_CSTD_FUNCTIONS")
      ENDIF(USE_STLPORT)
    ELSE(CYGWIN)
      IF(LINK_DYNAMIC_RUNTIMES)
          SET(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /G7")
          SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /Z7 /O2")
          SET(CMAKE_CXX_FLAGS_MINSIZEREL "/MD /O1")
          SET(CMAKE_CXX_FLAGS_DEBUG "/MDd /Z7 /Od /GZ")
          SET(CMAKE_C_FLAGS_RELEASE "/MD /O2")
          SET(CMAKE_C_FLAGS_RELWITHDEBINFO "/MD /Z7 /O2")
          SET(CMAKE_C_FLAGS_MINSIZEREL "/MD /O1")
          SET(CMAKE_C_FLAGS_DEBUG "/MDd /Z7 /Od /GZ")
        IF(USE_STLPORT)
          SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D \"_RTLDLL\"")
        ENDIF(USE_STLPORT)
      ELSE(LINK_DYNAMIC_RUNTIMES)
          SET(CMAKE_CXX_FLAGS_RELEASE "/MT /O2 /Ob2 /G7")
          SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MT /Z7 /O2")
          SET(CMAKE_CXX_FLAGS_MINSIZEREL "/MT /O1")
          SET(CMAKE_CXX_FLAGS_DEBUG "/MTd /Z7 /Od /GZ")
          SET(CMAKE_C_FLAGS_RELEASE "/MT /O2"")
          SET(CMAKE_C_FLAGS_RELWITHDEBINFO "/MT /Z7 /O2")
          SET(CMAKE_C_FLAGS_MINSIZEREL "/MT /O1")
          SET(CMAKE_C_FLAGS_DEBUG "/MTd /Z7 /Od /GZ")
      ENDIF(LINK_DYNAMIC_RUNTIMES)
  
      IF(USE_STLPORT)
        SET (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /D \"_STLP_DEBUG\"")
        SET (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /D \"_STLP_DEBUG\"")
        SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"VCL_STLPORT\"")
  #      SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"_STLP_DO_IMPORT_CSTD_FUNCTIONS\"")
      ENDIF(USE_STLPORT)
  
      INCLUDE_DIRECTORIES(${VXLSRC}/vcl/config.win32 )
    ENDIF(CYGWIN)
  ELSE(WIN32)
      IF(USE_STLPORT)
        SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVCL_STLPORT -pthread -fexceptions")
        SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVCL_STLPORT -pthread -fexceptions")
  #     SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_STLP_DO_IMPORT_CSTD_FUNCTIONS")
  #      IF(CMAKE_COMPILER_IS_GNUCXX)
            LINK_LIBRARIES( pthread )
  
  
          IF(CMAKE_BUILD_TYPE MATCHES "Debug")
              LINK_LIBRARIES( ${STLPORT}/lib/libstlport_gcc_stldebug.a )
          ELSE(CMAKE_BUILD_TYPE MATCHES "Debug")
              LINK_LIBRARIES( ${STLPORT}/lib/libstlport_gcc.a )
          ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
  
          SET (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_STLP_DEBUG")
          SET (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_STLP_DEBUG")
   #     ENDIF(CMAKE_COMPILER_IS_GNUCXX)
      ENDIF(USE_STLPORT)
  ENDIF(WIN32)

ENDIF(USE_STLPORT)
