set(_srcs DynamixelV1.cpp DynamixelV2.cpp)

set(test_srcs test.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libaqua/include)

find_package(Threads)

add_library(${PROJECT_NAME} SHARED ${_srcs})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})


add_executable(${PROJECT_NAME}_test ${test_srcs})
target_link_libraries(${PROJECT_NAME}_test ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME})

if(WIN32)
 set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT.LIB\"")
endif()

add_library(${PROJECT_NAME}Static STATIC ${_srcs})
target_link_libraries(${PROJECT_NAME}Static ${CMAKE_THREAD_LIBS_INIT})

set_property(TARGET ${PROJECT_NAME}Static APPEND PROPERTY COMPILE_DEFINITIONS EXPORT_${PROJECT_NAME}_STATIC)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}Static
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT component
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT component
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT component)
install(EXPORT ${PROJECT_NAME}
    DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Depends.cmake)
