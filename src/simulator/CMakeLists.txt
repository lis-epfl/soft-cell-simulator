include_directories(${SOFT_SIMULATOR_INCLUDE_DIR} ${BOX2D_INCLUDE_DIR})
link_directories(${BOX2D_LIBRARY_DIR})

file (GLOB_RECURSE
     SIMULATOR_SRC
     *.cpp
     )

add_library(soft-simulator STATIC ${SIMULATOR_SRC})
target_link_libraries(soft-simulator Box2D)
