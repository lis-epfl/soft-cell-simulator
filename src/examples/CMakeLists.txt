# Some flags for Freeglut and GLUI.
add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

# Define the framework files.
file(GLOB_RECURSE 
Testbed_Framework_SRCS
testbed/*.cpp)
  
#define the test files.
file(GLOB_RECURSE 
Testbed_Tests_SRCS
example/*.cpp)
  
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

include_directories (
	${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_PATH}
	${GLUI_INCLUDE_DIR} 
	${BOX2D_INCLUDE_DIR}
   ${SOFT_SIMULATOR_INCLUDE_DIR}
)

link_directories(${BOX2D_LIBRARY_DIR})

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib )
	set (OPENGL_LIBRARIES GL GLU GLUT X11)
endif(APPLE)

add_executable(example
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

target_link_libraries (
	example
	Box2D
	glui ${GLUT_LIBRARY} ${GLEW_LIBRARY} ${OPENGL_LIBRARIES}
	soft-simulator
) 
