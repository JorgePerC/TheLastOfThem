cmake_minimum_required(VERSION 3.13)

# Definir el nombre del proyecto
project (cMakeHWorld)

message("You are working at:")
message(${CMAKE_SOURCE_DIR})

# The path to the source is relative to the CMakeLists.txt file
#add_executable(binName src/helloWorld.cpp) 

# When working with libraries, you have two options, 

# static or dynamic. Static libraries are compiled into the
    # Add files to the output executable 
add_executable(staticCalc src/helloWorld.cpp libs/myLib.cpp) 
    

# OR...

    # Create the libraries folder, and add .cpp files onto it
set ( PATH_2_LIBS "/home/jpc/Documents/Development/TheLastOfThem/Activites/cmakeHelloWorld/libs" )
add_executable(longStaticCalc src/helloWorld.cpp) 

    # add headers routes
include_directories(
    include
)

# add_library(
#     longLibName 
#         STATIC
#     #include/myLib.hpp
#     ${PATH_2_LIBS}/myLib.cpp
# )

target_link_libraries(
    longStaticCalc
        # PRIVATE
    ${PATH_2_LIBS}/libmyLib.so
)

