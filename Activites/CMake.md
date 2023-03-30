# CMake

Make easier to compile complex projects 


On the low level, it calls for gcc, cxx...

    Note: 
        ccmake ..
        # Was design to automate all this
        cmake-gui
        # Graphical interface for configuring CMakeFiles

## Steps:
 1. Have the following directory structure: ![folderStructure](https://miro.medium.com/v2/resize:fit:319/1*dAo2jf6-a-KH1VQBg1hsoA.png)

 1. Configure your CMakeLists.txt file
        
        nano CMakeLists.txt

 1. Move into the build directory. This because it's a good practice to keep the output files in there

        cd build

 1. Create make config files *Makefile, CMakeFiles/, CMakeCache.txt*
    
        cmake ../Path/to/project

 1. Build project
    
        cmake --build projectDir

## CMakeLists Commands:

### cmake_minimum_required()
Defines the cmake version needed to compile the project
    
    cmake_minimum_required(VERSION 3.13)

*This is obligatory for each CMakeLists.txt*

### project()
Declares project name, in ROS, this is usually your package

    project (cMakeHWorld)

*This is obligatory for each CMakeLists.txt*

### include_directories()
Defines header location on disk

### find_package()
CMake looks on the disk the location of a library.
This only works if the library has the 'Config.cmake' file on its files

### add_executable()
List source files that binaries will be created upon. The path to the source is relative to the CMakeLists.txt file
    
    add_executable(binName src/helloWorld.cpp)

*This is obligatory for each CMakeLists.txt*

### set()
Create variables for the compilation project
    
    set (varName "my/value/thatIsUsusally/aPath")
    include_directories (
        include
        ${varName}$
    )
