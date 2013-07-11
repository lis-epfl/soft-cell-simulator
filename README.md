Soft Cell Simulator
=================

TODO: Short Description goes here

## Compile

The installation/building process of SCS comprises two stages:

1. Installation of the Box2D Physics Engine
2. Building of Soft Cell Simulator

I will assume the software to be compiled/installed on Linux Ubuntu 

### Install Box2D

1. Download the latest version of Box2D [here](http://code.google.com/p/box2d/) (Current version tested is 2.2.1) 
2. Unzip the Box2D archive, into a temporary directory BOX2D_FOLDER
3. Browse into the BOX2D_FOLDER/Build directory and compile Box2D. You can modify the preferred installation path 
by modifying the value of the CMAKE_INSTALL_PREFIX variable

    ```
    cmake -DCMAKE_BUILD_TYPE=Release -DBOX2D_INSTALL=ON -DBOX2D_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX=~/install_dir .. 
    make
    make install
    ```
    
### Build the Soft Cell Simulator

1. Install some required dependencies (freeglut, glui and glew)

   ```
   sudo apt-get install libglew1.6-dev freeglut3-dev libglui-dev
   ```

2. Create a folder for SCS, we will refer to as SCS_FOLDER
3. Download the SCS code in SCS_FOLDER

   ```
   cd SCS_FOLDER
   git clone https://github.com/amaesani/scs.git
   ```

4. Create a build folder in SCS_FOLDER, e.g. SCS_FOLDER/Build

   ```
   mkdir Build
   ```

5. Modify the CMakeLists.txt located in SCS_FOLDER/src/CmakeLists.txt, and update the correct paths for BOX2D_LIBRARY_DIR and
BOX2D_INCLUDE_DIR, pointing to paths where you installed the Box2D engine include files and libraries
6. Run cmake from the Build folder

    ```
    cd Build
    cmake -DCMAKE_BUILD_TYPE=Release ../src
    ```

7) Try out the examples located in SCS_FOLDER/Build/examples and have fun!! :-)
    

## Examples

TODO: Detailed Description of examples goes here
