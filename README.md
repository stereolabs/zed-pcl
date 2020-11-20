# Stereolabs ZED - PCL

This sample shows how to acquire and display a 3D point cloud with PCL (Point Cloud Library).


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- Make sure you have [PCL](https://github.com/PointCloudLibrary/pcl/releases) installed with its visualization module.
- For more information, read the ZED [Documentation](https://www.stereolabs.com/docs/) and [API documentation](https://www.stereolabs.com/docs/api/) or our [Community page](https://community.stereolabs.com)

### Prerequisites

- Windows 10, Ubuntu 16.04
- [ZED SDK **3**](https://www.stereolabs.com/developers/) and its dependency ([CUDA](https://developer.nvidia.com/cuda-downloads))

## Build the program

Download the sample and follow the instructions below: [More](https://www.stereolabs.com/docs/getting-started/application-development/)

#### Build for Windows

  - Create a folder called "build" in the source folder
  - Open cmake-gui and select the source and build folders
  - Generate the Visual Studio `Win64` solution
  - Open the resulting solution and change the solution configuration to `Release`
  - Build solution

#### Build for Linux

Open a terminal in the sample directory and execute the following command:

    mkdir build
    cd build
    cmake ..
    make

## Run the program

- Navigate to the build directory and launch the executable file
- Or open a terminal in the build directory and run the sample :

        ./ZED\ with\ PCL [path to SVO file]

You can optionally provide an SVO file path (recorded stereo video of the ZED)


### Troubleshooting
 - PCL view can start zoomed in, displaying only green and red colors. To adjust the viewport, simply zoom out using your mouse wheel or keyboard shortcuts.

### Limitations
- The point cloud conversion to PCL format is time consuming and will affect application running time.

## Support
If you need assistance go to our Community site at https://community.stereolabs.com/
