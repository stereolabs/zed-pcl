# zed-PCL

**This sample is designed to work with the ZED stereo camera only and requires the ZED SDK. For more information: https://www.stereolabs.com**

It demonstrates how to grab a point cloud with the ZED SDK and link the results with the Point Cloud Library (https://github.com/PointCloudLibrary/pcl).

**Warning:**
 - This sample is not designed to operate in real time
     
The color point cloud is converted and then displayed using PCLVisualizer. The 'grab' function of the ZED SDK in charge of the disparity map computation and 3D projection run in a thread.                                
##Build the program

Open a terminal in 'zed-pcl' directory and execute the following command:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
 

##Run the program

Open a terminal in build directory and execute the following command:

    $ ./ZED\ with\ PCL [path to SVO file]
