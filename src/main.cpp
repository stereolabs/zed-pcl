///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/************************************************************************************
 ** This sample demonstrates how to use PCL (Point Cloud Library) with the ZED SDK **
 ************************************************************************************/


// standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>

// OpenCV includes
#include <opencv2/opencv.hpp>

// ZED includes
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#ifdef _WIN32
#undef max
#undef min
#endif

// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace sl::zed;
using namespace std;


// Exchange structure

typedef struct image_bufferStruct {
    float* data_cloud;
    std::mutex mutex_input;
    int width, height;
} image_buffer;


Camera* zed;
image_buffer* buffer;
SENSING_MODE dm_type = STANDARD;
bool stop_signal;

// Grabbing function

void grab_run() {
    float* p_cloud;

    while (!stop_signal) {
        if (!zed->grab(dm_type)) {
            p_cloud = (float*) zed->retrieveMeasure(MEASURE::XYZRGBA).data; // Get the pointer
            // Fill the buffer
            buffer->mutex_input.lock(); // To prevent from data corruption
            memcpy(buffer->data_cloud, p_cloud, buffer->width * buffer->height * sizeof (float) * 4);			
			buffer->mutex_input.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

std::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
    return (viewer);
}

inline void cvtColor(float color0, uchar &r_, uchar &g_, uchar &b_) {
	unsigned char * ptr_im;
	ptr_im = (unsigned char *)&color0;
	r_ = ptr_im[0];
	g_ = ptr_im[1];
	b_ = ptr_im[2];
}

int main(int argc, char** argv) {
    stop_signal = false;

    if (argc > 2) {
        std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
        return -1;
    }

    if (argc == 1) // Use in Live Mode
        zed = new Camera(HD720);
    else // Use in SVO playback mode
        zed = new Camera(argv[1]);
	
    sl::zed::InitParams params;
    params.mode = PERFORMANCE;
	params.unit = METER;										// scale to fit openGL world
	params.coordinate = RIGHT_HANDED;		// openGL compatible
    params.verbose = true;

    ERRCODE err = zed->init(params);
    cout << errcode2str(err) << endl;
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }

	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;
	
    // allocate data
    buffer = new image_buffer();
    buffer->height = height;
    buffer->width = width;
    buffer->data_cloud = new float[buffer->height * buffer->width * 4];

    int size = height*width;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis(point_cloud_ptr);

    float* data_cloud;
    // Run thread
    std::thread grab_thread(grab_run);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

	float color;
    int index4 = 0;

    point_cloud_ptr->points.resize(size);

    while (!viewer->wasStopped()) {

        if (buffer->mutex_input.try_lock()) {
            data_cloud = buffer->data_cloud;
            index4 = 0;

            for (auto &it : point_cloud_ptr->points){
				float X = data_cloud[index4*4];
				if (!isValidMeasure(X)) // Checking if it's a valid point
                    it.x = it.y = it.z = it.rgb = 0;
				else{
					it.x =X;
					it.y = data_cloud[index4 * 4 + 1];
					it.z = data_cloud[index4 * 4 + 2];
					color = data_cloud[index4 * 4 + 3];
					uint32_t color_uint = *(uint32_t*)& color;
					unsigned char* color_uchar = (unsigned char*)&color_uint;
					color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
					it.rgb = *reinterpret_cast<float*> (&color_uint);
				}
				index4++;
            }
            buffer->mutex_input.unlock();
            viewer->updatePointCloud(point_cloud_ptr);
            viewer->spinOnce(15);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Stop the grabbing thread
    stop_signal = true;
    grab_thread.join();

    delete[] buffer->data_cloud;
    delete buffer;
    delete zed;
    return 0;
}
