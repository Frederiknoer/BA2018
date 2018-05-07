
#include <librealsense2/rs.hpp>
//#include <librealsense2/rs_advanced_mode.hpp>
#include <Eigen/SVD>
#include <cstring>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "Algorithms.h"
#include <functional>
#pragma once

#define IntelD435 "Intel RealSense 435"
struct frmdata
{
   	std::vector<Algorithms::pts> vtx;
    size_t size;
    double timestamp;
};
using namespace rs2;

class rsCam
{
public:
    rsCam();
    rsCam(int x, int y, int fps);
    bool startStream();

    const rs2::vertex* RqSingleFrame();

    rs2::frameset RqDepthFrame();
    Eigen::MatrixXf RqMatrix();
    ~rsCam();

    frmdata RqFrameData(std::vector<float> vec);
private:
    pipeline* _pipe;
    rs2::config _cfg;
    points _pts;
    void filtering(depth_frame& frame, int i);

};
