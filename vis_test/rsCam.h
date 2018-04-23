#ifndef VOLEST_RSCAM_H
#define VOLEST_RSCAM_H

#include <librealsense2/rs.hpp>
//#include <librealsense2/rs_advanced_mode.hpp>
#include <Eigen/SVD>
#include <cstring>
#include <iostream>

#define IntelD435 "Intel RealSense 435"


using namespace rs2;

class rsCam
{
public:
    rsCam();
    rsCam(int x, int y, int fps);
    bool startStream();

    const vertex* RqSingleFrame();

    const void* RqSingleRGB();
    Eigen::MatrixXf RqMatrix();
    ~rsCam();

private:
    pipeline* _pipe;
    rs2::config _cfg;


    pipeline* _pipe2;
    rs2::config _rgb;
    points _pts;
    void filtering(depth_frame& frame, int i);

};

#endif //VOLEST_RSCAM_H
