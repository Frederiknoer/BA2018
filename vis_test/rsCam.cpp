#include "rsCam.h"

rsCam::rsCam(){};
rsCam::rsCam(int x, int y, int fps)
{
    context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0)
        throw std::runtime_error("No device detected");
    device dev = list.front();
    //std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    if (std::strcmp(dev.get_info(RS2_CAMERA_INFO_NAME),IntelD435)!= 0)
    {
        std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << " is currently connected" << std::endl;
        throw std::runtime_error("Unknown device connected");
    }
    //config cfg;
    _cfg.enable_stream(RS2_STREAM_DEPTH,x,y,RS2_FORMAT_Z16,fps);

    auto depth_sens = dev.first<depth_sensor>();
    depth_sens.set_option(RS2_OPTION_LASER_POWER, 360);

    _pipe = new pipeline();

}
bool rsCam::startStream()
{
    _pipe->start(_cfg);
    for (int i = 0; i < 30; i++)_pipe->wait_for_frames();
    return true;
}
const rs2::vertex* rsCam::RqSingleFrame()
{
    const vertex* list;
    for (auto&& frames : _pipe->wait_for_frames()) {
        if (auto depth = frames.as<depth_frame>()) {
            filtering(depth, 15);
            pointcloud pc;
            _pts = pc.calculate(depth);
            list = _pts.get_vertices();
        }
    }
    if (list != nullptr)
    {
        return list;
    }
    else throw std::runtime_error("Can't fint dpeth stream");
}

rs2::frameset rsCam::RqDepthFrame()
{
    for (auto frames : _pipe->wait_for_frames()) {
        //if (auto depth = frames.as<rs2::frame>()) {
            return frames;
        //}
    }
}
frmdata rsCam::RqFrameData()
{
    frmdata fd;

    for (auto&& frames : _pipe->wait_for_frames()) {
        if (auto depth = frames.as<depth_frame>()) {
            //filtering(depth, 10);
            pointcloud pc;
            _pts = pc.calculate(depth);
            fd.vtx = _pts.get_vertices();
            fd.size = _pts.size();
            fd.timestamp = depth.get_timestamp();
        }
    }
    if (fd.vtx != nullptr)
    {
        return fd;
    }
    else throw std::runtime_error("Can't fint dpeth stream");
}
Eigen::MatrixXf rsCam::RqMatrix()
{
    const vertex* list;
    Eigen::MatrixXf temp;
    for (auto&& frames : _pipe->wait_for_frames())
    {
        if ( auto depth = frames.as<depth_frame>())
        {
            pointcloud pc;
            points pts = pc.calculate(depth);
            list = pts.get_vertices();

        }
    }
    int i = 0;
    while (i < 10)
    {
        //static int i = 0;

        temp(i,0) = list[i].x;
        temp(i,1) = list[i].y;
        temp(i,2) = list[i].z;
        temp(i,3) = 1.0f;
        std::cout << i << std::endl;
        i++;
    }
    return temp;
}
void rsCam::filtering(depth_frame& frame, int i)
{
    disparity_transform depth2Disparity;
    disparity_transform disparity2Depth(false);

    spatial_filter spat;
    temporal_filter temp;

    spat.set_option(RS2_OPTION_HOLES_FILL, 4);
    frame = depth2Disparity.process((frame));
    for (int j = 0; j < i;j++)
        for(auto && frames : _pipe->wait_for_frames())
        {
            if ( auto disp = frames.as<disparity_frame>())
            {
                frame = temp.process(disp);
            }
        }
    frame = spat.process(frame);
    frame = disparity2Depth.process(frame);
}
rsCam::~rsCam()
{
    _pipe->stop();
    delete _pipe;
}