#include "rsCam.h"

rsCam::rsCam(){};
rsCam::rsCam(int x, int y, int fps, int CP)
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

    //auto depth_sens = dev.first<depth_sensor>();
    //depth_sens.set_option(RS2_OPTION_LASER_POWER, 360);
	_roi = new roi_sensor(dev.first<rs2::depth_sensor>());
    _pipe = new pipeline(ctx);

	if (CP == 2 && x == 1280)
	{
		std::ifstream t("XRAY1280x720x30.json");
		std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
		rs400::advanced_mode deva = dev;
		deva.load_json(str);
		std::cout << "JSON loaded" << std::endl;
	}
    else if (CP == 2 && x == 848 && fps == 15)
    {
        std::ifstream t("XRAY848x480x15.json");
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        rs400::advanced_mode deva = dev;
        deva.load_json(str);
        std::cout << "JSON loaded" << std::endl;
    }
    else if (CP == 2 && x == 848 && fps == 60)
    {
        std::ifstream t("XRAY848x480x60.json");
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        rs400::advanced_mode deva = dev;
        deva.load_json(str);
        std::cout << "JSON loaded" << std::endl;
    }

}
bool rsCam::startStream()
{
    _pipe->start();
    for (int i = 0; i < 30; i++)_pipe->wait_for_frames();
    return true;
}
const rs2::vertex* rsCam::RqSingleFrame()
{
    const vertex* list;
    for (auto&& frames : _pipe->wait_for_frames()) {
        if (auto depth = frames.as<depth_frame>()) {
            filtering(depth, 30);
            pointcloud pc;
            _pts = pc.calculate(depth);
            list = _pts.get_vertices();
        }
    }
    if (list != nullptr)
    {
        return list;
    }
    else throw std::runtime_error("Can't find depth stream");
}

rs2::frameset rsCam::RqDepthFrame()
{
    for (auto frames : _pipe->wait_for_frames()) {
        //if (auto depth = frames.as<rs2::frame>()) {
            return frames;
        //}
    }
}

frmdata rsCam::RqFrameData(std::vector<float> vec)
{
	decimation_filter deci;
	deci.set_option(RS2_OPTION_FILTER_MAGNITUDE,2.0);
	frmdata fd;
	float minY = vec[0] - RSy/2;
    float maxY = vec[2] - RSy/2;
    float minX = vec[1] - RSx/2;
    float maxX = vec[3] - RSx/2;
    for (auto&& frames : _pipe->wait_for_frames()) {
        if (auto depth = frames.as<depth_frame>()) {
            //filtering(depth,0);
            //depth = deci.process(depth);
            pointcloud pc;
            _pts = pc.calculate(depth);
            auto arr =  _pts.get_vertices();
            fd.size = _pts.size();
            fd.timestamp = depth.get_timestamp();
			Algorithms::pts tempPT;
			for(int i = 0; i < fd.size; i++)
            {
				if(arr[i].x*1000.0f > minX && arr[i].x*1000.0f < maxX && arr[i].y*1000.0f > minY && arr[i].y*1000.0f < maxY && arr[i].z*1000.0f > 0.0f && arr[i].z*1000.0f < 1000.0f)
				{
		            tempPT.x = arr[i].x * 1000.0f;
		            tempPT.y = arr[i].y * 1000.0f;
		            tempPT.z = arr[i].z * 1000.0f;
					fd.vtx.push_back(tempPT);
				}
            }
        }
    }
    if (_pts.size() != 0)
    {
        return fd;
    }
    else throw std::runtime_error("Can't fint depth stream");
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
	
	decimation_filter deci;
    spatial_filter spat;
    temporal_filter temp;

	deci.set_option(RS2_OPTION_FILTER_MAGNITUDE,2.0);
    spat.set_option(RS2_OPTION_HOLES_FILL, 2);
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
	//frame = deci.process(frame);
}

bool rsCam::setROI(std::vector<float> vec)
{
	rs2::region_of_interest temp;
	temp.min_x = (int)vec[1];// - 1280/2;
	temp.max_x = (int)vec[3];// - 1280/2;
	temp.min_y = (int)vec[0];// - 1280/2;	
	temp.max_y = (int)vec[2];// - 1280/2;
	_roi->set_region_of_interest(temp);
	return true;
}	
void rsCam::stopStream()
{
	_pipe->stop();
}
rsCam::~rsCam()
{
	//_pipe->stop();
	delete _roi;
    delete _pipe;
}
