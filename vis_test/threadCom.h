//
// Created by fred on 4/27/18.
//

#ifndef VIS_TEST_THREADCOM_H
#define VIS_TEST_THREADCOM_H

#ifndef _PLCLINECOM_H
#define _PLCLINECOM_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <mutex>
#include <librealsense2/hpp/rs_frame.hpp>
#include <queue>


class threadCom
{

public:
    threadCom(){};
    void push_back(const rs2::vertex  * data);
    bool pop(const rs2::vertex * data);

private:
    std::queue<const rs2::vertex *> Q;
    std::mutex Qlock;

};
#endif
