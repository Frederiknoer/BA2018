//
// Created by fred on 4/27/18.
//

#include "threadCom.h"
#include <fstream>
#include <sstream>

using namespace boost;
using boost::asio::ip::udp;

#include <limits>

void threadCom::push_back(const rs2::vertex * data)
{
    Qlock.lock();
    Q.push(data);
    Qlock.unlock();
}

bool threadCom::pop(const rs2::vertex * data)
{
    std::lock_guard<std::mutex> lk(Qlock);
    bool res = false;
    if(Q.size() > 0)
    {
        res = true;
        data = Q.front();
        Q.pop();
        //std::cout << "\tQ size: " << Q. << std::endl;
    }
    return res;
}