#ifndef VIS_TEST_SAFEQUEUE_H
#define VIS_TEST_SAFEQUEUE_H

#include "rsCam.h"
#include <vector>


class SafeQueue
{
public:
    SafeQueue() {};
    void push(const rs2::vertex * frame)
    {
        if(!mutex)
        {
            mutex = true;
            frameQueue.push_back(frame);
            queueEmpty = false;
            if (frameQueue.empty())
                queueEmpty = true;
            mutex = false;
        }
    };

    const rs2::vertex * pull()
    {
        if (!mutex)
        {
            mutex = true;
            const rs2::vertex *temp;
            temp = frameQueue[0];
            frameQueue.erase(frameQueue.begin());
            if (frameQueue.empty())
                queueEmpty = true;
            mutex = false;
            return temp;
        }
    };


    bool queueEmpty = true;
private:
    std::vector<const rs2::vertex *> frameQueue = {};
    bool mutex = false;

};


#endif //VIS_TEST_SAFEQUEUE_H
