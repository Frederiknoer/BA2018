#ifndef VIS_TEST_SAFEQUEUE_H
#define VIS_TEST_SAFEQUEUE_H

#include "rsCam.h"
#include <vector>


class SafeQueue
{
public:
    SafeQueue() {};
    bool push(const rs2::vertex * frame)
    {
        if(!mutex)
        {
            mutex = true;
            frameQueue.push(frame);
            queueEmpty = false;
            if (frameQueue.empty())
                queueEmpty = true;
            mutex = false;
            return true;
        }
        else
            return false;
    };

    const rs2::vertex * pull()
    {
        if (!mutex)
        {
            mutex = true;
            const rs2::vertex *temp;
            temp = frameQueue.front();
            frameQueue.pop();
            if (frameQueue.empty())
                queueEmpty = true;
            mutex = false;
            return temp;
        }
        else
            return nullptr;
    };


    bool queueEmpty = true;
private:
    std::queue<const rs2::vertex *> frameQueue;
    bool mutex = false;

};


#endif //VIS_TEST_SAFEQUEUE_H
