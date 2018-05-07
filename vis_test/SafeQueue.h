#include "Algorithms.h"
#include "rsCam.h"
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>

class SafeQueue
{
public:
    SafeQueue(void) : q(), m(), c() {}

    ~SafeQueue(void) {}

  /* void enqueue(std::vector<Algorithms::pts> &t)
    {
        m.lock();
        q.push(t);
        c.notify_one();
        m.unlock();
    }

    std::vector<Algorithms::pts> dequeue(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while(q.empty())
            c.wait(lock); // release lock as long as the wait and reaquire it afterwards.

        std::vector<Algorithms::pts> val;
        val = q.front();
        q.pop();
        return val;
    }*/
 	void enqueue(frmdata &t)
    {
        m.lock();
        q.push(t);
        c.notify_one();
        m.unlock();
    }

    frmdata dequeue(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while(q.empty())
            c.wait(lock); // release lock as long as the wait and reaquire it afterwards.

        //frmdata val;
        auto val = q.front();
        q.pop();
        return val;
    }
    void clearQueue(void)
    {
        m.lock();
        for(int i = 0; i < q.size(); i++)
           q.pop();
        m.unlock();
    }

private:
   // std::queue<std::vector<Algorithms::pts>> q;
	std::queue<frmdata> q;
    mutable std::mutex m;
    std::condition_variable c;
};
