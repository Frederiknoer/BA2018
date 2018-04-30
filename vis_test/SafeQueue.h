#include "Algorithms.h"
#include "rsCam.h"
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>

class SafeQueue
{
public:
    SafeQueue(void) : q(), m(), c() {}

    ~SafeQueue(void) {}

    void enqueue(std::vector<Algorithms::pts>& t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    std::vector<Algorithms::pts>& dequeue(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while(q.empty())
            c.wait(lock); // release lock as long as the wait and reaquire it afterwards.

        std::vector<Algorithms::pts> val = q.front();
        q.pop();
        return val;
    }

private:
    std::queue<std::vector<Algorithms::pts>> q;
    mutable std::mutex m;
    std::condition_variable c;
};
