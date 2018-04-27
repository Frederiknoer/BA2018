
#include "Algorithms.h"
#include "rsCam.h"
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>

// A threadsafe-queue.
class SafeQueue
{
public:
    SafeQueue(void) : q(), m(), c() {}

    ~SafeQueue(void) {}

    // Add an element to the queue.
    void enqueue(std::vector<Algorithms::pts> t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    // Get the "front"-element.
    // If the queue is empty, wait till a element is avaiable.
    std::vector<Algorithms::pts> dequeue(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while(q.empty())
        {
            // release lock as long as the wait and reaquire it afterwards.
            c.wait(lock);
        }
        std::vector<Algorithms::pts> val = q.front();
        q.pop();
        return val;
    }

private:
    std::queue<std::vector<Algorithms::pts>> q;
    mutable std::mutex m;
    std::condition_variable c;
};
