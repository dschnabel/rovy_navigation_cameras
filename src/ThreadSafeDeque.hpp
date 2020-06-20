/*
 * ThreadSafeDeque.hpp
 *
 *  Created on: Jun. 13, 2020
 *      Author: daniel
 */

#ifndef SRC_THREADSAFEDEQUE_HPP_
#define SRC_THREADSAFEDEQUE_HPP_

#include <deque>
#include <mutex>

#define QUEUE_SIZE 200

using namespace std;

class ThreadSafeDeque {
public:
    void update(uint64_t stamp) {
        lock_guard<mutex> lock(mutex_);
        if (deque_.size() >= QUEUE_SIZE) {
            deque_.pop_front();
        }
        deque_.push_back(stamp);
    }

    uint64_t getClosest(uint64_t stamp) {
        lock_guard<mutex> lock(mutex_);
        if (deque_.size() > 0) {
            return binarySearch(0, deque_.size()-1, stamp);
        } else {
            return 0;
        }
    }
private:
    uint64_t binarySearch(uint p, uint r, uint64_t stamp) {
        if (p < r) {
            uint mid = (p + r)/2;
            uint64_t val = deque_.at(mid);
            if (val == stamp) {
                return val;
            }
            if (val > stamp) {
                if (mid == p) mid++;
                return binarySearch(p, mid-1, stamp);
            }
            if (val < stamp) {
                if (mid == r) mid--;
                return binarySearch(mid+1, r, stamp);
            }
        } else if (p == r) {
            uint64_t val = deque_.at(p);
            int diff = val - stamp;
            if (diff > 0) {
                if (p == 0) return 0;
                uint64_t val2 = deque_.at(p-1);
                int diff2 = stamp - val2;
                return (diff < diff2) ? val : val2;
            } else if (diff < 0) {
                if (p >= deque_.size()-1) return 0;
                uint64_t val2 = deque_.at(p+1);
                int diff2 = val2 - stamp;
                return (abs(diff) < diff2) ? val : val2;
            } else {
                return val;
            }
        }
        return 0;
    }

    deque<uint64_t> deque_;
    mutex mutex_;
};

#endif /* SRC_THREADSAFEDEQUE_HPP_ */
