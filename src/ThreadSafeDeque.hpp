/*
 * ThreadSafeDeque.hpp
 *
 *  Created on: Jun. 13, 2020
 *      Author: daniel
 */

#ifndef SRC_THREADSAFEDEQUE_HPP_
#define SRC_THREADSAFEDEQUE_HPP_

#include <ros/ros.h>
#include <deque>
#include <mutex>

#define QUEUE_SIZE 200

using namespace std;
using namespace rs2;

typedef pair<uint64_t, frame*> framePair;

class ThreadSafeDeque {
public:
    void update(uint64_t t, rs2::frame& f) {
        lock_guard<mutex> lock(mutex_);
        if (deque_.size() >= QUEUE_SIZE) {
            frame* f = deque_.front().second;
            deque_.pop_front();
            delete f;
        }
        deque_.push_back(framePair(t, new frame(f)));
    }

    frame* getClosest(uint64_t stamp) {
        lock_guard<mutex> lock(mutex_);
        if (deque_.size() > 0) {
            return binarySearch(0, deque_.size()-1, stamp);
        } else {
            return NULL;
        }
    }
private:
    frame* binarySearch(uint p, uint r, uint64_t stamp) {
        if (p < r) {
            uint mid = (p + r)/2;
            framePair &el = deque_.at(mid);
            uint64_t val = el.first;
            if (val == stamp) {
                return el.second;
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
            framePair &el = deque_.at(p);
            int diff = el.first - stamp;
            if (diff > 0) {
                if (p == 0) return NULL;
                framePair &el2 = deque_.at(p-1);
                int diff2 = stamp - el2.first;
                return (diff < diff2) ? el.second : el2.second;
            } else if (diff < 0) {
                if (p >= deque_.size()-1) return NULL;
                framePair &el2 = deque_.at(p+1);
                int diff2 = el2.first - stamp;
                return (abs(diff) < diff2) ? el.second : el2.second;
            } else {
                return el.second;
            }
        }
        return NULL;
    }

    deque<framePair> deque_;
    mutex mutex_;
};

#endif /* SRC_THREADSAFEDEQUE_HPP_ */
