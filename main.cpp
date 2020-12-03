#include <chrono>
#include <functional>
#include <glog/logging.h>
#include <iostream>
#include <queue>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <utility>

using namespace std;
using namespace std::chrono;

typedef time_point<system_clock> Timestamp;

typedef function<void()> Callback;

bool stop = false;

struct Timer {
  Timestamp expTime;
  Callback callback;

  Timer(Timestamp exp, Callback cb) : expTime(exp), callback(std::move(cb)) {}

  bool operator<(const Timer &other) const { return expTime > other.expTime; }
};

struct TimeQueue {
  priority_queue<Timer> queue;
  int timerFd;

  TimeQueue() {
    timerFd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    PLOG_IF(FATAL, timerFd < 0);
  }

  void runAfter(Callback cb, int seconds, int nanoSeconds = 0) {
    auto expTime = system_clock::now() + seconds * 1s + nanoSeconds * 1ns;
    Timer timer(expTime, std::move(cb));
    if (queue.empty() || (expTime < queue.top().expTime)) {
      resetExpTime(expTime);
    }
    queue.push(timer);
  }

  vector<Callback> getExpCallbacks() {

    auto now = system_clock::now();
    vector<Callback> ret;

    DLOG_ASSERT(!queue.empty());
    DLOG_ASSERT(now >= queue.top().expTime);

    while (!queue.empty()) {

      if (now >= queue.top().expTime) {
        auto cb = queue.top();
        queue.pop();
        ret.push_back(cb.callback);
      } else {
        break;
      }
    }

    if (!queue.empty()) {
      auto minExpTime = queue.top().expTime;
      resetExpTime(minExpTime);
    }
    return ret;
  }

  void resetExpTime(Timestamp expTime) const {

    auto diff =
        duration_cast<nanoseconds>(expTime - system_clock::now()).count();

    itimerspec it{};
    timespec ts{};

    ts.tv_sec = diff / 1000000000;
    ts.tv_nsec = diff % 1000000000;

    struct itimerspec oldValue {};
    it.it_value = ts;

    if (::timerfd_settime(timerFd, 0, &it, &oldValue) < 0) {
      PLOG(FATAL);
    }
  }
};

int main() {
  int epfd = ::epoll_create1(EPOLL_CLOEXEC);
  epoll_event event{};
  event.events = EPOLLIN | EPOLLPRI;
  auto *t = new TimeQueue();

  epoll_ctl(epfd, EPOLL_CTL_ADD, t->timerFd, &event);

  t->runAfter([] { DLOG(INFO) << "1"; }, 1, 100);
  t->runAfter([] { DLOG(INFO) << "2"; }, 2, 100);
  t->runAfter([] { DLOG(INFO) << "5"; }, 5, 100);
  t->runAfter([] { DLOG(INFO) << "3"; }, 3, 100);

  epoll_event out[100];

  while (!stop) {

    int n = epoll_wait(epfd, out, 100, 1e9);
    LOG_ASSERT(n > 0);

    long one;
    n = ::read(t->timerFd, &one, sizeof(one));
    LOG_ASSERT(n == sizeof(one));
    auto cbs = t->getExpCallbacks();

    for (auto &cb : cbs) {
      cb();
    }
  }
}
