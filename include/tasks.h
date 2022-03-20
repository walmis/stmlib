/*
 * tasks.h
 *
 *  Created on: Oct 22, 2021
 *      Author: walmis
 */

#ifndef STMLIB_INCLUDE_TASKS_H_
#define STMLIB_INCLUDE_TASKS_H_

#include <stdint.h>
#include <CBUF.h>
#include <functional>
#include "inplace_function.h"
#include <algorithm>
#include <iterator>
#include <string.h>

#include <libopencm3/cm3/cortex.h>


extern uint32_t millis();

using FunctionType =  modm::inplace_function<void(), sizeof(void*)*3>;

class Task
{
public:
    //virtual ~Task() {}

    virtual std::size_t size() const
    {
        return 1U;
    }

    virtual void exec() {}
};

template <typename TTask>
class TaskBound : public Task
{
public:

    // Size is minimal number of elements of size equal to sizeof(Task)
    // that will be able to store this TaskBound object
    static const std::size_t Size =
        ((sizeof(TaskBound<typename std::decay<TTask>::type>) - 1) /
                                                     sizeof(Task)) + 1;

    explicit TaskBound(const TTask& task)
      : task_(task)
    {
    }

    explicit TaskBound(TTask&& task)
      : task_(std::move(task))
    {
    }

    //~TaskBound() {}

    std::size_t size() const
    {
        return Size;
    }

    void exec()
    {
        task_();
    }

private:
    TTask task_;
};

class TaskQueue;

class PeriodicTask {
public:
    PeriodicTask(int period = 0, FunctionType callback = 0) : 
      callback(callback), period(period) {
    }
    ~PeriodicTask() {
      
    }
    bool exec() {
      if(is_running || !callback) return false;
      // free running
      if(period == 0) {
        callback();
      } else {
        uint32_t ms = millis();
        if(ms - last_run >= period) {
          last_run = ms;
          is_running = true;
          callback();
          is_running = false;
          return true;
        }
      }

      return false;
    }
    FunctionType callback;
    uint32_t period   = 0;
    uint32_t last_run = 0;
    PeriodicTask* next = 0;
    bool is_running = false;
};

struct TaskQueue
{
  static constexpr int maxTaskSize = 8;
  static constexpr int taskQueueSize = 32;

  uint16_t             m_get_idx;
  uint16_t             m_put_idx;
  void*                m_entry[ taskQueueSize ];
  PeriodicTask*        periodic_tasks = nullptr;
  PeriodicTask*        next_task      = nullptr;

  static_assert(sizeof(void*) == sizeof(Task));

  TaskQueue() {
    CBUF_Init((*this));
  }

  bool pushTask(PeriodicTask& task) {
    if(!periodic_tasks) {
      periodic_tasks = &task;
      task.next = nullptr;
    } else {
      PeriodicTask* t = periodic_tasks;
      while(t->next) {
        t = t->next;
      }
      t->next = &task;
      task.next = nullptr;
    }
    return true;
  }

  bool removeTask(PeriodicTask& task);
 
  template<typename TTask>
  bool pushTask(TTask&& task) {
    typedef TaskBound<typename std::decay<TTask>::type> TaskBoundType;
    static_assert(
        std::alignment_of<Task>::value == std::alignment_of<TaskBoundType>::value,
        "Alignment of TaskBound must be same as alignment of Task");

    static const std::size_t requiredQueueSize = TaskBoundType::Size;

    static_assert(
        requiredQueueSize <= maxTaskSize,
        "Task size too big");

    if(CBUF_Space((*this)) < requiredQueueSize) {
      return false;
    }

    Task data[requiredQueueSize];
    new (data) TaskBoundType(std::forward<TTask>(task));
    CM_ATOMIC_BLOCK() {
      for(size_t i = 0; i < requiredQueueSize; i++) {
        CBUF_Push((*this), *(void**)&data[i]);
      }
    }

    return true;
  }

  void runTask();
};


#endif /* STMLIB_INCLUDE_TASKS_H_ */
