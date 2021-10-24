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

struct TaskQueue
{
  volatile uint16_t    m_get_idx;
  volatile uint16_t    m_put_idx;
  Task                 m_entry[ 8 ];

  TaskQueue() {
    CBUF_Init((*this));
  }
    /*
  * queue a single run of function
  * can be called in ISR context
  */
  __attribute((noinline))
  void* getPlacePtr(int size) {
    void* placePtr = 0;

    CM_ATOMIC_BLOCK() {
      size_t contig = CBUF_ContigSpace((*this));
      size_t free = CBUF_Space((*this));
      //printf("get %d\n", size);
      if(size > contig) {
        Task tsk;
        //fill last entries with dummy tasks
        for(int i = 0; i < contig; i++) {
          //printf("fill\n");
          CBUF_Push((*this), tsk);
        }
      }

      if(CBUF_ContigSpace((*this)) >= size) {
        placePtr = CBUF_GetPushEntryPtr((*this));
        CBUF_AdvancePushIdxBy((*this), size);
      }
    }
    return placePtr;
  }

  template<typename TTask>
  bool pushTask(TTask&& task) {
    typedef TaskBound<typename std::decay<TTask>::type> TaskBoundType;
    static_assert(
        std::alignment_of<Task>::value == std::alignment_of<TaskBoundType>::value,
        "Alignment of TaskBound must be same as alignment of Task");

    static const std::size_t requiredQueueSize = TaskBoundType::Size;

    void* placePtr = getPlacePtr(requiredQueueSize);
    if(!placePtr) return false;

    //printf("place c:%d g:%d p:%d %d\n", CBUF_ContigSpace(tskq), tskq.m_get_idx, tskq.m_put_idx, CBUF_Error(tskq));
    new (placePtr) TaskBoundType(std::forward<TTask>(task));
    //printf("size %d\n", sizeof(TaskBoundType));

    return true;
  }

  void runTask() {
    if(!CBUF_IsEmpty((*this))) {
      auto ptr = CBUF_GetPopEntryPtr((*this));
      int size = ptr->size();
      ptr->exec();
      CBUF_AdvancePopIdxBy((*this), size);
    }
  }
};


#endif /* STMLIB_INCLUDE_TASKS_H_ */
