#include <tasks.h>

void TaskQueue::runTask() {
  if(!CBUF_IsEmpty((*this))) {
    auto ptr = CBUF_GetPopEntryPtr((*this));
    int size = ((Task*)ptr)->size();
    void* tmp[maxTaskSize];
    //memcpy(tmp, ptr, size*sizeof(Task));
    CM_ATOMIC_BLOCK() {
      for(int i = 0; i < size; i++) {
        tmp[i] = CBUF_Pop((*this));
      }
    }
    ((Task*)tmp)->exec();
  }

  if(!next_task) {
    next_task = periodic_tasks;
  }
  if(next_task) {
    //printf("next %p\n", next_task);
    PeriodicTask* task = next_task;
    next_task = next_task->next;
    task->exec();

    return;
  }

}

bool TaskQueue::removeTask(PeriodicTask& task) {
  if(periodic_tasks == &task) {
    //remove first item from list
    periodic_tasks = periodic_tasks->next;
  }
  if(next_task == &task) {
    next_task = nullptr;
  }
  PeriodicTask* t = periodic_tasks;

  if(!t) {
    return false;
  }
  while(t->next) {
    if(t->next == &task) {
      t->next = task.next;
      return true;
    }
  }

  return false;
}
