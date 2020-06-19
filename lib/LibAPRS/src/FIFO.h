#ifndef UTIL_FIFO_H
#define UTIL_FIFO_H

#include <cstddef>
#include "mbed.h"


typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
  Semaphore *sem;
} FIFOBuffer;


inline bool fifo_isempty(FIFOBuffer &f) {
  return f.head == f.tail;
}


inline bool fifo_isfull(FIFOBuffer &f) {
  return ((f.head == f.begin) && (f.tail == f.end)) || (f.tail == f.head - 1);
}


inline void fifo_push(FIFOBuffer &f, unsigned char c) {
  *(f.tail) = c;
  
  if (f.tail == f.end) {
    f.tail = f.begin;
  } else {
    f.tail++;
  }
}


inline unsigned char fifo_pop(FIFOBuffer &f) {
  if(f.head == f.end) {
    f.head = f.begin;
    return *(f.end);
  } else {
    return *(f.head++);
  }
}


inline void fifo_flush(FIFOBuffer &f) {
  f.head = f.tail;
}


inline bool fifo_isempty_locked(FIFOBuffer &f) {
  bool result;
  
  f.sem->acquire();
  result = fifo_isempty(f);
  f.sem->release();
  
  return result;
}

inline bool fifo_isfull_locked(FIFOBuffer &f) {
  bool result;
  
  f.sem->acquire();  
  result = fifo_isfull(f);
  f.sem->release();
  
  return result;
}

inline void fifo_push_locked(FIFOBuffer &f, unsigned char c) {
	
  f.sem->acquire();
  fifo_push(f, c);
  f.sem->release();
  
}

inline unsigned char fifo_pop_locked(FIFOBuffer &f) {
  unsigned char c;
  
  f.sem->acquire();
  c = fifo_pop(f);
  f.sem->release();
  
  return c;
}

inline void fifo_init(FIFOBuffer &f, unsigned char *buffer, size_t size) {
  f.sem = new Semaphore(1);
  f.head = f.tail = f.begin = buffer;
  f.end = buffer + size -1;
}

inline size_t fifo_len(FIFOBuffer &f) {
  return f.end - f.begin;
}

#endif
