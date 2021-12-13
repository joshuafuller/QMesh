#ifndef MAIL_QUEUE_HPP
#define MAIL_QUEUE_HPP

#ifndef TEST_FEC
#include "os_portability.hpp"
#endif /* TEST_FEC */
#include <atomic>


template<class T>
class EventMail {
private:
    static constexpr int EVENTMAIL_QUEUE_DEPTH = 32;
    atomic<int> level{};
    Mail_portable<T, EVENTMAIL_QUEUE_DEPTH> mail;

public:
    EventMail() : level(0) { }

    auto dequeue_mail_timeout(const uint32_t timeout_ms, bool &timed_out) -> T {
        T mail_item; 
        timed_out = false;
        osEvent evt = mail.get(timeout_ms);
        if(evt.status == osEventMail) {
            mail_item = *(static_cast<T *>(evt.value.p));
            mail.free(static_cast<T *>(evt.value.p));
        } else if(evt.status == osEventTimeout) {
            timed_out = true;
        } else { PORTABLE_ASSERT(false); }
        level -= 1;
        return mail_item;
    }

    /** 
    * Dequeues a value from an Mbed OS mailbox. Returns the dequeued value.
    * @param mail_queue The mailbox.
    * @param T The type of the value to be dequeued.
    */
    auto dequeue_mail() -> T {
        T mail_item; 
        for(;;) {
            osEvent evt = mail.get();
            if(evt.status == osEventMail) {
                mail_item = *(static_cast<T *>(evt.value.p));
                mail.free(static_cast<T *>(evt.value.p));
                break;
            }
            PORTABLE_ASSERT(false);
        } 
        level -= 1;
        return mail_item;
    }

    /** 
    * Enqueues a value onto an Mbed OS mailbox. Quietly drops the 
    * request if the queue is full.
    * @param mail_queue The mailbox.
    * @param T The type of the value to be enqueued.
    * @param val The value to be enqueued.
    */
    void enqueue_mail_nonblocking(T val) {
        auto mail_item = mail.alloc();
        if(mail_item == NULL) {
            return;
        }
        *mail_item = val;
        level += 1;
        mail.put(mail_item);
    }

    /** 
    * Enqueues a value onto an Mbed OS mailbox.
    * @param mail_queue The mailbox.
    * @param T The type of the value to be enqueued.
    * @param val The value to be enqueued.
    */
    void enqueue_mail(T val) {
        auto mail_item = mail.alloc_for(osWaitForever);
        PORTABLE_ASSERT(mail_item != NULL);
        *mail_item = val;
        while(mail.full()) {}
        level += 1;
        mail.put(mail_item);
    }

    auto full() -> bool {
        return mail.full();
    }

    auto empty() -> bool {
        return mail.empty();
    }

    auto getLevel() -> int {
        return level;
    }
};

#endif /* MAIL_QUEUE_HPP */