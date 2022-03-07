#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>

/**
 * RING FIFO BUFFER (QUEUE)  First In - First Out.
 *
 * Буфер заточен на работу в многопоточном режиме. Запись в него происходит во время прерываний,
 * а чтение в main_loop (основной программе). Поэтому в нем не может быть счетчика элементов
 * (счетчик нужно менять и при чтении и при записи и это может вызвать конфликт). Вместо этого
 * используются два указателя типа volatile:  head (указывает куда писать) и
 * tail (указывает откуда читать). При записи меняется только head, а при чтении только tail
 *
 * Проблема в том что head == tail и когда буфер пуст и когда буфер полон и необходимо как-то эти два
 * состояния различать. Существуют разные способы решения этой проблемы.
 * 1) head и tail все время растут и выходят за пределы maxsize а индексы вычисляются
 * их делением на maxsize  по модулю
 * 2) заполнять буфер не до конца а лишь до (buffer size - 1), оставляя всегда пустой слот.
 * В этом случае буфер считается полным когда head + 1 == tail
 *
 * Деление операция дорогая и долгая на микроконтроллерах поэтому используем подход (2)
 * By using a single empty cell to detect the “full” case,
 * we can support a single producer and single consumer without a lock
 * (as long as put and get don’t modify the same variables).
 * The queue (buffer) is "thread-safe" because the producer will only modify the write_pointer index,
 * and the consumer will only modify the read_pointer index.
 * While either index might be slightly out-of-date in a given context,
 * this will not impact the thread safety of the queue.
 *
 * Note: This solution is ok only for ONE producer and consumer.
 * If there are multiple producers/consumers interacting with a queue,
 * we will need a lock https://embedjournal.com/implementing-circular-buffer-embedded-c/#synchronization-and-race-conditions
 *
 *  Реализация взята из https://github.com/pepyakin/msp430-uart/blob/master/msp430-uart/include/ringbuf.h
 * (в гите у меня есть форк на него)
 */

typedef struct
{
    unsigned char* buffer;
    unsigned int buff_size; // размер буфера (записать мы можем соответсвенно на 1 меньше => buff_siz -1)
    volatile unsigned int head; // индекс куда будет записан след элемент
    volatile unsigned int tail; //  индекс откуда будет считан fifo элемент
} ringbuffer;


inline void ringbuffer_init(ringbuffer* ringbuf, unsigned char* buffer, unsigned int buf_size) {
    ringbuf->buffer = buffer;
    ringbuf->buff_size = buf_size;
    ringbuf->tail = 0;
    ringbuf->head = 0;
}

/*
 * Возвращает, пустой ли кольцевой буффер?
 */
inline bool ringbuffer_empty(ringbuffer* ringbuf) {
    return ringbuf->head == ringbuf->tail;
}

/*
 * Кладет в буффер элемент.
 * return true(1) if element was written successfully
 * and false(0) if ringbuf is full and element can not be written
 */
inline bool ringbuffer_write(ringbuffer* ringbuf, unsigned char ch) {
    unsigned int next_head = (unsigned int) (ringbuf->head + 1);
    if (next_head == ringbuf->buff_size) {
        next_head = 0;
    }

    if (next_head == ringbuf->tail) { // буфер полон
        return false;
    }

    ringbuf->buffer[ringbuf->head] = ch;
    ringbuf->head = next_head;
    return true;
}

/*
 * Извлекает один элемент из буффера и записывает его в переменную по указанному адресу
 * return true(1) if element was read successfully
 * and false(0) if ringbuf is empty and element can not be read
 */
inline bool  ringbuffer_read(ringbuffer* ringbuf, unsigned char* chp) {
    // если буфер пустой
    if (ringbuf->head == ringbuf->tail) {
        return false;
    }
    *chp = ringbuf->buffer[ringbuf->tail];

    unsigned int next_tail = (unsigned int) (ringbuf->tail + 1);
    if (next_tail == ringbuf->buff_size) {
        next_tail = 0;
    }
    ringbuf->tail = next_tail;
    return true;
}


/*
 * Возвращает, сколько элементов находится в буфере (доступно для чтения)
 */
inline unsigned int ringbuffer_available_for_read(ringbuffer* ringbuf) {
    long length = ringbuf->head - ringbuf->tail;
    if (length >= 0) {
        /*
         * Указатель для записи находится спереди или на том же месте
         * как и указатель для чтения (в этом случае буфер пуст).
         *
         * |---R-----W----|
         *
         * R - ringbuf->tail
         * W - ringbuf->head
         */
        return length;
    } else {
        /*
         * Указатель для записи находится позади указателя чтения. Это
         * значит что указатель головы (записи) перешёл на новый круг, в то время как
         * указатель хвоста (чтения) остался на прежнем кругу.
         *
         * |--W---------R-|
         */
       return -length - 1;
    }
}

/*
 * Возвращает, сколько элементов буфер может вместить (доступно для записи)
 */
inline unsigned char ringbuffer_available_for_write(ringbuffer* ringbuf) {
    unsigned int capacity = (ringbuf->buff_size - 1);
    long length = ringbuf->head - ringbuf->tail;
    if (length >= 0) {
        /*
         * Указатель для записи находится спереди или на том же месте
         * как и указатель для чтения (в этом случае буфер пуст).
         *
         * |---R-----W----|
         *
         * R - ringbuf->tail
         * W - ringbuf->head
         */
        return capacity - length;
    } else {
        /*
         * Указатель для записи находится позади указателя чтения. Это
         * значит что указатель головы (записи) перешёл на новый круг, в то время как
         * указатель хвоста (чтения) остался на прежнем кругу.
         *
         * |--W---------R-|
         */
        return capacity + length + 1;
    }

}

#endif //RINGBUF_H
