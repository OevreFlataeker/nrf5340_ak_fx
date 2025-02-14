#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_IsEmpty(RingBuffer_t *rb) {
    return rb->head == rb->tail;
}

bool RingBuffer_IsFull(RingBuffer_t *rb) {
    return ((rb->head + 1) & (RING_BUFFER_SIZE - 1)) == rb->tail;
}

bool RingBuffer_Push(RingBuffer_t *rb, int32_t data) {
    if (RingBuffer_IsFull(rb)) {
        return false;  // Buffer full
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) & (RING_BUFFER_SIZE - 1);
    return true;
}

bool RingBuffer_Pop(RingBuffer_t *rb, int32_t *data) {
    if (RingBuffer_IsEmpty(rb)) {
        return false;  // Buffer empty
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & (RING_BUFFER_SIZE - 1);
    return true;
}
