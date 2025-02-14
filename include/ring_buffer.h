#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RING_BUFFER_SIZE  8192  // Power of 2 for efficiency

typedef struct {
    int32_t buffer[RING_BUFFER_SIZE];  // 32-bit stereo samples (L+R)
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb);
bool RingBuffer_IsEmpty(RingBuffer_t *rb);
bool RingBuffer_IsFull(RingBuffer_t *rb);
bool RingBuffer_Push(RingBuffer_t *rb, int32_t data);
bool RingBuffer_Pop(RingBuffer_t *rb, int32_t *data);

#endif