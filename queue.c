#include <stdio.h>
#include "queue.h"

void queue_init(Queue *queue, void *buffer, int max_size)
{
  Queue *q = queue;
  q->buffer = buffer;
  q->buffer_size = max_size;
  q->head = buffer;
  q->size = 0;
}

int queue_size(Queue *queue)
{
  return queue->size;
}

int queue_enqueue(Queue *queue, void *buffer, int size)
{
  Queue *q = queue;
  unsigned char *bp = buffer;
  unsigned char *qp = q->head + q->size;
  int i;

  if (q->size + size > q->buffer_size)
    return -1;
  for (i=0; i<size; i++) {
    if (qp >= q->buffer + q->buffer_size) {
      qp = q->buffer;
    }
    *qp++ = *bp++;
  }
  q->size += size;
  return 0;
}

int queue_enqueue_from_queue(Queue *queue, Queue *other)
{
  Queue *q = queue;
  unsigned char *qp = q->head + q->size;
  int i;

  if (q->size + other->size > q->buffer_size)
    return -1;
  for (i=0; i<other->size; i++) {
    if (qp >= q->buffer + q->buffer_size) {
      qp = q->buffer;
    }
    *qp++ = queue_peek(other, i);
  }
  q->size += other->size;
  return 0;
}

int queue_dequeue(Queue *queue, void *buffer, int size)
{
  Queue *q = queue;
  if (buffer != NULL) {
    unsigned char *bp = buffer;
    unsigned char *qp = q->head;
    int i;
    for (i=0; i<size; i++) {
      if (i == q->size)
        break;
      bp[i] = *qp++;
      if (qp >= q->buffer + q->buffer_size) {
        qp = q->buffer;
      }
    }
  }

  if (size > q->size)
    size = q->size;
  if (q->head + size >= q->buffer + q->buffer_size) {
    q->head = q->head + size - q->buffer_size;
  } else {
    q->head += size;
  }
  q->size -= size;
  return size;
}

unsigned char queue_peek(Queue *queue, int pos)
{
  Queue *q = queue;
  if (q->head + pos >= q->buffer + q->buffer_size) {
    return *(q->head + pos - q->buffer_size);
  } else {
    return *(q->head + pos);
  }
}

// clear queue
void queue_clear(Queue *queue)
{
  Queue *q = queue;
  q->size = 0;
}
