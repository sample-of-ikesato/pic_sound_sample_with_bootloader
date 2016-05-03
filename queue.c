#include <stdio.h>
#include "queue.h"


void queue_init(Queue *q, void *buffer, int max_size)
{
  q->buffer = buffer;
  q->buffer_size = max_size;
  q->head = buffer;
  q->size = 0;
}

int queue_size(Queue *q)
{
  return q->size;
}

int queue_enqueue(Queue *q, void *buffer, int size)
{
  unsigned char *bp = buffer;
  unsigned char *qp = q->head;
  int i;
  int ret = 0;

  if (q->size + size > q->buffer_size)
    ret = -1;
  for (i=0; i<size; i++) {
    if (qp >= q->buffer + q->buffer_size) {
      qp -= q->buffer_size;
    }
    *qp++ = *bp++;
  }
  if (q->size + size > q->buffer_size)
    q->size = q->buffer_size;
  else
    q->size += size;
  q->head = qp;
  return ret;
}

int queue_enqueue_from_queue(Queue *q, Queue *other)
{
  unsigned char *qp = q->head;
  int i;
  int ret = 0;

  if (q->size + other->size > q->buffer_size)
    ret = -1;
  for (i=0; i<other->size; i++) {
    if (qp >= q->buffer + q->buffer_size) {
      qp -= q->buffer_size;
    }
    *qp++ = queue_peek(other, i);
  }
  if (q->size + other->size > q->buffer_size)
    q->size = q->buffer_size;
  else
    q->size += other->size;
  q->head = qp;
  return ret;
}

int queue_dequeue(Queue *q, void *buffer, int size)
{
  if (buffer != NULL) {
    unsigned char *bp = buffer;
    unsigned char *qp = q->head - q->size;
    if (qp < q->buffer) {
      qp += q->buffer_size;
    }
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
  q->size -= size;
  return size;
}

unsigned char queue_peek(Queue *q, int pos)
{
  unsigned char *qp = q->head - q->size + pos;
  if (qp < q->buffer) {
    qp += q->buffer_size;
  }
  return *qp;
}

// clear queue
void queue_clear(Queue *q)
{
  q->size = 0;
}
