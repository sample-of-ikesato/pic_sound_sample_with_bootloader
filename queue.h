#ifndef _queue_h_
#define _queue_h_

typedef struct Queue_t {
  unsigned char *head;
  unsigned char *buffer;
  int buffer_size;
  int size;
} Queue;

// initilize Queue object
void queue_init(Queue *queue, void *buffer, int max_size);

// get current queue size
int queue_size(Queue *queue);

// enqueu buffer
// return 0:success -1:buffer over flow
int queue_enqueue(Queue *queue, void *buffer, int size);

// dequeue buffer
// return readed buffer size (less than argument size)
int queue_dequeue(Queue *queue, void *buffer, int size);

// enqueu from queue
// return 0:success -1:buffer over flow
int queue_enqueue_from_queue(Queue *queue, Queue *other);

// peek queue
// return queue data
unsigned char queue_peek(Queue *queue, int pos);

// clear queue
void queue_clear(Queue *queue);

#endif//_queue_h_
