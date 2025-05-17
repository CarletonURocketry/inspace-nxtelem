#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

/* A circular buffer without a backing array */
struct circ_buffer {
  void *data;    /* The backing array of this buffer */
  int capacity;  /* The maximum number of elements in this buffer */
  int head;      /* The next write position in the buffer */
  int size;      /* the number of elements in the buffer */
  int elem_size; /* The size of each element in the buffer */
};

/* An iterator of struct circ_buffer */
struct circ_iterator {
  struct circ_buffer buffer; /* A view of the buffer this iterator refers to */
};

void circ_buffer_init(struct circ_buffer *buffer, void* backing, int max_elems, int elem_size);
void circ_buffer_append(struct circ_buffer *buffer, void *data);
void *circ_buffer_get(struct circ_buffer *buffer);
void *circ_buffer_pop(struct circ_buffer *buffer);
void circ_iterator_init(struct circ_iterator *it, struct circ_buffer *buffer);
void *circ_iterator_next(struct circ_iterator *it);

#endif /* _CIRCULAR_BUFFER_H_ */
