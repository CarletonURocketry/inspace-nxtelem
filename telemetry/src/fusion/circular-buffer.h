/* A circular buffer without a backing array */
struct circ_buffer {
  void *data;
  int capacity;
  int head;
  int size;
  int elem_size;
};

/* An iterator of struct circ_buffer */
struct circ_iterator {
  struct circ_buffer buffer;
};

void circ_buffer_init(struct circ_buffer *buffer, void* backing, int max_elems, int elem_size);
void circ_buffer_append(struct circ_buffer *buffer, void *data);
void *circ_buffer_get(struct circ_buffer *buffer);
void *circ_buffer_pop(struct circ_buffer *buffer);
void circ_iterator_init(struct circ_iterator *it, struct circ_buffer *buffer);
void *circ_iterator_next(struct circ_iterator *it);
