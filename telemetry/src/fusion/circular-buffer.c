#include <stdlib.h>
#include <string.h>

#include "circular-buffer.h"

/**
 * Initializes a circular buffer with the given parameters
 * @param buffer The buffer structure to initialize
 * @param backing The backing array to use for storage
 * @param max_elems The maximum number of elements the backing array can hold
 * @param elem_size The size of each element in bytes
 */
void circ_buffer_init(struct circ_buffer *buffer, void* backing, int max_elems, int elem_size) {
  buffer->data = backing;
  buffer->capacity = max_elems;
  buffer->head = 0;
  buffer->size = 0;
  buffer->elem_size = elem_size;
}

int circ_buffer_size(struct circ_buffer *buffer) {
  return buffer->size;
}

/**
 * Appends an element to the circular buffer, overwriting oldest data if full
 * @param buffer The circular buffer to append to
 * @param data Pointer to the data to append
 */
void circ_buffer_append(struct circ_buffer *buffer, void *data) {
  if (buffer->size < buffer->capacity) {
    ++buffer->size;
  }
  memcpy(buffer->data + (buffer->head * buffer->elem_size), data, buffer->elem_size);
  buffer->head = (buffer->head + 1) % buffer->capacity;
}

/**
 * Returns a pointer to the most recently added element without removing it
 * @param buffer The circular buffer to read from
 * @return Pointer to the most recent element, or NULL if buffer is empty
 */
void *circ_buffer_get(struct circ_buffer *buffer) {
  if (buffer->size == 0) return NULL;
  int pos = (buffer->head - 1 + buffer->capacity) % buffer->capacity;
  return buffer->data + (pos * buffer->elem_size);
}

/**
 * Removes and returns the most recently added element
 * @param buffer The circular buffer to pop from
 * @return Pointer to the removed element, or NULL if buffer is empty
 */
void *circ_buffer_pop(struct circ_buffer *buffer) {
  if (buffer->size == 0) return NULL;
  --buffer->size;
  buffer->head = (buffer->head - 1 + buffer->capacity) % buffer->capacity;
  return buffer->data + (buffer->head * buffer->elem_size);
}

/**
 * Initializes an iterator for the circular buffer
 * @param it The iterator structure to initialize
 * @param buffer The buffer to create an iterator for
 */
void circ_iterator_init(struct circ_iterator *it, struct circ_buffer *buffer) {
  it->buffer = *buffer;
}

/**
 * Returns the next element from the iterator
 * @param it The iterator to get the next element from
 * @return Pointer to the next element, or NULL if no more elements
 */
void *circ_iterator_next(struct circ_iterator *it) {
  return circ_buffer_pop(&it->buffer);
}
