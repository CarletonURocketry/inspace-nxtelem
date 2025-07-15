#include <string.h>

#include "circular-buffer.h"

/**
 * Initializes a circular buffer with the given parameters
 * @param buffer The buffer structure to initialize
 * @param backing The backing array to use for storage
 * @param max_elems The maximum number of elements the backing array can hold
 * @param elem_size The size of each element in bytes
 */
void circ_buffer_init(struct circ_buffer *buffer, void *backing, int max_elems, int elem_size) {
    buffer->data = backing;
    buffer->capacity = max_elems;
    buffer->head = 0;
    buffer->size = 0;
    buffer->elem_size = elem_size;
}

int circ_buffer_size(struct circ_buffer *buffer) { return buffer->size; }

/**
 * Appends an element to the circular buffer, overwriting oldest data if full
 * @param buffer The circular buffer to push an element onto
 * @param data Pointer to the data to push
 */
void circ_buffer_push(struct circ_buffer *buffer, void *data) {
    if (buffer->size < buffer->capacity) {
        ++buffer->size;
    }
    memcpy(buffer->data + (buffer->head * buffer->elem_size), data, buffer->elem_size);
    buffer->head = (buffer->head + 1) % buffer->capacity;
}

/**
 * Pushes an element into the circular buffer, copies out the oldest element if full before overwriting it
 * @param buffer The circular buffer to push to
 * @param data Pointer to the data to push
 * @param out Pointer to the location to copy the oldest element to if the buffer is full
 * @return 1 if the buffer was full and the oldest element was copied out, 0 otherwise
 */
int circ_buffer_push_out(struct circ_buffer *buffer, void *data, void *out) {
    int ret = 0;
    if (buffer->size == buffer->capacity) {
        memcpy(out, buffer->data + (buffer->head * buffer->elem_size), buffer->elem_size);
        ret = 1;
    }
    circ_buffer_push(buffer, data);
    return ret;
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
void circ_iterator_init(struct circ_iterator *it, struct circ_buffer *buffer) { it->buffer = *buffer; }

/**
 * Returns the next element from the iterator
 * @param it The iterator to get the next element from
 * @return Pointer to the next element, or NULL if no more elements
 */
void *circ_iterator_next(struct circ_iterator *it) { return circ_buffer_pop(&it->buffer); }
