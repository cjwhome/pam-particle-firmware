/* MY_pb_common.h: Common support functions for MY_pb_encode.c and MY_pb_decode.c.
 * These functions are rarely needed by applications directly.
 */

#ifndef MY_PB_COMMON_H_INCLUDED
#define MY_PB_COMMON_H_INCLUDED

#include "MY_pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the field iterator structure to beginning.
 * Returns false if the message type is empty. */
bool MY_pb_field_iter_begin(MY_pb_field_iter_t *iter, const MY_pb_msgdesc_t *desc, void *message);

/* Get a field iterator for extension field. */
bool MY_pb_field_iter_begin_extension(MY_pb_field_iter_t *iter, MY_pb_extension_t *extension);

/* Same as MY_pb_field_iter_begin(), but for const message pointer.
 * Note that the pointers in MY_pb_field_iter_t will be non-const but shouldn't
 * be written to when using these functions. */
bool MY_pb_field_iter_begin_const(MY_pb_field_iter_t *iter, const MY_pb_msgdesc_t *desc, const void *message);
bool MY_pb_field_iter_begin_extension_const(MY_pb_field_iter_t *iter, const MY_pb_extension_t *extension);

/* Advance the iterator to the next field.
 * Returns false when the iterator wraps back to the first field. */
bool MY_pb_field_iter_next(MY_pb_field_iter_t *iter);

/* Advance the iterator until it points at a field with the given tag.
 * Returns false if no such field exists. */
bool MY_pb_field_iter_find(MY_pb_field_iter_t *iter, uint32_t tag);

/* Find a field with type MY_PB_LTYPE_EXTENSION, or return false if not found.
 * There can be only one extension range field per message. */
bool MY_pb_field_iter_find_extension(MY_pb_field_iter_t *iter);

#ifdef MY_PB_VALIDATE_UTF8
/* Validate UTF-8 text string */
bool MY_pb_validate_utf8(const char *s);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

