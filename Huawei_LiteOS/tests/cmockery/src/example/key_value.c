/*
 * Copyright 2008 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct KeyValue {
    unsigned int key;
    const char* value;
} KeyValue;

static KeyValue *key_values = NULL;
static unsigned int number_of_key_values = 0;

void set_key_values(KeyValue * const new_key_values,
                    const unsigned int new_number_of_key_values) {
    key_values = new_key_values;
    number_of_key_values = new_number_of_key_values;
}

// Compare two key members of KeyValue structures.
int key_value_compare_keys(const void *a, const void *b) {
    return (int)((KeyValue*)a)->key - (int)((KeyValue*)b)->key;
}

// Search an array of key value pairs for the item with the specified value.
KeyValue* find_item_by_value(const char * const value) {
  unsigned int i;
    for (i = 0; i < number_of_key_values; i++) {
        if (strcmp(key_values[i].value, value) == 0) {
            return &key_values[i];
        }
    }
    return NULL;
}

// Sort an array of key value pairs by key.
void sort_items_by_key() {
    qsort(key_values, number_of_key_values, sizeof(*key_values),
          key_value_compare_keys);
}
