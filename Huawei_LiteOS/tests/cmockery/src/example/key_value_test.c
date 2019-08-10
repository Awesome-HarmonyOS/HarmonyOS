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
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <string.h>
#include <cmockery.h>

/* This is duplicated here from the module setup_teardown.c to reduce the
 * number of files used in this test. */
typedef struct KeyValue {
    unsigned int key;
    const char* value;
} KeyValue;

void set_key_values(KeyValue * const new_key_values,
                    const unsigned int new_number_of_key_values);
extern KeyValue* find_item_by_value(const char * const value);
extern void sort_items_by_key();

static KeyValue key_values[] = {
    { 10, "this" },
    { 52, "test" },
    { 20, "a" },
    { 13, "is" },
};

void create_key_values(void **state) {
    KeyValue * const items = (KeyValue*)test_malloc(sizeof(key_values));
    memcpy(items, key_values, sizeof(key_values));
    *state = (void*)items;
    set_key_values(items, sizeof(key_values) / sizeof(key_values[0]));
}

void destroy_key_values(void **state) {
    test_free(*state);
    set_key_values(NULL, 0);
}

void test_find_item_by_value(void **state) {
    unsigned int i;
    for (i = 0; i < sizeof(key_values) / sizeof(key_values[0]); i++) {
        KeyValue * const found  = find_item_by_value(key_values[i].value);
        assert_true(found);
        assert_int_equal(found->key, key_values[i].key);
        assert_string_equal(found->value, key_values[i].value);
    }
}

void test_sort_items_by_key(void **state) {
    unsigned int i;
    KeyValue * const kv = *state;
    sort_items_by_key();
    for (i = 1; i < sizeof(key_values) / sizeof(key_values[0]); i++) {
        assert_true(kv[i - 1].key < kv[i].key);
    }
}

int main(int argc, char* argv[]) {
    const UnitTest tests[] = {
        unit_test_setup_teardown(test_find_item_by_value, create_key_values,
                                 destroy_key_values),
        unit_test_setup_teardown(test_sort_items_by_key, create_key_values,
                                 destroy_key_values),
    };
    return run_tests(tests);
}
