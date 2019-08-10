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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif
#ifdef _LINUX
#include <sys/types.h>
#endif

#if UNIT_TESTING
extern void* _test_malloc(const size_t size, const char* file, const int line);
extern void* _test_calloc(const size_t number_of_elements, const size_t size,
                          const char* file, const int line);
extern void _test_free(void* const ptr, const char* file, const int line);

#define malloc(size) _test_malloc(size, __FILE__, __LINE__)
#define calloc(num, size) _test_calloc(num, size, __FILE__, __LINE__)
#define free(ptr) _test_free(ptr, __FILE__, __LINE__)
#endif // UNIT_TESTING

void leak_memory() {
    int * const temporary = (int*)malloc(sizeof(int));
    *temporary = 0;
}

void buffer_overflow() {
    char * const memory = (char*)malloc(sizeof(int));
    memory[sizeof(int)] = '!';
    free(memory);
}

void buffer_underflow() {
    char * const memory = (char*)malloc(sizeof(int));
    memory[-1] = '!';
    free(memory);
}
