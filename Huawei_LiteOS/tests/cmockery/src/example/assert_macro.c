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
#include <string.h>

static const char* status_code_strings[] = {
    "Address not found",
    "Connection dropped",
    "Connection timed out",
};

const char* get_status_code_string(const unsigned int status_code) {
    return status_code_strings[status_code];
};

unsigned int string_to_status_code(const char* const status_code_string) {
    unsigned int i;
    for (i = 0; i < sizeof(status_code_strings) /
                    sizeof(status_code_strings[0]); i++) {
        if (strcmp(status_code_strings[i], status_code_string) == 0) {
            return i;
        }
    }
    return ~0U;
}
