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
typedef struct DatabaseConnection DatabaseConnection;

/* Function that takes an SQL query string and sets results to an array of
 * pointers with the result of the query.  The value returned specifies the
 * number of items in the returned array of results.  The returned array of
 * results are statically allocated and should not be deallocated using free()
 */
typedef unsigned int (*QueryDatabase)(
    DatabaseConnection* const connection, const char * const query_string,
    void *** const results);

// Connection to a database.
struct DatabaseConnection {
    const char *url;
    unsigned int port;
    QueryDatabase query_database;
};

// Connect to a database.
DatabaseConnection* connect_to_database(const char * const url,
                                        const unsigned int port);

