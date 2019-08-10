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
#include <stdio.h>
#include <database.h>
#ifdef _WIN32
#define snprintf _snprintf
#endif // _WIN32

// Connect to the database containing customer information.
DatabaseConnection* connect_to_customer_database() {
    return connect_to_database("customers.abcd.org", 321);
}

/* Find the ID of a customer by his/her name returning a value > 0 if
 * successful, 0 otherwise. */
unsigned int get_customer_id_by_name(
        DatabaseConnection * const connection,
        const char * const customer_name) {
    char query_string[256];
    int number_of_results;
    void **results;
    snprintf(query_string, sizeof(query_string),
             "SELECT ID FROM CUSTOMERS WHERE NAME = %s", customer_name);
    number_of_results = connection->query_database(connection, query_string,
                                                   &results);
    if (number_of_results != 1) {
        return -1;
    }
    return (unsigned int)results[0];
}
