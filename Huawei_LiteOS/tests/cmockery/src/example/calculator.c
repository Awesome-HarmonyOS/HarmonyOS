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

// A calculator example used to demonstrate the cmockery testing library.

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <assert.h>
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// If this is being built for a unit test.
#if UNIT_TESTING

/* Redirect printf to a function in the test application so it's possible to
 * test the standard output. */
#ifdef printf
#undef printf
#endif // printf
#define printf example_test_printf

extern void print_message(const char *format, ...);

extern int example_test_printf(const char *format, ...);

/* Redirect fprintf to a function in the test application so it's possible to
 * test error messages. */
#ifdef fprintf
#undef fprintf
#endif // fprintf
#define fprintf example_test_fprintf

extern int example_test_fprintf(FILE * const file, const char *format, ...);

// Redirect assert to mock_assert() so assertions can be caught by cmockery.
#ifdef assert
#undef assert
#endif // assert
#define assert(expression) \
    mock_assert((int)(expression), #expression, __FILE__, __LINE__)
void mock_assert(const int result, const char* expression, const char *file,
                 const int line);

/* Redirect calloc and free to test_calloc() and test_free() so cmockery can
 * check for memory leaks. */
#ifdef calloc
#undef calloc
#endif // calloc
#define calloc(num, size) _test_calloc(num, size, __FILE__, __LINE__)
#ifdef free
#undef free
#endif // free
#define free(ptr) _test_free(ptr, __FILE__, __LINE__)
void* _test_calloc(const size_t number_of_elements, const size_t size,
                   const char* file, const int line);
void _test_free(void* const ptr, const char* file, const int line);

/* main is defined in the unit test so redefine name of the the main function
 * here. */
#define main example_main

/* All functions in this object need to be exposed to the test application,
 * so redefine static to nothing. */
#define static

#endif // UNIT_TESTING


// A binary arithmetic integer operation (add, subtract etc.)
typedef int (*BinaryOperator)(int a, int b);

// Structure which maps operator strings to functions.
typedef struct OperatorFunction {
    const char* operator;
    BinaryOperator function;
} OperatorFunction;


static int add(int a, int b);
static int subtract(int a, int b);
static int multiply(int a, int b);
static int divide(int a, int b);

// Associate operator strings to functions.
static OperatorFunction operator_function_map[] = {
    {"+", add},
    {"-", subtract},
    {"*", multiply},
    {"/", divide},
};

static int add(int a, int b) {
    return a + b;
}

static int subtract(int a, int b) {
    return a - b;
}

static int multiply(int a, int b) {
    return a * b;
}

static int divide(int a, int b) {
    assert(b);  // Check for divide by zero.
    return a / b;
}

/* Searches the specified array of operator_functions for the function
 * associated with the specified operator_string.  This function returns the
 * function associated with operator_string if successful, NULL otherwise.
 */
static BinaryOperator find_operator_function_by_string(
        const size_t number_of_operator_functions,
        const OperatorFunction * const operator_functions,
        const char* const operator_string) {
    size_t i;
    assert(!number_of_operator_functions || operator_functions);
    assert(operator_string);

    for (i = 0; i < number_of_operator_functions; i++) {
        const OperatorFunction *const operator_function =
            &operator_functions[i];
        if (strcmp(operator_function->operator, operator_string) == 0) {
            return operator_function->function;
        }
    }
    return NULL;
}

/* Perform a series of binary arithmetic integer operations with no operator
 * precedence.
 *
 * The input expression is specified by arguments which is an array of
 * containing number_of_arguments strings.  Operators invoked by the expression
 * are specified by the array operator_functions containing
 * number_of_operator_functions, OperatorFunction structures.  The value of
 * each binary operation is stored in a pointer returned to intermediate_values
 * which is allocated by malloc().
 *
 * If successful, this function returns the integer result of the operations.
 * If an error occurs while performing the operation error_occurred is set to
 * 1, the operation is aborted and 0 is returned.
 */
static int perform_operation(
        int number_of_arguments, char *arguments[],
        const size_t number_of_operator_functions,
        const OperatorFunction * const operator_functions,
        int * const number_of_intermediate_values,
        int ** const intermediate_values, int * const error_occurred) {
    char *end_of_integer;
    int value;
    unsigned int i;
    assert(!number_of_arguments || arguments);
    assert(!number_of_operator_functions || operator_functions);
    assert(error_occurred);
    assert(number_of_intermediate_values);
    assert(intermediate_values);

    *error_occurred = 0;
    *number_of_intermediate_values = 0;
    *intermediate_values = NULL;
    if (!number_of_arguments)
        return 0;

    // Parse the first value.
    value = (int)strtol(arguments[0], &end_of_integer, 10);
    if (end_of_integer == arguments[0]) {
        // If an error occurred while parsing the integer.
        fprintf(stderr, "Unable to parse integer from argument %s\n",
                arguments[0]);
        *error_occurred = 1;
        return 0;
    }

    // Allocate an array for the output values.
    *intermediate_values = calloc(((number_of_arguments - 1) / 2),
                                  sizeof(**intermediate_values));

    i = 1;
    while (i < number_of_arguments) {
        int other_value;
        const char* const operator_string = arguments[i];
        const BinaryOperator function = find_operator_function_by_string(
            number_of_operator_functions, operator_functions, operator_string);
        int * const intermediate_value =
            &((*intermediate_values)[*number_of_intermediate_values]);
        (*number_of_intermediate_values) ++;

        if (!function) {
            fprintf(stderr, "Unknown operator %s, argument %d\n",
                    operator_string, i);
            *error_occurred = 1;
            break;
        }
        i ++;

        if (i == number_of_arguments) {
            fprintf(stderr, "Binary operator %s missing argument\n",
                    operator_string);
            *error_occurred = 1;
            break;
        }

        other_value = (int)strtol(arguments[i], &end_of_integer, 10);
        if (end_of_integer == arguments[i]) {
            // If an error occurred while parsing the integer.
            fprintf(stderr, "Unable to parse integer %s of argument %d\n",
                    arguments[i], i);
            *error_occurred = 1;
            break;
        }
        i ++;

        // Perform the operation and store the intermediate value.
        *intermediate_value = function(value, other_value);
        value = *intermediate_value;
    }
    if (*error_occurred) {
        free(*intermediate_values);
        *intermediate_values = NULL;
        *number_of_intermediate_values = 0;
        return 0;
    }
    return value;
}

int main(int argc, char *argv[]) {
    int return_value;
    int number_of_intermediate_values;
    int *intermediate_values;
    // Peform the operation.
    const int result = perform_operation(
        argc - 1, &argv[1],
        sizeof(operator_function_map) / sizeof(operator_function_map[0]),
        operator_function_map, &number_of_intermediate_values,
        &intermediate_values, &return_value);

    // If no errors occurred display the result.
    if (!return_value && argc > 1) {
        unsigned int i;
        unsigned int intermediate_value_index = 0;
        printf("%s\n", argv[1]);
        for (i = 2; i < argc; i += 2) {
            assert(intermediate_value_index < number_of_intermediate_values);
            printf("  %s %s = %d\n", argv[i], argv[i + 1],
                   intermediate_values[intermediate_value_index++]);
        }
        printf("= %d\n", result);
    }
    if (intermediate_values) {
        free(intermediate_values);
    }

    return return_value;
}
