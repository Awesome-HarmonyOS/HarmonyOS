/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/


#include "MQTTPacket.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#if !defined(_WINDOWS)
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#else
#include <winsock2.h>
#include <ws2tcpip.h>
#define MAXHOSTNAMELEN 256
#define EAGAIN WSAEWOULDBLOCK
#define EINTR WSAEINTR
#define EINPROGRESS WSAEINPROGRESS
#define EWOULDBLOCK WSAEWOULDBLOCK
#define ENOTCONN WSAENOTCONN
#define ECONNRESET WSAECONNRESET
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

struct Options
{
    char *connection;         /**< connection to system under test. */
    char **haconnections;
    int hacount;
    int verbose;
    int test_no;
} options =
{
    "tcp://m2m.eclipse.org:1883",
    NULL,
    0,
    0,
    0,
};

void usage()
{

}

void getopts(int argc, char **argv)
{
    int count = 1;

    while (count < argc)
    {
        if (strcmp(argv[count], "--test_no") == 0)
        {
            if (++count < argc)
                options.test_no = atoi(argv[count]);
            else
                usage();
        }
        else if (strcmp(argv[count], "--connection") == 0)
        {
            if (++count < argc)
            {
                options.connection = argv[count];
                printf("\nSetting connection to %s\n", options.connection);
            }
            else
                usage();
        }
        else if (strcmp(argv[count], "--haconnections") == 0)
        {
            if (++count < argc)
            {
                char *tok = strtok(argv[count], " ");
                options.hacount = 0;
                options.haconnections = malloc(sizeof(char *) * 5);
                while (tok)
                {
                    options.haconnections[options.hacount] = malloc(strlen(tok) + 1);
                    strcpy(options.haconnections[options.hacount], tok);
                    options.hacount++;
                    tok = strtok(NULL, " ");
                }
            }
            else
                usage();
        }
        else if (strcmp(argv[count], "--verbose") == 0)
        {
            options.verbose = 1;
            printf("\nSetting verbose on\n");
        }
        count++;
    }
}


#define LOGA_DEBUG 0
#define LOGA_INFO 1
#include <stdarg.h>
#include <time.h>
#include <sys/timeb.h>
void MyLog(int LOGA_level, char *format, ...)
{
    static char msg_buf[256];
    va_list args;
    struct timeb ts;

    struct tm *timeinfo;

    if (LOGA_level == LOGA_DEBUG && options.verbose == 0)
        return;

    ftime(&ts);
    timeinfo = localtime(&ts.time);
    strftime(msg_buf, 80, "%Y%m%d %H%M%S", timeinfo);

    sprintf(&msg_buf[strlen(msg_buf)], ".%.3hu ", ts.millitm);

    va_start(args, format);
    vsnprintf(&msg_buf[strlen(msg_buf)], sizeof(msg_buf) - strlen(msg_buf), format, args);
    va_end(args);

    printf("%s\n", msg_buf);
    fflush(stdout);
}


#if defined(WIN32) || defined(_WINDOWS)
#define mqsleep(A) Sleep(1000*A)
#define START_TIME_TYPE DWORD
static DWORD start_time = 0;
START_TIME_TYPE start_clock(void)
{
    return GetTickCount();
}
#elif defined(AIX)
#define mqsleep sleep
#define START_TIME_TYPE struct timespec
START_TIME_TYPE start_clock(void)
{
    static struct timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    return start;
}
#else
#define mqsleep sleep
#define START_TIME_TYPE struct timeval
/* TODO - unused - remove? static struct timeval start_time; */
START_TIME_TYPE start_clock(void)
{
    struct timeval start_time;
    gettimeofday(&start_time, NULL);
    return start_time;
}
#endif


#if defined(WIN32)
long elapsed(START_TIME_TYPE start_time)
{
    return GetTickCount() - start_time;
}
#elif defined(AIX)
#define assert(a)
long elapsed(struct timespec start)
{
    struct timespec now, res;

    clock_gettime(CLOCK_REALTIME, &now);
    ntimersub(now, start, res);
    return (res.tv_sec) * 1000L + (res.tv_nsec) / 1000000L;
}
#else
long elapsed(START_TIME_TYPE start_time)
{
    struct timeval now, res;

    gettimeofday(&now, NULL);
    timersub(&now, &start_time, &res);
    return (res.tv_sec) * 1000 + (res.tv_usec) / 1000;
}
#endif


#define assert(a, b, c, d) myassert(__FILE__, __LINE__, a, b, c, d)
#define assert1(a, b, c, d, e) myassert(__FILE__, __LINE__, a, b, c, d, e)

int tests = 0;
int failures = 0;
FILE *xml;
START_TIME_TYPE global_start_time;
char output[3000];
char *cur_output = output;


void write_test_result()
{
    long duration = elapsed(global_start_time);

    fprintf(xml, " time=\"%ld.%.3ld\" >\n", duration / 1000, duration % 1000);
    if (cur_output != output)
    {
        fprintf(xml, "%s", output);
        cur_output = output;
    }
    fprintf(xml, "</testcase>\n");
}


void myassert(char *filename, int lineno, char *description, int value, char *format, ...)
{
    ++tests;
    if (!value)
    {
        va_list args;

        ++failures;
        printf("Assertion failed, file %s, line %d, description: %s\n", filename, lineno, description);

        va_start(args, format);
        vprintf(format, args);
        va_end(args);

        cur_output += sprintf(cur_output, "<failure type=\"%s\">file %s, line %d </failure>\n",
                              description, filename, lineno);
    }
    else
        MyLog(LOGA_DEBUG, "Assertion succeeded, file %s, line %d, description: %s", filename, lineno, description);
}

#define min(a, b) ((a < b) ? a : b)

int checkMQTTStrings(MQTTString a, MQTTString b)
{
    if (!a.lenstring.data)
    {
        a.lenstring.data = a.cstring;
        if (a.cstring)
            a.lenstring.len = strlen(a.cstring);
    }
    if (!b.lenstring.data)
    {
        b.lenstring.data = b.cstring;
        if (b.cstring)
            b.lenstring.len = strlen(b.cstring);
    }
    return memcmp(a.lenstring.data, b.lenstring.data, min(a.lenstring.len, b.lenstring.len)) == 0;
}


int checkConnectPackets(MQTTPacket_connectData *before, MQTTPacket_connectData *after)
{
    int rc = 0;
    int start_failures = failures;

    assert("struct_ids should be the same",
           memcmp(before->struct_id, after->struct_id, 4) == 0, "struct_ids were different %.4s\n", after->struct_id);

    assert("struct_versions should be the same",
           before->struct_version == after->struct_version, "struct_versions were different\n", rc);

    assert("MQTT versions should be the same",
           before->MQTTVersion == after->MQTTVersion, "MQTT versions were different\n", rc);

    assert("ClientIDs should be the same",
           checkMQTTStrings(before->clientID, after->clientID), "ClientIDs were different\n", rc);

    assert("keepAliveIntervals should be the same",
           before->keepAliveInterval == after->keepAliveInterval, "keepAliveIntervals were different %d\n", after->keepAliveInterval);

    assert("cleansessions should be the same",
           before->cleansession == after->cleansession, "cleansessions were different\n", rc);

    assert("willFlags should be the same",
           before->willFlag == after->willFlag, "willFlags were different\n", rc);

    if (before->willFlag)
    {
        assert("will struct_ids should be the same",
               memcmp(before->will.struct_id, after->will.struct_id, 4) == 0, "will struct_ids were different %.4s\n", after->struct_id);

        assert("will struct_versions should be the same",
               before->will.struct_version == after->will.struct_version, "will struct_versions were different\n", rc);

        assert("topic names should be the same",
               checkMQTTStrings(before->will.topicName, after->will.topicName), "topic names were different\n", rc);

        assert("messages should be the same",
               checkMQTTStrings(before->will.message, after->will.message), "messages were different\n", rc);

        assert("retained flags should be the same",
               before->will.retained == after->will.retained, "retained flags were different\n", rc);

        assert("will qos should be the same",
               before->will.qos == after->will.qos, "will qos were different\n", rc);
    }

    assert("usernames should be the same",
           checkMQTTStrings(before->clientID, after->clientID), "usernames were different\n", rc);
    assert("passwords should be the same",
           checkMQTTStrings(before->password, after->password), "passwords were different\n", rc);
    return failures == start_failures;
}

int test1(struct Options options)
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    MQTTPacket_connectData data_after = MQTTPacket_connectData_initializer;
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 1 - serialization of connect and back");

    data.clientID.cstring = "me";

    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";

    data.willFlag = 1;
    data.will.message.cstring = "will message";
    data.will.qos = 1;
    data.will.retained = 0;
    data.will.topicName.cstring = "will topic";

    rc = MQTTSerialize_connect(buf, buflen, &data);
    assert("good rc from serialize connect", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_connect(&data_after, buf, buflen);
    assert("good rc from deserialize connect", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    rc = checkConnectPackets(&data, &data_after);
    assert("packets should be the same",  rc == 1, "packets were different\n", rc);

    /* exit: */
    MyLog(LOGA_INFO, "TEST1: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}


int test2(struct Options options)
{
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);

    unsigned char dup = 0;
    int qos = 2;
    unsigned char retained = 0;
    unsigned short msgid = 23;
    MQTTString topicString = MQTTString_initializer;
    unsigned char *payload = (unsigned char *)"kkhkhkjkj jkjjk jk jk ";
    int payloadlen = strlen((char *)payload);

    unsigned char dup2 = 1;
    int qos2 = 1;
    unsigned char retained2 = 1;
    unsigned short msgid2 = 3243;
    MQTTString topicString2 = MQTTString_initializer;
    unsigned char *payload2 = NULL;
    int payloadlen2 = 0;

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 2 - serialization of publish and back");

    topicString.cstring = "mytopic";
    rc = MQTTSerialize_publish(buf, buflen, dup, qos, retained, msgid, topicString,
                               payload, payloadlen);
    assert("good rc from serialize publish", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_publish(&dup2, &qos2, &retained2, &msgid2, &topicString2,
                                 &payload2, &payloadlen2, buf, buflen);
    assert("good rc from deserialize publish", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    assert("dups should be the same", dup == dup2, "dups were different %d\n", dup2);
    assert("qoss should be the same", qos == qos2, "qoss were different %d\n", qos2);
    assert("retaineds should be the same", retained == retained2, "retaineds were different %d\n", retained2);
    assert("msgids should be the same", msgid == msgid2, "msgids were different %d\n", msgid2);

    assert("topics should be the same",
           checkMQTTStrings(topicString, topicString2), "topics were different %s\n", ""); //topicString2);

    assert("payload lengths should be the same",
           payloadlen == payloadlen2, "payload lengths were different %d\n", payloadlen2);

    assert("payloads should be the same",
           memcmp(payload, payload2, payloadlen) == 0, "payloads were different %s\n", "");

    /*exit:*/
    MyLog(LOGA_INFO, "TEST2: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}



int test3(struct Options options)
{
    int i = 0;
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);
#define TOPIC_COUNT 2

    unsigned char dup = 0;
    unsigned short msgid = 23;
    int count = TOPIC_COUNT;
    MQTTString topicStrings[TOPIC_COUNT] = { MQTTString_initializer, MQTTString_initializer };
    int req_qoss[TOPIC_COUNT] = {2, 1};

    unsigned char dup2 = 1;
    unsigned short msgid2 = 2223;
    int count2 = 0;
    MQTTString topicStrings2[TOPIC_COUNT] = { MQTTString_initializer, MQTTString_initializer };
    int req_qoss2[TOPIC_COUNT] = {0, 0};

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 2 - serialization of subscribe and back");

    topicStrings[0].cstring = "mytopic";
    topicStrings[1].cstring = "mytopic2";
    rc = MQTTSerialize_subscribe(buf, buflen, dup, msgid, count, topicStrings, req_qoss);
    assert("good rc from serialize subscribe", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_subscribe(&dup2, &msgid2, 2, &count2, topicStrings2, req_qoss2, buf, buflen);
    assert("good rc from deserialize subscribe", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    assert("dups should be the same", dup == dup2, "dups were different %d\n", dup2);
    assert("msgids should be the same", msgid == msgid2, "msgids were different %d\n", msgid2);

    assert("count should be the same", count == count2, "counts were different %d\n", count2);

    for (i = 0; i < count2; ++i)
    {
        assert("topics should be the same",
               checkMQTTStrings(topicStrings[i], topicStrings2[i]), "topics were different %s\n", "");

        assert("qoss should be the same", req_qoss[i] == req_qoss2[i], "qoss were different %d\n", req_qoss2[i]);
    }

    /*exit:*/
    MyLog(LOGA_INFO, "TEST3: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}


int test4(struct Options options)
{
    int i = 0;
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);
#define TOPIC_COUNT 2

    int msgid = 23;
    int count = TOPIC_COUNT;
    int granted_qoss[TOPIC_COUNT] = {2, 1};

    unsigned short msgid2 = 2223;
    int count2 = 0;
    int granted_qoss2[TOPIC_COUNT] = {0, 0};

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 4 - serialization of suback and back");

    rc = MQTTSerialize_suback(buf, buflen, msgid, count, granted_qoss);
    assert("good rc from serialize suback", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_suback(&msgid2, 2, &count2, granted_qoss2, buf, buflen);
    assert("good rc from deserialize suback", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    assert("msgids should be the same", msgid == msgid2, "msgids were different %d\n", msgid2);

    assert("count should be the same", count == count2, "counts were different %d\n", count2);

    for (i = 0; i < count2; ++i)
        assert("qoss should be the same", granted_qoss[i] == granted_qoss2[i], "qoss were different %d\n", granted_qoss2[i]);

    /* exit: */
    MyLog(LOGA_INFO, "TEST4: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}


int test5(struct Options options)
{
    int i = 0;
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);
#define TOPIC_COUNT 2

    unsigned char dup = 0;
    unsigned short msgid = 23;
    int count = TOPIC_COUNT;
    MQTTString topicStrings[TOPIC_COUNT] = { MQTTString_initializer, MQTTString_initializer };

    unsigned char dup2 = 1;
    unsigned short msgid2 = 2223;
    int count2 = 0;
    MQTTString topicStrings2[TOPIC_COUNT] = { MQTTString_initializer, MQTTString_initializer };

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 2 - serialization of unsubscribe and back");

    topicStrings[0].cstring = "mytopic";
    topicStrings[1].cstring = "mytopic2";
    rc = MQTTSerialize_unsubscribe(buf, buflen, dup, msgid, count, topicStrings);
    assert("good rc from serialize unsubscribe", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_unsubscribe(&dup2, &msgid2, 2, &count2, topicStrings2, buf, buflen);
    assert("good rc from deserialize unsubscribe", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    assert("dups should be the same", dup == dup2, "dups were different %d\n", dup2);
    assert("msgids should be the same", msgid == msgid2, "msgids were different %d\n", msgid2);

    assert("count should be the same", count == count2, "counts were different %d\n", count2);

    for (i = 0; i < count2; ++i)
        assert("topics should be the same",
               checkMQTTStrings(topicStrings[i], topicStrings2[i]), "topics were different %s\n", "");

    /* exit: */
    MyLog(LOGA_INFO, "TEST5: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}


int test6(struct Options options)
{
    int rc = 0;
    unsigned char buf[100];
    int buflen = sizeof(buf);

    unsigned char sessionPresent = 1;
    unsigned char connack_rc = 77;

    unsigned char sessionPresent2 = 0;
    unsigned char connack_rc2 = 0;

    fprintf(xml, "<testcase classname=\"test1\" name=\"de/serialization\"");
    global_start_time = start_clock();
    failures = 0;
    MyLog(LOGA_INFO, "Starting test 2 - serialization of connack and back");

    rc = MQTTSerialize_connack(buf, buflen, connack_rc, sessionPresent);
    assert("good rc from serialize connack", rc > 0, "rc was %d\n", rc);

    rc = MQTTDeserialize_connack(&sessionPresent2, &connack_rc2, buf, buflen);
    assert("good rc from deserialize connack", rc == 1, "rc was %d\n", rc);

    /* data after should be the same as data before */
    assert("connack rcs should be the same", connack_rc == connack_rc2, "connack rcs were different %d\n", connack_rc2);
    assert("session present flags should be the same", sessionPresent == sessionPresent2,
           "session present flags were different %d\n", sessionPresent2);

    /* exit: */
    MyLog(LOGA_INFO, "TEST6: test %s. %d tests run, %d failures.",
          (failures == 0) ? "passed" : "failed", tests, failures);
    write_test_result();
    return failures;
}


int main(int argc, char **argv)
{
    int rc = 0;
    int (*tests[])() = {NULL, test1, test2, test3, test4, test5, test6};

    xml = fopen("TEST-test1.xml", "w");
    fprintf(xml, "<testsuite name=\"test1\" tests=\"%d\">\n", (int)(ARRAY_SIZE(tests) - 1));

    getopts(argc, argv);

    if (options.test_no == 0)
    {
        /* run all the tests */
        for (options.test_no = 1; options.test_no < ARRAY_SIZE(tests); ++options.test_no)
            rc += tests[options.test_no](options); /* return number of failures.  0 = test succeeded */
    }
    else
        rc = tests[options.test_no](options); /* run just the selected test */

    if (rc == 0)
        MyLog(LOGA_INFO, "verdict pass");
    else
        MyLog(LOGA_INFO, "verdict fail");

    fprintf(xml, "</testsuite>\n");
    fclose(xml);
    return rc;
}
