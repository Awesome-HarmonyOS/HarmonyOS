#include "lwip_check.h"

#include "ip4/test_ip4.h"
#include "udp/test_udp.h"
#include "tcp/test_tcp.h"
#include "tcp/test_tcp_oos.h"
#include "core/test_mem.h"
#include "core/test_pbuf.h"
#include "etharp/test_etharp.h"
#include "dhcp/test_dhcp.h"
#include "mdns/test_mdns.h"

#include "lwip/init.h"

Suite* create_suite(const char* name, testfunc *tests, size_t num_tests, SFun setup, SFun teardown)
{
  size_t i;
  Suite *s = suite_create(name);

  for(i = 0; i < num_tests; i++) {
    TCase *tc_core = tcase_create(name);
    if ((setup != NULL) || (teardown != NULL)) {
      tcase_add_checked_fixture(tc_core, setup, teardown);
    }
    tcase_add_named_test(tc_core, tests[i]);
    suite_add_tcase(s, tc_core);
  }
  return s;
}

#ifdef LWIP_UNITTESTS_LIB
int lwip_unittests_run(void)
#else
int main(void)
#endif
{
  int number_failed;
  SRunner *sr;
  size_t i;
  suite_getter_fn* suites[] = {
    ip4_suite,
    udp_suite,
    tcp_suite,
    tcp_oos_suite,
    mem_suite,
    pbuf_suite,
    etharp_suite,
    dhcp_suite,
    mdns_suite
  };
  size_t num = sizeof(suites)/sizeof(void*);
  LWIP_ASSERT("No suites defined", num > 0);

  lwip_init();

  sr = srunner_create((suites[0])());
  for(i = 1; i < num; i++) {
    srunner_add_suite(sr, ((suite_getter_fn*)suites[i])());
  }

#ifdef LWIP_UNITTESTS_NOFORK
  srunner_set_fork_status(sr, CK_NOFORK);
#endif
#ifdef LWIP_UNITTESTS_FORK
  srunner_set_fork_status(sr, CK_FORK);
#endif

  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
