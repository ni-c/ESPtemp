#include <backoff.h>
#include <unity.h>

static void test_backoff_doubles_from_one_period_up_to_the_cap()
{
    const uint16_t cap = backoffCap(7200e6, 300e6);
    TEST_ASSERT_EQUAL_UINT16(24, cap);
    TEST_ASSERT_EQUAL_UINT16(1, backoffSleeps(0, cap));
    TEST_ASSERT_EQUAL_UINT16(2, backoffSleeps(1, cap));
    TEST_ASSERT_EQUAL_UINT16(4, backoffSleeps(2, cap));
    TEST_ASSERT_EQUAL_UINT16(8, backoffSleeps(3, cap));
    TEST_ASSERT_EQUAL_UINT16(16, backoffSleeps(4, cap));
    TEST_ASSERT_EQUAL_UINT16(24, backoffSleeps(5, cap));
    TEST_ASSERT_EQUAL_UINT16(24, backoffSleeps(6, cap));
    TEST_ASSERT_EQUAL_UINT16(24, backoffSleeps(255, cap));
}

static void test_backoff_cap_counts_whole_periods_and_is_at_least_one()
{
    TEST_ASSERT_EQUAL_UINT16(24, backoffCap(7400e6, 300e6));
    TEST_ASSERT_EQUAL_UINT16(1, backoffCap(300e6, 300e6));
    TEST_ASSERT_EQUAL_UINT16(1, backoffCap(100e6, 300e6));
    TEST_ASSERT_EQUAL_UINT16(1, backoffCap(-1, 300e6));
    TEST_ASSERT_EQUAL_UINT16(1, backoffCap(7200e6, 0));
    TEST_ASSERT_EQUAL_UINT16(1, backoffCap(7200e6, -300e6));
    TEST_ASSERT_EQUAL_UINT16(65535, backoffCap(1e12, 1));
}

static void test_backoff_with_the_smallest_and_largest_caps()
{
    TEST_ASSERT_EQUAL_UINT16(1, backoffSleeps(0, 0));
    TEST_ASSERT_EQUAL_UINT16(1, backoffSleeps(3, 0));
    TEST_ASSERT_EQUAL_UINT16(1, backoffSleeps(3, 1));
    TEST_ASSERT_EQUAL_UINT16(32768, backoffSleeps(15, 65535));
    TEST_ASSERT_EQUAL_UINT16(65535, backoffSleeps(16, 65535));
    TEST_ASSERT_EQUAL_UINT16(10, backoffSleeps(16, 10));
}

void runBackoffTests()
{
    RUN_TEST(test_backoff_doubles_from_one_period_up_to_the_cap);
    RUN_TEST(test_backoff_cap_counts_whole_periods_and_is_at_least_one);
    RUN_TEST(test_backoff_with_the_smallest_and_largest_caps);
}
