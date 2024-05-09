/*! \file test_energy.c
 *
 *  \brief Energy Monitor unit test
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "unity.h"  	/* Framework */

#include "energy.h"    /* Module under test */

#include <string.h>	/* memset() */

static unsigned fixture_expect_update = ~0U;

#define EXPECT_UPDATE(scope_)                     \
do                                                \
{                                                 \
    TEST_ASSERT_EQUAL(~0U, fixture_expect_update); \
    fixture_expect_update = scope_;               \
} while(0)

static void fixture_update(unsigned scope)
{
    /* Update expected? */
    TEST_ASSERT_TRUE(~fixture_expect_update);
    /* Update matches? */
    TEST_ASSERT_EQUAL(fixture_expect_update, scope);
    fixture_expect_update = ~0U;
}

void setUp(void)
{
    TEST_ASSERT_TRUE(energy_init());
    energy_register(&fixture_update);
}

void tearDown(void)
{
    EXPECT_UPDATE(~0U);
}

void test_init(void)
{
    energy_status_t s;
    (void)memset(&s, 0x55, sizeof(s));

    energy_read(&s);
    TEST_ASSERT_EQUAL(0, s.total_kilowatt_hours);
    TEST_ASSERT_EQUAL(0, s.part_milliwatt_hours);
    TEST_ASSERT_EQUAL(0, s.latest_milliwatts);
}

void test_reset(void)
{
    energy_status_t s;
    (void)memset(&s, 0x55, sizeof(s));

    EXPECT_UPDATE(0);
    energy_reset(12345678, 654321);
    energy_read(&s);
    TEST_ASSERT_EQUAL(12345678, s.total_kilowatt_hours);
    TEST_ASSERT_EQUAL(654321, s.part_milliwatt_hours);
}

void test_1Wh(void)
{
    EXPECT_UPDATE(0);
    energy_reset(112358, 999999);
    EXPECT_UPDATE(3000);
    energy_1Wh(3600);	/* 1kW */

    energy_status_t s;
    energy_read(&s);
    TEST_ASSERT_EQUAL(112359, s.total_kilowatt_hours);
    TEST_ASSERT_EQUAL(0, s.part_milliwatt_hours);
    TEST_ASSERT_EQUAL(1000000, s.latest_milliwatts);
    TEST_ASSERT_EQUAL(3600, s.update_milliseconds);
}

void test_register_update(void)
{
    EXPECT_UPDATE(0);
    energy_reset(112358, 998012);

    static energy_status_t status;
    static bool updated;

    void callback(unsigned scope)
    {
        TEST_ASSERT_EQUAL(3000, scope);
        energy_read(&status);
        TEST_ASSERT_EQUAL(112358, status.total_kilowatt_hours);
        TEST_ASSERT_EQUAL(999000, status.part_milliwatt_hours);
        TEST_ASSERT_EQUAL(50000, status.latest_milliwatts);
        TEST_ASSERT_EQUAL(72000, status.update_milliseconds);
        updated = true;
    }
    energy_register(&callback);

    (void)memset(&status, 0x55, sizeof(status));
    updated = false;

    energy_1Wh(72000);	/* 50W */

    TEST_ASSERT_TRUE(updated);
}

void test_history_30s(void)
{
    energy_watts_t watts_30s[30];

    /* Initially history is empty */
    TEST_ASSERT_EQUAL(0, energy_graph_3s(0, 30, watts_30s));

    /* 10s @ 360W */
    EXPECT_UPDATE(3000);
    energy_1Wh(10000);
    energy_watts_t watts_10s[30];
    TEST_ASSERT_EQUAL(3, energy_graph_3s(0, 30, watts_10s));

    /* 16s @ 225W */
    EXPECT_UPDATE(3000);
    energy_1Wh(16000);
    energy_watts_t watts_26s[30];
    TEST_ASSERT_EQUAL(8, energy_graph_3s(0, 30, watts_26s));

    /* 4s @ 900W */
    EXPECT_UPDATE(3000);
    energy_1Wh(4000);

    /* Final history is 30s */
    TEST_ASSERT_EQUAL(10, energy_graph_3s(0, 30, watts_30s));

    energy_watts_t expected[10] =
    {
    /* 4s @ 900W */
        900, 450,                   /* 225/3 + 225/3 + 900/3 = 75+75+300 */
    /* 16s @ 225W */
        225, 225, 225, 225, 270,    /* 360/3 + 225/3 + 225/3 = 120+75+75 */
    /* 10s @ 360W */
        360, 360, 360,
    };
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected+7, watts_10s, 3);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected+2, watts_26s, 8);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected, watts_30s, 10);
}

void test_history_9s(void)
{
    /* 4s @ 900W */
    EXPECT_UPDATE(3000);
    energy_1Wh(4000);
    /* 1s @ 3600W */
    EXPECT_UPDATE(0);
    energy_1Wh(1000);
    /* 4s @ 900W */
    EXPECT_UPDATE(3000);
    energy_1Wh(4000);

    energy_watts_t expected[4] =
    {
    /* 4s @ 900W */
        900,
    /* 1s @ 3600W */                /* 900/3 + 3600/3 + 900/3 = 300+1200+300 */
        1800,
    /* 4s @ 900W */
        900,
    /* Sentinel */
        0xCAFE,
    };

    energy_watts_t watts[4] = { 0xDEAD, 0xBEEF, 0xFEED, 0xCAFE };

    /* More data than buffer */
    TEST_ASSERT_EQUAL(2, energy_graph_3s(0, 2, watts));
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected, watts, 2);
    TEST_ASSERT_EQUAL(0xFEED, watts[2]);

    /* More buffer than data */
    TEST_ASSERT_EQUAL(3, energy_graph_3s(0, 4, watts));
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected, watts, 4);

    /* Offset */
    TEST_ASSERT_EQUAL(2, energy_graph_3s(1, 4, watts));
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected+1, watts, 2);
}

void test_history_3m(void)
{
    /* 60s @ 60W */
    EXPECT_UPDATE(3000);
    energy_1Wh(60000);
    /* 120s @ 30W */
    EXPECT_UPDATE(90000);
    energy_1Wh(120000);

    energy_watts_t watts[60];
    TEST_ASSERT_EQUAL(30, energy_graph_3s(0, 60, watts));
    for (unsigned i = 0; i < 30; i++)
        TEST_ASSERT_EQUAL(30, watts[i]);

    TEST_ASSERT_EQUAL(2, energy_graph_90s(0, 60, watts));
    TEST_ASSERT_EQUAL(30, watts[0]);
    TEST_ASSERT_EQUAL(50, watts[1]);    /* (60*60+30*30)/90 = (3600+900)/90 */
}

void test_history_2h(void)
{
    for (unsigned i = 0; i < 40; i++)
    {
        /* 60s @ 60W */
        EXPECT_UPDATE(3000);
        energy_1Wh(60000);
        /* 120s @ 30W */
        EXPECT_UPDATE((i+1) % 15 ? 90000 : 2700000);
        energy_1Wh(120000);
    }

    energy_watts_t watts[60];
    TEST_ASSERT_EQUAL(30, energy_graph_90s(0, 60, watts));
    for (unsigned i = 0; i < 30; i++)
    {
        /* Alternates 30/50W */
        TEST_ASSERT_EQUAL((i&1) ? 50 : 30, watts[i]);
    }

    TEST_ASSERT_EQUAL(2, energy_graph_45m(0, 60, watts));
    TEST_ASSERT_EQUAL(40, watts[0]);
    TEST_ASSERT_EQUAL(40, watts[1]);
}
