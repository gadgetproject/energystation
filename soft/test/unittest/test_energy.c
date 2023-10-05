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

void setUp(void)
{
	energy_init();
}

void tearDown(void)
{
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

	energy_reset(12345678, 654321);
	energy_read(&s);
	TEST_ASSERT_EQUAL(12345678, s.total_kilowatt_hours);
	TEST_ASSERT_EQUAL(654321, s.part_milliwatt_hours);
}

void test_1Wh(void)
{
	energy_reset(112358, 999999);
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
	energy_reset(112358, 998012);

	static energy_status_t status;
	static bool updated;

	void callback(void)
	{
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
