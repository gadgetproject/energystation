/*! \file energy.c
 *
 *  \brief Energy monitor
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

#include "energy.h"

#include <stddef.h>
#include <string.h>

static energy_status_t energy_status;
static energy_cb energy_update;

bool energy_init(void)
{
    (void)memset(&energy_status, 0, sizeof(energy_status));
    energy_update = NULL;
    return false;
}

void energy_read(energy_status_t* status)
{
    (void)memcpy(status, &energy_status, sizeof(*status));
}

energy_cb energy_register(energy_cb callback)
{
    energy_cb was = energy_update;
    energy_update = callback;
    return was;
}

void energy_reset(uint32_t tkwh, uint32_t pmwh)
{
    energy_status.total_kilowatt_hours = tkwh;
    energy_status.part_milliwatt_hours = pmwh;

    /* Update observer */
    if (energy_update)
    {
        energy_update();
    }
}

void energy_1Wh(unsigned milliseconds)
{
    unsigned duration_ms = milliseconds - energy_status.update_milliseconds;

    /* 1Wh == 3600Ws */
    if (milliseconds)
    {
        energy_status.latest_milliwatts = (3600U*1000U*1000U)/milliseconds;
    }

    /* This signal discards any fraction of a Wh */
    unsigned disregard = energy_status.part_milliwatt_hours % 1000;
    energy_status.part_milliwatt_hours += 1000-disregard;

    /* Roll over into kWh */
    if (energy_status.part_milliwatt_hours >= 1000000)
    {
        energy_status.part_milliwatt_hours -= 1000000;
        energy_status.total_kilowatt_hours += 1;
    }

    /* Update observer */
    energy_status.update_milliseconds = milliseconds;
    if (energy_update)
    {
        energy_update();
    }
}
