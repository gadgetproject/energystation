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

#if 0
# include <stdio.h>
# define LOG_DBG(printf_like...) (void)fprintf(stderr, printf_like)
#else
/* Compiler will parse all parameters but generate no code */
# define LOG_DBG(printf_like...) while(0) ((void(*)())0)(printf_like)
#endif

static energy_status_t energy_status;
static energy_cb energy_update;

#define ENERGY_HISTORY_DAYS 32
#define ENERGY_MILLISECONDS_ROLLOVER ((unsigned)(ENERGY_HISTORY_DAYS*24U*60U*60U*1000U))

static struct
{
    unsigned milliseconds;                      /* Time accumulator */
    unsigned millijoules;                       /* Energy accumulator */
    energy_watts_t sec3[30];                    /* 3s per reading */
    energy_watts_t sec90[30];                   /* 90s per reading */
    energy_watts_t min45[32];                   /* 45m=2700s per reading */
    energy_watts_t hour24[ENERGY_HISTORY_DAYS]; /* 24h=84600s per reading */
    bool rollover;                              /* TRUE after time rollover */
} energy_history;


bool energy_init(void)
{
    (void)memset(&energy_status, 0, sizeof(energy_status));
    (void)memset(&energy_history, 0, sizeof(energy_history));
    energy_update = NULL;
    return true;
}

void energy_read(energy_status_t* status)
{
    (void)memcpy(status, &energy_status, sizeof(*status));
}

/**
 * @brief Retrieve graph of energy usage
 * @param bucket_size_ms duration of each bucket
 * @param bucket_count number of buckets
 * @param bucket array of power measurements
 * @param offset zero-based offset into graph
 * @param max_items to return in buffer
 * @param buffer to receive graph
 * @return number of items put in buffer
 */
static unsigned energy_graph(
    const unsigned bucket_size_ms,
    const unsigned bucket_count,
    const energy_watts_t* const bucket,
    unsigned offset,
    unsigned max_items,
    energy_watts_t* buffer
)
{
    /* Range check */
    if (offset >= bucket_count)
    {
        return 0;
    }
    if ((offset+max_items) > bucket_count)
    {
        max_items = bucket_count-offset;
    }

    /* Find latest entry and whether buckets have rolled over */
    unsigned index = energy_history.milliseconds / bucket_size_ms;
    bool rollover = energy_history.rollover || index >= bucket_count;
    index %= bucket_count;
    if (index >= offset)
    {
        index -= offset;
    }
    else if (rollover)
    {
        index += bucket_count-offset;
    }
    else
    {
        return 0;
    }

    /* Copy in data, reversing order as graph data is latest first */
    unsigned items;
    for (items = 0; items < max_items; items++)
    {
        if (index == 0)
        {
            if (!rollover)
            {
                break;
            }
            index = bucket_count;
        }
        index--;
        buffer[items] = bucket[index];
    }
    return items;
}

unsigned energy_graph_3s(unsigned offset, unsigned max_items, energy_watts_t* buffer)
{
    return energy_graph(3000U, 30U, energy_history.sec3,
                        offset, max_items, buffer);
}

unsigned energy_graph_90s(unsigned offset, unsigned max_items, energy_watts_t* buffer)
{
    return energy_graph(90000U, 30U, energy_history.sec90,
                        offset, max_items, buffer);
}

unsigned energy_graph_45m(unsigned offset, unsigned max_items, energy_watts_t* buffer)
{
    return energy_graph(45U*60000U, 32U, energy_history.min45,
                        offset, max_items, buffer);
}

unsigned energy_graph_24h(unsigned offset, unsigned max_items, energy_watts_t* buffer)
{
    return energy_graph(24U*60U*60000U, ENERGY_HISTORY_DAYS, energy_history.hour24,
                        offset, max_items, buffer);
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

/**
 * @brief Add average power to history
 * @param watts average power
 * @param milliseconds over which power was averaged, range 0..3000
 */
static void energy_record(energy_watts_t watts, unsigned milliseconds)
{
    unsigned index = (energy_history.milliseconds / 3000U);
    LOG_DBG("energy_record(%u, %u) index=%u\n", watts, milliseconds, index);

    /* Bump timestamp */
    energy_history.milliseconds += milliseconds;
    if (energy_history.milliseconds >= ENERGY_MILLISECONDS_ROLLOVER)
    {
        energy_history.milliseconds -= ENERGY_MILLISECONDS_ROLLOVER;
        energy_history.rollover = true;
    }

    /* Find 3s record */
    unsigned index_3s = index % 30U;
    index /= 30U;
    energy_history.sec3[index_3s] = watts;

    /* Calculate new 90s record ?*/
    if (index_3s == 29U)
    {
        unsigned sum_watts;
        unsigned index_90s = index % 30U;
        index /= 30U;

        sum_watts = 0;
        for (unsigned i = 0; i < 30; i++)
        {
            sum_watts += energy_history.sec3[i];
        }
        energy_history.sec90[index_90s] = (sum_watts+15U)/30U;

        /* Calculate new 45m record? */
        if (index_90s == 29)
        {
            unsigned index_45m = index % 32U;
            index /= 32U;

            sum_watts = 0;
            for (unsigned i = 0; i < 30; i++)
            {
                sum_watts += energy_history.sec90[i];
            }
            energy_history.min45[index_45m] = (sum_watts+15U)/30U;

            /* Calculate new 24h record? */
            if (index_45m == 31)
            {
                sum_watts = 0;
                for (unsigned i = 0; i < 32; i++)
                {
                    sum_watts += energy_history.min45[i];
                }
                energy_history.hour24[index] = (sum_watts+16U)/32U;
            }
        }
    }
    LOG_DBG("energy_history.milliseconds=%u\n", energy_history.milliseconds);
    LOG_DBG("energy_history.millijoules=%u\n", energy_history.millijoules);
    LOG_DBG("energy_history.sec3[]=");
    for (unsigned i = 0; i < 30; i++)
        LOG_DBG("%u%c", energy_history.sec3[i], i < 29 ? ' ' : '\n');

}

void energy_1Wh(unsigned milliseconds)
{
    unsigned duration_ms = milliseconds - energy_status.update_milliseconds;

    /* 1Wh == 3600Ws */
    energy_watts_t watts = 0;
    if (milliseconds)
    {
        energy_status.latest_milliwatts = (3600U*1000U*1000U)/milliseconds;
        watts = (energy_status.latest_milliwatts+500U)/1000U;
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

    /* Rollover 3s boundary? */
    unsigned sub3s_ms = 3000-(energy_history.milliseconds % 3000);
    if (milliseconds < sub3s_ms)
    {
        sub3s_ms = milliseconds;
    }
    else
    {
        /* Accumulate energy to 3s boundary */
        energy_record((energy_history.millijoules + sub3s_ms*watts + 500U)/3000U, sub3s_ms);
        energy_history.millijoules = 0;

        /* Register continuous 3s chunks */
        for (sub3s_ms = milliseconds-sub3s_ms; sub3s_ms >= 3000U; sub3s_ms -= 3000U)
        {
            energy_record(watts, 3000U);
        }
    }

    /* Accumulate remaining energy */
    energy_history.millijoules += sub3s_ms*watts;
    energy_history.milliseconds += sub3s_ms;

    /* Update observer */
    energy_status.update_milliseconds = milliseconds;
    if (energy_update)
    {
        energy_update();
    }
}
