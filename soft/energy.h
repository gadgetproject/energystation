/*! \file energy.h
 *
 *  \brief Energy monitor API
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

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise the energy monitor
 * @retval FALSE failed to start
 */
bool energy_init(void);

/**
 * Energy monitor status
 */
typedef struct
{
    uint32_t total_kilowatt_hours;  /**< Cumulative energy counter */
    uint32_t part_milliwatt_hours;  /**< Fractional part of total_kilowatt_hours */
    uint32_t latest_milliwatts;     /**< Instantaneous power */
    uint32_t update_milliseconds;   /**< Update interval or 0 if unknown */
} energy_status_t;

/**
 * @brief Snapshot the current status
 * @param [out] status receive buffer
 */
void energy_read(energy_status_t* status);

/**
 * @brief Energy monitor update callback
 */
typedef void (*energy_cb)(void);

/**
 * @brief Register the energy update callback function
 * @param callback to receive or NULL to stop
 * @returns previous callback or NULL if none
 */
energy_cb energy_register(energy_cb callback);

/**
 * @brief Reset the total kilowatt hours counter
 * @param tkwh total_kilowatt_hours value
 * @param pmwh part_milliwatt_hours value
 */
void energy_reset(uint32_t tkwh, uint32_t pmwh);

/**
 * @brief 1 Watt-hour unit of energy has been consumed on each call
 * @param milliseconds since previous unit or 0 if not known
 */
void energy_1Wh(unsigned milliseconds);
