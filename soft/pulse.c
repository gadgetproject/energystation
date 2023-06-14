/*! \file pulse.c
 *
 *  \brief Pulse detector
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

#include "pulse.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define PULSE_DEBOUNCE_PERIOD K_MSEC(10)    /**< time after which light level is stable */

static bool pulse_detected = false;
static struct k_timer pulse_debounce[2];    /**< ON/OFF debounce timers */
static uint64_t pulse_time_ms = 0;          /**< time of last pulse or 0 if unknown */
static pulse_cb pulse_callback = NULL;
static const struct gpio_dt_spec pulse_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(pulse), gpios);

/**
 * @brief Handle pulse interrupt debounce timer firing.
 * The timer is reset on each interrupt so this function is only called if
 * the light level is stable for PULSE_DEBOUNCE_PERIOD
 *
 * @param debounce timer object used to indicate ON or OFF transtion
 */
static void pulse_change(struct k_timer *debounce)
{
    bool on = (debounce == &pulse_debounce[true]);

    /* Glitches are transients that don't stabilise to a new state */
    if (on == pulse_detected)
    {
        printk("pulse glitch %d\n", on);
        return;
    }

    /* Report new pulse state */
    pulse_detected = on;
    if (pulse_detected)
    {
        uint64_t now_ms = (uint64_t)k_uptime_get();
        pulse_cb callback = __atomic_load_n(&pulse_callback, __ATOMIC_SEQ_CST);
        if (callback)
        {
            uint64_t duration = pulse_time_ms ? (now_ms-pulse_time_ms) : 0;
            callback(duration > UINT32_MAX ? 0 : (uint32_t)duration);
        }
        pulse_time_ms = now_ms;
    }
}

/**
 * @brief Handle pulse level change interrupt
 * @param dev
 * @param irq
 * @param pins
 */
static void pulse_isr(const struct device *dev, struct gpio_callback *irq, uint32_t pins)
{
    /* Figure out if light is detected or not */
    bool on = gpio_pin_get_dt(&pulse_pin) != 0;

    /* Debounce with one-shot timers; Zephyr documentation says:
     * - safe to use from ISRs
     * - safe to stop a timer that isn't running
     */
    k_timer_stop(&pulse_debounce[!on]);
    k_timer_start(&pulse_debounce[on], PULSE_DEBOUNCE_PERIOD, K_NO_WAIT);
}

bool pulse_init(void)
{
    /* Setup debounce timers */
    k_timer_init(&pulse_debounce[0], pulse_change, NULL);
    k_timer_init(&pulse_debounce[1], pulse_change, NULL);

    if (!device_is_ready(pulse_pin.port))
    {
        printk("pulse port not ready\n");
        return false;
    }

    int err;
    err = gpio_pin_configure_dt(&pulse_pin, GPIO_INPUT);
    if (err)
    {
        printk("gpio_pin_configure_dt(pulse)=%d\n", err);
        return false;
    }

    err = gpio_pin_interrupt_configure_dt(&pulse_pin, GPIO_INT_EDGE_BOTH);
    if (err)
    {
        printk("gpio_pin_interrut_configure_dt(pulse)=%d\n", err);
        return false;
    }

    static struct gpio_callback pulse_irq;
    gpio_init_callback(&pulse_irq, pulse_isr, BIT(pulse_pin.pin));

    err = gpio_add_callback(pulse_pin.port, &pulse_irq);
    if (err)
    {
        printk("gpio_add_callback(pulse)=%d\n", err);
        return false;
    }

    printk("pulse_init() ok");
    return true;
}

pulse_cb pulse_register(pulse_cb callback)
{
    return __atomic_exchange_n(&pulse_callback, callback, __ATOMIC_SEQ_CST);
}
