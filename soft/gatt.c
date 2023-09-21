/*! \file gatt.c
 *
 *  \brief Bluetooth LE GATT Service Implementation
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

#include "gatt.h"
#include "energy.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/sys/printk.h>

static const struct bt_data gatt_adv[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, GATT_UUID_SERVICE),
};

static struct
{
    const struct bt_gatt_attr *tkwh_attr;
} gatt_db;

/**
 * @brief A database change has occurred; notify remotes
 */
static void gatt_update(void)
{
    static energy_status_t status;
    energy_read(&status);

    printk("Energy=%u.%03ukWh\n", status.total_kilowatt_hours, status.part_milliwatt_hours/1000);
    printk("Power=%umW\n", status.latest_milliwatts);
    printk("Period=%ums\n", status.update_milliseconds);

    /* Notify remote */
    if (gatt_db.tkwh_attr)
    {
        /* Littleendian */
        (void)bt_gatt_notify(NULL, gatt_db.tkwh_attr,
                             &status.total_kilowatt_hours,
                             sizeof(status.total_kilowatt_hours));
    }
}

bool gatt_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("bt_enable()=%d\n", err);
        return false;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME,
                          gatt_adv, ARRAY_SIZE(gatt_adv),
                          NULL, 0);
    if (err)
    {
        printk("bt_le_adv_start()=%d\n", err);
        return false;
    }

    if (energy_register(gatt_update))
    {
        printk("energy_register already\n");
        return false;
    }

    printk("gatt_init() ok\n");
    return true;
}

static ssize_t gatt_read_tkwh(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    static energy_status_t status;
    energy_read(&status);

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &status.total_kilowatt_hours,
                             sizeof(status.total_kilowatt_hours));
}

static ssize_t gatt_write_tkwh(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset,
                               uint8_t flags)
{
    uint32_t tkwh;

    if (offset)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(tkwh) || !buf)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    /* Littleendian */
    (void)memcpy(&tkwh, buf, sizeof(tkwh));
    energy_reset(tkwh, 0);

    return len;
}

static void gatt_tkwh_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for bt_gatt_notify() on write */
    gatt_db.tkwh_attr = attr-1;
}

BT_GATT_SERVICE_DEFINE(gatt_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(GATT_UUID_SERVICE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_TOTAL_KWH_VAL),
                           ( BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_tkwh, gatt_write_tkwh, NULL),
    BT_GATT_CCC(gatt_tkwh_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
);
