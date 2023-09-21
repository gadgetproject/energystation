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
    const struct bt_gatt_attr *pwh_attr;
    const struct bt_gatt_attr *cw_attr;
    const struct bt_gatt_attr *eus_attr;
} gatt_db;

/**
 * @brief A database change has occurred; notify remotes
 */
static void gatt_update(void)
{
    energy_status_t status;
    energy_read(&status);

    printk("Energy=%u.%03ukWh\n", status.total_kilowatt_hours, status.part_milliwatt_hours/1000);
    printk("Power=%umW\n", status.latest_milliwatts);
    printk("Period=%ums\n", status.update_milliseconds);

    /* Notify remote */
    if (gatt_db.tkwh_attr)
    {
        static uint32_t tkwh;
        tkwh = status.total_kilowatt_hours;
        /* Littleendian */
        (void)bt_gatt_notify(NULL, gatt_db.tkwh_attr,
                             &tkwh, sizeof(tkwh));
    }
    if (gatt_db.pwh_attr)
    {
        static uint16_t pwh;
        pwh = status.part_milliwatt_hours/1000;
        /* Littleendian */
        (void)bt_gatt_notify(NULL, gatt_db.pwh_attr,
                             &pwh, sizeof(pwh));
    }
    if (gatt_db.cw_attr)
    {
        static uint16_t watts;
        watts = status.latest_milliwatts < (0xFFFFU*1000U)
              ? (status.latest_milliwatts+500) / 1000U
              : 0xFFFFU;
        /* Littleendian */
        (void)bt_gatt_notify(NULL, gatt_db.cw_attr,
                             &watts, sizeof(watts));
    }
    if (gatt_db.eus_attr)
    {
        static uint16_t seconds;
        seconds = status.update_milliseconds < (0xFFFF*1000U)
                ? (status.update_milliseconds+999) / 1000U
                : 0xFFFFU;
        /* Littleendian */
        (void)bt_gatt_notify(NULL, gatt_db.eus_attr,
                             &seconds, sizeof(seconds));
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
    energy_status_t status;
    energy_read(&status);

    static uint32_t tkwh;
    tkwh = status.total_kilowatt_hours;

    /* Littleendian */
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &tkwh, sizeof(tkwh));
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

static ssize_t gatt_read_pwh(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    energy_status_t status;
    energy_read(&status);

    static uint16_t pwh;
    pwh = status.part_milliwatt_hours/1000;

    /* Littleendian */
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &pwh, sizeof(pwh));
}

static ssize_t gatt_write_pwh(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset,
                              uint8_t flags)
{
    uint16_t pwh;

    if (offset)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(pwh) || !buf)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    /* Littleendian */
    (void)memcpy(&pwh, buf, sizeof(pwh));

    energy_status_t status;
    energy_read(&status);
    energy_reset(status.total_kilowatt_hours, 1000U*(uint32_t)pwh);

    return len;
}

static ssize_t gatt_read_cw(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    energy_status_t status;
    energy_read(&status);

    static uint16_t watts;
    watts = status.latest_milliwatts < (0xFFFFU*1000U)
          ? (status.latest_milliwatts+500) / 1000U
          : 0xFFFFU;

    /* Littleendian */
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &watts,
                             sizeof(watts));
}

static ssize_t gatt_read_eus(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    energy_status_t status;
    energy_read(&status);

    static uint16_t seconds;
    seconds = status.update_milliseconds < (0xFFFF*1000U)
            ? (status.update_milliseconds+999) / 1000U
            : 0xFFFFU;

    /* Littleendian */
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &seconds,
                             sizeof(seconds));
}

static void gatt_tkwh_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.tkwh_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_pwh_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.pwh_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_cw_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.cw_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_eus_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.eus_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
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

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_PART_WH_VAL),
                           ( BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_pwh, gatt_write_pwh, NULL),
    BT_GATT_CCC(gatt_pwh_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_CURRENT_W_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_cw, NULL, NULL),
    BT_GATT_CCC(gatt_cw_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_EXPECTED_UPDATE_S_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_eus, NULL, NULL),
    BT_GATT_CCC(gatt_eus_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
);
