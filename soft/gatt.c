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
    const struct bt_gatt_attr *w_3s_attr;
    const struct bt_gatt_attr *w_90s_attr;
    const struct bt_gatt_attr *w_45m_attr;
    const struct bt_gatt_attr *w_24h_attr;
} gatt_db;

/**
 * @brief Fetch graph data from energy module
 * @param offset of value in octets
 * @param max_length of buffer in octets
 * @param value buffer
 * @return number of octets placed in value buffer
 */
static uint16_t gatt_fetch_graph(uint16_t offset, uint16_t max_length, void* value,
    unsigned (*energy_graph)(unsigned, unsigned, energy_watts_t*))
{
    if (offset % sizeof(energy_watts_t) || max_length < sizeof(energy_watts_t))
    {
        /* Invalid request */
        return 0;
    }

    /* Fetch system-endian data */
    energy_watts_t* buffer = value;
    unsigned count = energy_graph(offset/sizeof(energy_watts_t),
                                  max_length/sizeof(energy_watts_t),
                                  buffer);

    /* ASSUME system-endian is littleendian so no conversion required */

    return count*sizeof(energy_watts_t);
}


/**
 * @brief A database change has occurred; notify remotes
 * @param scope of update in milliseconds
 */
static void gatt_update(unsigned scope)
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
    if (scope >= 3000 && gatt_db.w_3s_attr)
    {
        static uint8_t watts[sizeof(energy_watts_t)];
        if (gatt_fetch_graph(0, sizeof(watts), watts, energy_graph_3s))
        {
            (void)bt_gatt_notify(NULL, gatt_db.w_3s_attr,
                                 watts, sizeof(watts));
        }
    }
    if (scope >= 90000 && gatt_db.w_90s_attr)
    {
        static uint8_t watts[sizeof(energy_watts_t)];
        if (gatt_fetch_graph(0, sizeof(watts), watts, energy_graph_90s))
        {
            (void)bt_gatt_notify(NULL, gatt_db.w_90s_attr,
                                 watts, sizeof(watts));
        }
    }
    if (scope >= 45*60000 && gatt_db.w_45m_attr)
    {
        static uint8_t watts[sizeof(energy_watts_t)];
        if (gatt_fetch_graph(0, sizeof(watts), watts, energy_graph_45m))
        {
            (void)bt_gatt_notify(NULL, gatt_db.w_45m_attr,
                                 watts, sizeof(watts));
        }
    }
    if (scope >= 24*60*60000 && gatt_db.w_24h_attr)
    {
        static uint8_t watts[sizeof(energy_watts_t)];
        if (gatt_fetch_graph(0, sizeof(watts), watts, energy_graph_24h))
        {
            (void)bt_gatt_notify(NULL, gatt_db.w_24h_attr,
                                 watts, sizeof(watts));
        }
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

static ssize_t gatt_read_w_3s(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint16_t octets = gatt_fetch_graph(offset, len, buf, energy_graph_3s);
    if (!octets)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    return octets;
}

static ssize_t gatt_read_w_90s(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint16_t octets = gatt_fetch_graph(offset, len, buf, energy_graph_90s);
    if (!octets)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    return octets;
}

static ssize_t gatt_read_w_45m(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint16_t octets = gatt_fetch_graph(offset, len, buf, energy_graph_45m);
    if (!octets)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    return octets;
}

static ssize_t gatt_read_w_24h(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint16_t octets = gatt_fetch_graph(offset, len, buf, energy_graph_24h);
    if (!octets)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    return octets;
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

static void gatt_w_3s_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.w_3s_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_w_90s_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.w_90s_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_w_45m_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.w_45m_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
}

static void gatt_w_24h_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value)
{
    /* Store for gatt_update() */
    gatt_db.w_24h_attr = (value & BT_GATT_CCC_NOTIFY) ? attr-1 : 0;
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

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_GRAPH_W_3S_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_w_3s, NULL, NULL),
    BT_GATT_CCC(gatt_w_3s_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_GRAPH_W_90S_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_w_90s, NULL, NULL),
    BT_GATT_CCC(gatt_w_90s_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_GRAPH_W_45M_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_w_45m, NULL, NULL),
    BT_GATT_CCC(gatt_w_45m_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(GATT_UUID_GRAPH_W_24H_VAL),
                           ( BT_GATT_CHRC_READ
                             | BT_GATT_CHRC_NOTIFY),
                           ( BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                           gatt_read_w_24h, NULL, NULL),
    BT_GATT_CCC(gatt_w_24h_cfg_changed,
                (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
);
