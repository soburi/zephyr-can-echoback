#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

#define CAN_ID_STANDARD_MASK    (0x7FFU)
#define CAN_ID_EXTENDED_MASK    (0x1FFFFFFFU)
#define CAN_ID_TYPE_MASK        (0xC0000000U)
#define CAN_ID_TYPE_STANDARD    (0x00U << 30)
#define CAN_ID_TYPE_FD_STANDARD (0x01U << 30)
#define CAN_ID_TYPE_EXTENDED    (0x02U << 30)
#define CAN_ID_TYPE_FD_EXTENDED (0x03U << 30)

extern int rcar_canfd_send(const struct device *dev, int ch, uint32_t id, const uint8_t *data, uint8_t len);
extern int rcar_canfd_poll_recv(const struct device *dev, int ch, uint32_t *id, uint8_t *len, uint8_t *data);

int main(void)
{
    printk("Hello from can_echoback sample\n");
    const struct device *canfd = DEVICE_DT_GET(DT_NODELABEL(canfd));
    if (!device_is_ready(canfd)) {
        printk("canfd not ready\n");
        return 0;
    }

    uint8_t tx[64] = {0};
    uint8_t rx[64];
    uint32_t id, id_flags, id_without_flags;
    uint8_t len = 8;
    uint32_t cnt = 1;
    volatile uint32_t dummy;

    tx[0] = 0xff;
    rcar_canfd_send(canfd, 3, 0x123 | CAN_ID_TYPE_FD_EXTENDED, tx, 8);
    while(1) {
        for (int ch = 3; ch < 5; ++ch) {
            while (!rcar_canfd_poll_recv(canfd, ch, &id, &len, rx)) {
                // Debug log
                /*
                printk("RX(ch%d) id=0x%x len=%u data=", ch, id, len);
                for (int i=0;i<len;i++) printk("%02x ", rx[i]);
                printk("\n");
                */

                // echo back
                id_flags = id & CAN_ID_TYPE_MASK;
                if ( (id_flags == CAN_ID_TYPE_STANDARD) || (id_flags == CAN_ID_TYPE_FD_STANDARD) ) { // Standard ID
                    id_without_flags = (id+1) & CAN_ID_STANDARD_MASK;
                }
                else {
                    id_without_flags = (id+1) & CAN_ID_EXTENDED_MASK;
                }

                for (int i=0; i<len; ++i) {
                    tx[i] = rx[i] + 1;
                }
                rcar_canfd_send(canfd, ch, (id_flags | id_without_flags), tx, len);
            }
        }
    }
}
