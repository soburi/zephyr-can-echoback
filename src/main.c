#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAN_ID_STANDARD_MASK    (0x7FFU)
#define CAN_ID_EXTENDED_MASK    (0x1FFFFFFFU)
#define CAN_ID_TYPE_MASK        (0xC0000000U)
#define CAN_ID_TYPE_STANDARD    (0x00U << 30)
#define CAN_ID_TYPE_FD_STANDARD (0x01U << 30)
#define CAN_ID_TYPE_EXTENDED    (0x02U << 30)
#define CAN_ID_TYPE_FD_EXTENDED (0x03U << 30)

extern int rcar_canfd_send(const struct device *dev, int ch, uint32_t id, const uint8_t *data, uint8_t len);
extern int rcar_canfd_poll_recv(const struct device *dev, int ch, uint32_t *id, uint8_t *len, uint8_t *data);

#define CAN_CH_MIN 3
#define CAN_CH_MAX 4

#define CAN_ID_VEHICLE_STATUS_1 1001U
#define CAN_ID_VEHICLE_STATUS_2 985U
#define CAN_ID_VEHICLE_STATUS_3 986U
#define CAN_ID_EV_POWERTRAIN    180U
#define CAN_ID_ESP              184U
#define CAN_ID_EPS_STATUS       192U
#define CAN_ID_BCM_STATUS_1     142U

#define SPEED_FULL_KMH      200U
#define ENGINE_FULL_RPM     8000U
#define PULSE_MIN_FREQ_HZ   100U
#define PULSE_MAX_FREQ_HZ   150U
#define PULSE_LOW_US        200U
#define PULSE_USEC_PER_SEC  1000000U
#define OUTPUT_UPDATE_MS    1U
#define DEBUG_PRINT_MS      1000U
#define ANSI_HOME           "\x1b[H"
#define ANSI_CLEAR_EOL      "\x1b[K"

BUILD_ASSERT(PULSE_MAX_FREQ_HZ > PULSE_MIN_FREQ_HZ);
BUILD_ASSERT(PULSE_LOW_US < (PULSE_USEC_PER_SEC / PULSE_MAX_FREQ_HZ));

enum led_index {
    LED_FAULT_PRESENT,
    LED_A,
    LED_B,
    LED_C,
    LED_D,
    LED_COUNT,
};

struct vehicle_state {
    uint32_t speed_kmh;
    uint32_t engine_rpm;
    uint8_t shift_position;
    bool left_turn;
    bool right_turn;
    bool eps_fault;
    bool abs_fault;
    bool motor_fault;
    bool left_seat_belt_fastened;
    bool right_seat_belt_fastened;
};

struct pulse_output {
    const char *name;
    enum led_index pin;
    uint32_t high_us;
    bool on;
    bool enabled;
};

struct debug_stats {
    uint32_t rx_total;
    uint32_t rx_decoded;
    uint32_t rx_bad_len;
    uint32_t rx_unknown;
    uint32_t last_id;
    uint8_t last_ch;
    uint8_t last_len;
    uint8_t last_data[8];
};

static const struct device *canfd = DEVICE_DT_GET(DT_NODELABEL(canfd));
static struct vehicle_state state = {
    .left_seat_belt_fastened = true,
    .right_seat_belt_fastened = true,
};
static struct debug_stats debug_stats;
static bool led_output_state[LED_COUNT];
K_MUTEX_DEFINE(state_lock);
K_SEM_DEFINE(app_ready, 0, 2);

#define CONNECTOR_NODE DT_NODELABEL(connector)

#define GPIO_DT_SPEC_GET_CHILD(node_id) GPIO_DT_SPEC_GET(node_id, gpios)

static struct gpio_dt_spec pins[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CONNECTOR_NODE, GPIO_DT_SPEC_GET_CHILD, (,))
};

BUILD_ASSERT(ARRAY_SIZE(pins) == LED_COUNT);

static const char *const led_names[] = {
    "FAULT_PRESENT",
    "A",
    "B",
    "C",
    "D",
};

BUILD_ASSERT(ARRAY_SIZE(led_names) == LED_COUNT);

int init_condition_check(void)
{
    if (!device_is_ready(canfd)) {
        printk("canfd not ready\n");
        return -1;
    }

    for (size_t i = 0; i < ARRAY_SIZE(pins); ++i) {
        int ret;

        if (!gpio_is_ready_dt(&pins[i])) {
            printk("GPIO%u not ready\n", (uint32_t)i);
            return -1;
        }

        ret = gpio_pin_configure_dt(&pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("GPIO%u configure failed: %d\n", (uint32_t)i, ret);
            return ret;
        }
    }
    return 0;
}

static uint32_t can_id_without_flags(uint32_t id)
{
    uint32_t id_flags = id & CAN_ID_TYPE_MASK;

    if ((id_flags == CAN_ID_TYPE_STANDARD) || (id_flags == CAN_ID_TYPE_FD_STANDARD)) {
        return id & CAN_ID_STANDARD_MASK;
    }

    return id & CAN_ID_EXTENDED_MASK;
}

static bool bit_is_set(const uint8_t *data, size_t len, size_t byte, uint8_t bit)
{
    if (byte >= len) {
        return false;
    }

    return ((data[byte] >> bit) & 0x1U) != 0U;
}

static bool decode_can_frame(uint32_t raw_id, const uint8_t *data, uint8_t len)
{
    uint32_t id = can_id_without_flags(raw_id);
    bool decoded = true;

    if (len != 8U) {
        k_mutex_lock(&state_lock, K_FOREVER);
        debug_stats.rx_bad_len++;
        k_mutex_unlock(&state_lock);
        return false;
    }

    k_mutex_lock(&state_lock, K_FOREVER);

    switch (id) {
    case CAN_ID_VEHICLE_STATUS_1:
        state.speed_kmh = (((uint16_t)data[0] << 7) | (data[1] >> 1)) / 64U;
        break;
    case CAN_ID_VEHICLE_STATUS_2:
        state.engine_rpm = ((data[2] * 256U) + data[3]) / 4U;
        break;
    case CAN_ID_VEHICLE_STATUS_3:
        state.left_turn = bit_is_set(data, len, 0, 1);
        state.right_turn = bit_is_set(data, len, 0, 2);
        break;
    case CAN_ID_EV_POWERTRAIN:
        state.motor_fault = bit_is_set(data, len, 0, 1);
        state.shift_position = (data[1] >> 2) & 0x07U;
        break;
    case CAN_ID_ESP:
        state.abs_fault = bit_is_set(data, len, 3, 3);
        break;
    case CAN_ID_EPS_STATUS:
        state.eps_fault = bit_is_set(data, len, 0, 6);
        break;
    case CAN_ID_BCM_STATUS_1:
        state.left_seat_belt_fastened = bit_is_set(data, len, 1, 4);
        state.right_seat_belt_fastened = bit_is_set(data, len, 1, 5);
        break;
    default:
        decoded = false;
        debug_stats.rx_unknown++;
        break;
    }

    if (decoded) {
        debug_stats.rx_decoded++;
    }

    k_mutex_unlock(&state_lock);

    return decoded;
}

static uint8_t pulse_percent_from_value(uint32_t value, uint32_t full_scale)
{
    uint32_t percent = (value * 100U) / full_scale;

    if (percent > 100U) {
        return 100U;
    }

    return percent;
}

static uint32_t pulse_frequency_from_percent(uint8_t percent)
{
    return PULSE_MIN_FREQ_HZ +
           (((PULSE_MAX_FREQ_HZ - PULSE_MIN_FREQ_HZ) * percent) / 100U);
}

static uint32_t pulse_high_us_from_percent(uint8_t percent)
{
    uint32_t frequency_hz = pulse_frequency_from_percent(percent);
    uint32_t period_us = PULSE_USEC_PER_SEC / frequency_hz;

    return period_us - PULSE_LOW_US;
}

static void led_set_raw(enum led_index pin, bool on)
{
    (void)gpio_pin_set_dt(&pins[pin], on ? 1 : 0);
    led_output_state[pin] = on;
}

static void led_set_logged(enum led_index pin, bool on, const char *reason)
{
    ARG_UNUSED(reason);

    led_set_raw(pin, on);
}

static void led_toggle_logged(enum led_index pin, const char *reason)
{
    led_set_logged(pin, !led_output_state[pin], reason);
}

static struct pulse_output speed_pulse = {
    .name = "speed",
    .pin = LED_C,
};
static struct pulse_output engine_pulse = {
    .name = "engine",
    .pin = LED_D,
};

static void pulse_output_step(struct pulse_output *pulse, struct k_timer *timer)
{
    if (!pulse->enabled) {
        led_set_raw(pulse->pin, false);
        return;
    }

    if (!pulse->on) {
        pulse->on = true;
        led_set_raw(pulse->pin, true);
        k_timer_start(timer, K_USEC(pulse->high_us), K_NO_WAIT);
        return;
    }

    pulse->on = false;
    led_set_raw(pulse->pin, false);
    k_timer_start(timer, K_USEC(PULSE_LOW_US), K_NO_WAIT);
}

static void speed_pulse_timer_handler(struct k_timer *timer)
{
    pulse_output_step(&speed_pulse, timer);
}

static void engine_pulse_timer_handler(struct k_timer *timer)
{
    pulse_output_step(&engine_pulse, timer);
}

K_TIMER_DEFINE(speed_pulse_timer, speed_pulse_timer_handler, NULL);
K_TIMER_DEFINE(engine_pulse_timer, engine_pulse_timer_handler, NULL);

static void pulse_output_stop(struct pulse_output *pulse, struct k_timer *timer, const char *reason)
{
    k_timer_stop(timer);
    pulse->high_us = 0U;
    pulse->on = false;
    pulse->enabled = false;
    led_set_logged(pulse->pin, false, reason);
}

static void pulse_output_set_percent(struct pulse_output *pulse, struct k_timer *timer,
                                     uint8_t percent)
{
    uint32_t high_us = pulse_high_us_from_percent(percent);

    if (pulse->enabled && (pulse->high_us == high_us)) {
        return;
    }

    pulse->high_us = high_us;
    pulse->on = false;
    pulse->enabled = true;
    led_set_raw(pulse->pin, false);
    k_timer_start(timer, K_NO_WAIT, K_NO_WAIT);
}

static void led_task(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    k_sem_take(&app_ready, K_FOREVER);

    for (;;) {
        struct vehicle_state snapshot;
        bool seat_belt_open;
        bool diagnostic_active;
        bool turn_active;

        k_mutex_lock(&state_lock, K_FOREVER);
        snapshot = state;
        k_mutex_unlock(&state_lock);

        seat_belt_open =
            !snapshot.left_seat_belt_fastened || !snapshot.right_seat_belt_fastened;
        diagnostic_active = snapshot.eps_fault || snapshot.abs_fault ||
                    snapshot.motor_fault || seat_belt_open;
        turn_active = snapshot.left_turn || snapshot.right_turn;

        led_set_logged(LED_FAULT_PRESENT, diagnostic_active, "fault-present");

        if (diagnostic_active) {
            /* led1..led4 encode EPS, ABS, motor, and seat-belt diagnostics. */
            led_set_logged(LED_A, snapshot.eps_fault, "eps-fault");
            led_set_logged(LED_B, snapshot.abs_fault, "abs-fault");
            pulse_output_stop(&speed_pulse, &speed_pulse_timer, "diagnostic-mode");
            pulse_output_stop(&engine_pulse, &engine_pulse_timer, "diagnostic-mode");
            led_set_logged(LED_C, snapshot.motor_fault, "motor-fault");
            led_set_logged(LED_D, seat_belt_open, "seat-belt-open");
        } else if (turn_active) {
            pulse_output_stop(&speed_pulse, &speed_pulse_timer, "turn-mode");
            pulse_output_stop(&engine_pulse, &engine_pulse_timer, "turn-mode");
            led_set_logged(LED_A, snapshot.left_turn, "left-turn");
            led_set_logged(LED_B, snapshot.right_turn, "right-turn");
        } else {
            uint8_t speed_percent = pulse_percent_from_value(snapshot.speed_kmh,
                                    SPEED_FULL_KMH);
            uint8_t engine_percent = pulse_percent_from_value(snapshot.engine_rpm,
                                     ENGINE_FULL_RPM);

            pulse_output_set_percent(&speed_pulse, &speed_pulse_timer, speed_percent);
            pulse_output_set_percent(&engine_pulse, &engine_pulse_timer, engine_percent);

            led_set_logged(LED_A, snapshot.left_turn, "left-turn");
            led_set_logged(LED_B, snapshot.right_turn, "right-turn");
        }

        k_sleep(K_MSEC(OUTPUT_UPDATE_MS));
    }
}

static void can_rx_task(void *arg1, void *arg2, void *arg3)
{
    uint8_t tx[64] = {0};
    uint8_t rx[64];
    uint32_t id, id_flags, id_without_flags;
    uint8_t len = 8;
    uint32_t cnt = 1;
    volatile uint32_t dummy;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    ARG_UNUSED(id_flags);
    ARG_UNUSED(id_without_flags);

    printk("start can_rx_task\n");

    k_sem_take(&app_ready, K_FOREVER);

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

                k_mutex_lock(&state_lock, K_FOREVER);
                debug_stats.rx_total++;
                debug_stats.last_ch = ch;
                debug_stats.last_id = can_id_without_flags(id);
                debug_stats.last_len = len;
                for (uint8_t i = 0U; (i < len) && (i < ARRAY_SIZE(debug_stats.last_data)); ++i) {
                    debug_stats.last_data[i] = rx[i];
                }
                k_mutex_unlock(&state_lock);

                (void)decode_can_frame(id, rx, len);

#if 0
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
#endif
            }
        }

        k_sleep(K_MSEC(1));
    }
}

static void print_debug_status(struct debug_stats *prev)
{
    struct vehicle_state snapshot;
    struct debug_stats stats;
    bool seat_belt_open;
    uint32_t error_count;
    const char *turn_text;
    bool fault_printed = false;

    k_mutex_lock(&state_lock, K_FOREVER);
    snapshot = state;
    stats = debug_stats;
    k_mutex_unlock(&state_lock);

    seat_belt_open =
        !snapshot.left_seat_belt_fastened || !snapshot.right_seat_belt_fastened;
    error_count = stats.rx_unknown + stats.rx_bad_len;

    if (snapshot.left_turn && snapshot.right_turn) {
        turn_text = "Left|Right";
    } else if (snapshot.left_turn) {
        turn_text = "Left";
    } else if (snapshot.right_turn) {
        turn_text = "Right";
    } else {
        turn_text = "";
    }

    printk(ANSI_HOME);
    printk(ANSI_CLEAR_EOL "\n" ANSI_CLEAR_EOL "\n" ANSI_CLEAR_EOL "\n"
           ANSI_CLEAR_EOL "\n" ANSI_CLEAR_EOL "\n" ANSI_HOME);
    printk("[Rx]: %u err=%u [Last: %u:", stats.rx_total, error_count, stats.last_id);
    for (uint8_t i = 0U; (i < stats.last_len) && (i < ARRAY_SIZE(stats.last_data)); ++i) {
        printk(" %02x", stats.last_data[i]);
    }
    printk("]" ANSI_CLEAR_EOL "\n");
    printk("Speed = %u km/h" ANSI_CLEAR_EOL "\n", snapshot.speed_kmh);
    printk("Engine = %u rpm" ANSI_CLEAR_EOL "\n", snapshot.engine_rpm);
    printk("Turn = %s" ANSI_CLEAR_EOL "\n", turn_text);
    printk("fault =");
    if (snapshot.eps_fault) {
        printk("%sEPS", fault_printed ? "|" : " ");
        fault_printed = true;
    }
    if (snapshot.abs_fault) {
        printk("%sABS", fault_printed ? "|" : " ");
        fault_printed = true;
    }
    if (snapshot.motor_fault) {
        printk("%sMotor", fault_printed ? "|" : " ");
        fault_printed = true;
    }
    if (seat_belt_open) {
        printk("%sBelt", fault_printed ? "|" : " ");
    }
    printk(ANSI_CLEAR_EOL "\n");
    printk(ANSI_CLEAR_EOL);

    *prev = stats;
}

K_THREAD_DEFINE(led_tid, 2048, led_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(can_rx_tid, 2048, can_rx_task, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    struct debug_stats prev = {0};

    if (init_condition_check()) {
        printk("Invalid condition\n");
        return -1;
    }

    for (uint32_t i = 0; i < 2U; ++i) {
        k_sem_give(&app_ready);
    }

    for (;;) {
        k_sleep(K_MSEC(DEBUG_PRINT_MS));
        print_debug_status(&prev);
        //led_toggle_logged(LED_B, "debug-toggle");
    }
}
