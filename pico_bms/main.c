// ROV - Pico 2 "BMS": odczyt BMS Daly po CAN (can2040 na PIO0 + transceiver SN65HVD230) jako wezel micro-ROS.
//
//  Pinout: GP4 = CAN TX (-> CTX transceivera), GP5 = CAN RX (<- CRX), 3V3, GND. CAN 250 kbit/s.
//  Protokol Daly CAN: ID rozszerzone 0x18 | DataID | dst | src. Host (0x40) pyta BMS (0x01):
//    zapytanie  0x18<DID>0140, 8 bajtow 0x00
//    odpowiedz  0x18<DID>4001, 8 bajtow danych (0x95/0x96 = kilka ramek, bajt 0 = numer ramki)
//  DataID: 0x90 V/I/SOC, 0x91 max/min ogniwo, 0x92 max/min temp, 0x93 MOS/pojemnosc, 0x94 stan,
//          0x95 napiecia ogniw, 0x96 temperatury, 0x97 balans, 0x98 bledy.
//  Publikuje: /rov/bms/status (rov_msgs/BmsStatus, 2 Hz), /rov/battery (sensor_msgs/BatteryState, 2 Hz),
//             /rov/bms/info (std_msgs/String, 1 Hz: diagnostyka CAN, widziane ID).
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/color_rgba.h>
#include <std_msgs/msg/u_int8.h>
#include <rov_msgs/msg/bms_status.h>
#include "hardware/pio.h"
#include "ws2812.pio.h"

#include "microros_transport.h"
#include "can2040.h"

#define CAN_TX_PIN   4
#define CAN_RX_PIN   5
// Zasilanie pojazdu: SSR (IN+ -> GP6, IN- -> GND). Wylaczony podczas LADOWANIA wykrytego z BMS po CAN
// (0x93 stan == 1 ladowanie albo 0x94 ladowarka podlaczona albo prad > +CHG_CURRENT_A), z histereza czasowa:
// wylacz po CHG_OFF_MS ladowania, wlacz po CHG_ON_MS bez ladowania. Gdy BMS milczy -> SSR zostaje jak byl
// (po starcie: ON, zeby pojazd dalo sie uruchomic bez BMS). Opcjonalny kontaktron na GP7 (3V3, pull-down):
// jesli USE_REED=1, SSR wymaga dodatkowo magnesu. Pico BMS musi byc zasilane PRZED SSR (bateria -> 5 V -> VSYS).
// /rov/power (std_msgs/Bool): false = odetnij na POWER_CYCLE_MS i wroc (zdalny restart), true = anuluj odciecie.
#define SSR_PIN        6
// SSR_ACTIVE_LOW 1 = SSR przez tranzystor NPN/N-MOSFET (GP6 -> 1k -> baza, pull-up 4.7k GP6->3V3, kolektor -> SSR IN-, SSR IN+ -> 5V):
//   GP6 wolny/wysoki = SSR ON (takze w BOOTSEL podczas flashowania - Jetson nie traci zasilania), GP6 = 0 -> SSR OFF.
// SSR_ACTIVE_LOW 0 = SSR IN+ prosto na GP6 (stara wersja; podczas flashowania SSR puszcza!).
#define SSR_ACTIVE_LOW 0
#define SSR_WRITE(on)  gpio_put(SSR_PIN, SSR_ACTIVE_LOW ? !(on) : (on))
#define REED_PIN       7
#define USE_REED       0
#define REED_DEBOUNCE_MS 200
#define POWER_CYCLE_MS 5000
#define CHG_CURRENT_A  0.5f
#define CHG_OFF_MS     2000
#define CHG_ON_MS      5000
// Ring LED WS2812B: DIN -> GP8 (PIO1). Topic /rov/led (std_msgs/ColorRGBA: r,g,b 0..1, a = jasnosc 0..1).
#define LED_PIN        8
#define LED_COUNT      16
#define LED_PIO        pio1
// Czujnik zalania (plytka CD4093): OUT1 -> GP2, OUT2 -> GP3, Uz -> 3V3, GND. Wysoki = ZALANIE. Topic /rov/leak (UInt8: bit0, bit1).
#define LEAK1_PIN      2
#define LEAK2_PIN      3
#define LEAK_DEBOUNCE_MS 300
#define CAN_BITRATE  250000
#define CAN_PIO      0

#define DALY_PRIO    0x18
#define DALY_HOST    0x40
#define DALY_BMS     0x01
#define POLL_MS      120        // odstep miedzy zapytaniami (BMS Daly potrzebuje ~50-100 ms)
#define PUB_MS       500
#define ONLINE_MS    5000
#define PING_MS      250

// ---- CAN ----
static struct can2040 cbus;
#define RING 64
static struct can2040_msg rx_ring[RING];
static volatile uint32_t rx_head, rx_tail;
static volatile uint32_t can_err;

static void can_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    (void)cd;
    if (notify == CAN2040_NOTIFY_RX) {
        uint32_t h = rx_head;
        if (((h + 1) % RING) != rx_tail) { rx_ring[h] = *msg; rx_head = (h + 1) % RING; }
        else can_err++;
    } else if (notify == CAN2040_NOTIFY_ERROR) can_err++;
}
static void pio_irq(void) { can2040_pio_irq_handler(&cbus); }

static void can_init(void) {
    can2040_setup(&cbus, CAN_PIO);
    can2040_callback_config(&cbus, can_cb);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);
    can2040_start(&cbus, clock_get_hz(clk_sys), CAN_BITRATE, CAN_RX_PIN, CAN_TX_PIN);
}

static bool daly_request(uint8_t did) {
    struct can2040_msg m;
    m.id = CAN2040_ID_EFF | ((uint32_t)DALY_PRIO << 24) | ((uint32_t)did << 16) | ((uint32_t)DALY_BMS << 8) | DALY_HOST;
    m.dlc = 8; memset(m.data, 0, 8);
    if (!can2040_check_transmit(&cbus)) return false;
    return can2040_transmit(&cbus, &m) == 0;
}

// ---- stan BMS ----
static rov_msgs__msg__BmsStatus bms;
static uint32_t last_rx_ms = 0, last_bms_ms = 0;
static struct { uint32_t id; uint32_t n; } seen[8];
static int seen_n;

static uint32_t ms_now(void) { return to_ms_since_boot(get_absolute_time()); }
static float clampf(float v) { if (!(v == v) || v < 0) return 0; return v > 1 ? 1 : v; }

static void note_id(uint32_t id) {
    for (int i = 0; i < seen_n; i++) if (seen[i].id == id) { seen[i].n++; return; }
    if (seen_n < 8) { seen[seen_n].id = id; seen[seen_n].n = 1; seen_n++; }
}

static void daly_parse(uint8_t did, const uint8_t *d) {
    switch (did) {
    case 0x90:
        bms.voltage_v = ((d[0] << 8) | d[1]) * 0.1f;
        bms.current_a = ((int)((d[4] << 8) | d[5]) - 30000) * 0.1f;
        bms.soc_pct   = ((d[6] << 8) | d[7]) * 0.1f;
        break;
    case 0x91:
        bms.max_cell_mv = (uint16_t)((d[0] << 8) | d[1]); bms.max_cell_no = d[2];
        bms.min_cell_mv = (uint16_t)((d[3] << 8) | d[4]); bms.min_cell_no = d[5];
        break;
    case 0x92:
        bms.max_temp_c = (int8_t)(d[0] - 40); bms.max_temp_no = d[1];
        bms.min_temp_c = (int8_t)(d[2] - 40); bms.min_temp_no = d[3];
        break;
    case 0x93:
        bms.state = d[0]; bms.charge_mos = d[1] != 0; bms.discharge_mos = d[2] != 0; bms.bms_life = d[3];
        bms.remaining_ah = (float)(((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7]) * 0.001f;
        break;
    case 0x94:
        bms.cell_count = d[0] > 48 ? 48 : d[0]; bms.temp_count = d[1] > 16 ? 16 : d[1];
        bms.charger_connected = d[2] != 0; bms.load_connected = d[3] != 0; bms.dio_bits = d[4];
        bms.cycles = (uint16_t)((d[5] << 8) | d[6]);
        break;
    case 0x95: {
        int f = d[0];                                   // numer ramki 1..16, 3 ogniwa na ramke
        if (f >= 1 && f <= 16)
            for (int k = 0; k < 3; k++) { int c = (f - 1) * 3 + k; if (c < 48) bms.cell_voltage_v[c] = ((d[1 + 2 * k] << 8) | d[2 + 2 * k]) * 0.001f; }
        break;
    }
    case 0x96: {
        int f = d[0];                                   // 7 temperatur na ramke
        if (f >= 1 && f <= 3)
            for (int k = 0; k < 7; k++) { int t = (f - 1) * 7 + k; if (t < 16) bms.temperature_c[t] = (int8_t)(d[1 + k] - 40); }
        break;
    }
    case 0x97: memcpy(bms.balance_bits, d, 8); break;
    case 0x98: memcpy(bms.fault_bits, d, 8); break;
    default: return;
    }
    last_bms_ms = ms_now();
}

static void can_drain(void) {
    while (rx_tail != rx_head) {
        struct can2040_msg m = rx_ring[rx_tail]; rx_tail = (rx_tail + 1) % RING;
        last_rx_ms = ms_now();
        uint32_t id = m.id & 0x1FFFFFFF;
        note_id(id);
        if (!(m.id & CAN2040_ID_EFF)) continue;
        uint8_t prio = id >> 24, did = (id >> 16) & 0xFF, dst = (id >> 8) & 0xFF, src = id & 0xFF;
        if (prio == DALY_PRIO && dst == DALY_HOST && src == DALY_BMS && m.dlc >= 8) daly_parse(did, m.data);
    }
}

// ---- micro-ROS ----
static rcl_allocator_t allocator; static rclc_support_t support; static rcl_node_t node; static rclc_executor_t executor;
static rcl_publisher_t pub_bms, pub_batt, pub_info, pub_power, pub_leak;
static std_msgs__msg__UInt8 msg_leak;
static rcl_subscription_t sub_leak_reset;
static std_msgs__msg__Bool msg_leak_reset;
static uint8_t leak_state, leak_latched;   // /rov/leak: bity 0-1 = stan biezacy, bity 4-5 = zatrzask (do resetu z GUI)
static void leak_update(uint32_t now) {    // debounce: stan musi sie utrzymac LEAK_DEBOUNCE_MS
    static uint8_t cand; static uint32_t since;
    uint8_t raw = (uint8_t)(gpio_get(LEAK1_PIN) | (gpio_get(LEAK2_PIN) << 1));
    if (raw != cand) { cand = raw; since = now; }
    else if (raw != leak_state && now - since >= LEAK_DEBOUNCE_MS) leak_state = raw;
    leak_latched |= leak_state;
}
static void cb_leak_reset(const void *msgin) { (void)msgin; leak_latched = 0; }
static rcl_subscription_t sub_power, sub_led;
static rcl_timer_t tmr_poll, tmr_pub, tmr_info;
static std_msgs__msg__Bool msg_power_cmd, msg_power_state;

// ---- zasilanie pojazdu: SSR = NIE ladowanie (z BMS, histereza) AND [kontaktron] AND brak zdalnego odciecia ----
static bool reed_state = true, ssr_state, charging, charging_raw, dsg_on = true;   // dsg_on = MOS rozladowania wg BMS (kontaktron na BMS)
static uint32_t reed_change_ms, power_cut_until, chg_since_ms, nochg_since_ms;
static void power_init(void) {
    gpio_init(REED_PIN); gpio_set_dir(REED_PIN, GPIO_IN); gpio_pull_down(REED_PIN);
    gpio_init(SSR_PIN); SSR_WRITE(1); gpio_set_dir(SSR_PIN, GPIO_OUT);   // stan ON ustawiony zanim pin stanie sie wyjsciem (bez przerwy na SSR)
    sleep_ms(5);
    if (USE_REED) reed_state = gpio_get(REED_PIN);
    ssr_state = reed_state;                 // start: wlaczone (chyba ze kontaktron mowi inaczej)
    SSR_WRITE(ssr_state);
}
static void power_update(uint32_t now) {
    if (USE_REED) {
        bool r = gpio_get(REED_PIN);
        if (r != reed_state) {
            if (!reed_change_ms) reed_change_ms = now;
            else if (now - reed_change_ms >= REED_DEBOUNCE_MS) { reed_state = r; reed_change_ms = 0; }
        } else reed_change_ms = 0;
    }
    // ladowanie wg BMS, tylko gdy dane swieze (< ONLINE_MS); bez BMS zostaje poprzedni stan
    if (last_bms_ms && now - last_bms_ms < ONLINE_MS) {
        dsg_on = bms.discharge_mos;          // kontaktron jest na BMS: brak magnesu -> MOS rozladowania OFF -> SSR OFF
        charging_raw = (bms.state == 1) || bms.charger_connected || (bms.current_a > CHG_CURRENT_A);
        if (charging_raw) { nochg_since_ms = 0; if (!chg_since_ms) chg_since_ms = now; if (!charging && now - chg_since_ms >= CHG_OFF_MS) charging = true; }
        else              { chg_since_ms = 0;   if (!nochg_since_ms) nochg_since_ms = now; if (charging && now - nochg_since_ms >= CHG_ON_MS) charging = false; }
    }
    bool cut = power_cut_until && (int32_t)(power_cut_until - now) > 0;
    if (!cut) power_cut_until = 0;
    bool want = reed_state && dsg_on && !charging && !cut;
    if (want != ssr_state) { ssr_state = want; SSR_WRITE(want); }
}
// ---- ring LED WS2812B ----
static uint led_sm;
static std_msgs__msg__ColorRGBA msg_led;
static void led_init(void) {
    uint offset = pio_add_program(LED_PIO, &ws2812_program);
    led_sm = pio_claim_unused_sm(LED_PIO, true);
    ws2812_program_init(LED_PIO, led_sm, offset, LED_PIN, 800000.0f);
}
static uint32_t led_px[LED_COUNT];              // GRB
static float man_r, man_g, man_b, man_a;        // kolor reczny z /rov/led (tlo)
static inline uint32_t grb(float r, float g, float b) {   // 0..1 -> GRB z prosta gamma 2.2
    float R = clampf(r), G = clampf(g), B = clampf(b);
    uint8_t ri = (uint8_t)(R * R * 255 + 0.5f), gi = (uint8_t)(G * G * 255 + 0.5f), bi = (uint8_t)(B * B * 255 + 0.5f);
    return ((uint32_t)gi << 16) | ((uint32_t)ri << 8) | bi;
}
static void led_show(void) { for (int i = 0; i < LED_COUNT; i++) pio_sm_put_blocking(LED_PIO, led_sm, led_px[i] << 8); }
static void led_fill(float r, float g, float b, float a) {
    uint32_t c = grb(r * clampf(a), g * clampf(a), b * clampf(a));
    for (int i = 0; i < LED_COUNT; i++) led_px[i] = c;
    led_show();
}
static void cb_led(const void *msgin) {
    const std_msgs__msg__ColorRGBA *m = msgin;
    man_r = m->r; man_g = m->g; man_b = m->b; man_a = clampf(m->a);
    if (!charging) led_fill(man_r, man_g, man_b, man_a);
}
// ---- animacje: start (kometa, 3 s) i ladowanie (zielone wypelnienie wg SOC + biegnacy impuls) ----
#define BOOT_ANIM_MS 3000
static uint32_t boot_t0;
static bool boot_done, anim_was_charging;
static void led_anim(uint32_t now) {
    static uint32_t last; if (now - last < 30) return; last = now;
    float t = now * 0.001f;
    if (!boot_done) {
        uint32_t el = now - boot_t0;
        if (el >= BOOT_ANIM_MS) { boot_done = true; led_fill(man_r, man_g, man_b, man_a); return; }
        float fade = el < BOOT_ANIM_MS - 600 ? 1.0f : (float)(BOOT_ANIM_MS - el) / 600.0f;   // wygaszenie na koncu
        float head = fmodf(el * 0.001f * 1.5f, 1.0f) * LED_COUNT;                         // 1.5 obrotu/s
        for (int i = 0; i < LED_COUNT; i++) {
            float d = fmodf(head - i + LED_COUNT, LED_COUNT);                               // odleglosc za glowa komety
            float v = d < 8 ? (1.0f - d / 8.0f) : 0;                                        // ogon 8 LED
            led_px[i] = grb(0.1f * v * fade, 0.4f * v * fade, 1.0f * v * fade);
        }
        led_show(); return;
    }
    if (charging) {
        anim_was_charging = true;
        float soc = bms.soc_pct > 0 ? bms.soc_pct / 100.0f : 0;
        float lit = soc * LED_COUNT;
        float pulse = fmodf(t * 0.6f, 1.0f) * (LED_COUNT + 6) - 3;                          // impuls raz na ~1.7 s
        float breath = 0.55f + 0.45f * sinf(t * 2.0f);
        for (int i = 0; i < LED_COUNT; i++) {
            float base = i < (int)lit ? 0.35f : (i == (int)lit ? 0.35f * (lit - (int)lit) : 0.02f);
            float d = fabsf(i - pulse); float p = d < 3 ? (1.0f - d / 3.0f) : 0;
            float v = base * breath + 0.6f * p * (i <= lit ? 1.0f : 0.3f);
            led_px[i] = grb(0.05f * v, v, 0.1f * v);
        }
        led_show(); return;
    }
    if (anim_was_charging) { anim_was_charging = false; led_fill(man_r, man_g, man_b, man_a); }   // koniec ladowania -> tlo
}

static void cb_power(const void *msgin) {
    const std_msgs__msg__Bool *m = msgin;
    if (m->data) power_cut_until = 0;                              // true = anuluj odciecie
    else power_cut_until = ms_now() + POWER_CYCLE_MS;              // false = odetnij na 5 s i wroc
    power_update(ms_now());
}
static sensor_msgs__msg__BatteryState batt;
static std_msgs__msg__String msg_info;
static char info_buf[300];
static float cell_buf[48], ctemp_buf[16];
static uint32_t agent_ok;

static void stamp(builtin_interfaces__msg__Time *t) {
    int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : (int64_t)time_us_64() * 1000;
    t->sec = (int32_t)(ns / 1000000000LL); t->nanosec = (uint32_t)(ns % 1000000000LL);
}

// Brak ACK na magistrali (BMS wylaczony / odpiety): can2040 ponawialby ramke bez konca (~1700 prob/s, pulsujacy prad
// transceivera = trzeszczaca przetwornica). Po 1 s bez udanej wysylki czyscimy kolejke i pytamy raz na sekunde.
static bool bus_backoff, bus_stopped;
static uint32_t backoff_since, backoff_try_ms;
static void bus_restart(void) { can2040_stop(&cbus); can2040_start(&cbus, clock_get_hz(clk_sys), CAN_BITRATE, CAN_RX_PIN, CAN_TX_PIN); bus_stopped = false; }
static void bus_watchdog(uint32_t now) {
    static uint32_t last_tx_total, last_tx_ok_ms;
    struct can2040_stats st; can2040_get_statistics(&cbus, &st);
    if (st.tx_total != last_tx_total || !last_tx_ok_ms) { last_tx_total = st.tx_total; last_tx_ok_ms = now; }
    if (rx_tail != rx_head || (last_rx_ms && now - last_rx_ms < 1000)) {
        if (bus_backoff) { bus_backoff = false; if (bus_stopped) bus_restart(); }
        last_tx_ok_ms = now;
    }
    if (!bus_backoff && now - last_tx_ok_ms > 1000 && st.tx_attempt) { bus_restart(); bus_backoff = true; backoff_since = now; backoff_try_ms = now; }
    // w trybie backoff: po 100 ms od proby bez sukcesu zatrzymaj magistrale (cisza) az do nastepnej proby
    if (bus_backoff && !bus_stopped && now - backoff_try_ms > 100 && st.tx_total == last_tx_total) { can2040_stop(&cbus); bus_stopped = true; }
}

static void cb_poll(rcl_timer_t *t, int64_t l) {
    (void)t; (void)l;
    static const uint8_t dids[] = { 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98 };
    static int i;
    static uint32_t last_backoff_try;
    uint32_t now = ms_now();
    if (bus_backoff) {
        if (now - last_backoff_try < 1000) return;
        last_backoff_try = now; backoff_try_ms = now;
        bus_restart();                      // czysta kolejka, jedna proba
        i = 0;
    }
    daly_request(dids[i]); i = (i + 1) % (int)sizeof dids;
}

static void cb_pub(rcl_timer_t *t, int64_t l) {
    (void)t; (void)l;
    uint32_t now = ms_now();
    struct can2040_stats st; can2040_get_statistics(&cbus, &st);
    stamp(&bms.header.stamp);
    bms.online = last_bms_ms && (now - last_bms_ms) < ONLINE_MS;
    bms.can_rx = st.rx_total; bms.can_tx = st.tx_total; bms.can_errors = st.parse_error + can_err;
    bms.age_ms = last_bms_ms ? now - last_bms_ms : 0xFFFFFFFFu;
    (void)rcl_publish(&pub_bms, &bms, NULL);

    batt.header.stamp = bms.header.stamp;
    batt.voltage = bms.voltage_v; batt.current = bms.current_a; batt.percentage = bms.soc_pct / 100.0f;
    batt.charge = bms.remaining_ah;
    batt.capacity = (bms.soc_pct > 1.0f) ? bms.remaining_ah * 100.0f / bms.soc_pct : 0.0f;   // szacunek
    batt.present = bms.online;
    batt.power_supply_status = !bms.online ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN :
        bms.state == 1 ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING :
        bms.state == 2 ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING :
                         sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
    batt.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    batt.cell_voltage.size = bms.cell_count; for (int i = 0; i < bms.cell_count; i++) cell_buf[i] = bms.cell_voltage_v[i];
    batt.cell_temperature.size = bms.temp_count; for (int i = 0; i < bms.temp_count; i++) ctemp_buf[i] = bms.temperature_c[i];
    batt.temperature = bms.max_temp_c;
    (void)rcl_publish(&pub_batt, &batt, NULL);
    msg_power_state.data = ssr_state;
    (void)rcl_publish(&pub_power, &msg_power_state, NULL);
    msg_leak.data = (uint8_t)(leak_state | (leak_latched << 4));
    (void)rcl_publish(&pub_leak, &msg_leak, NULL);
}

static void cb_info(rcl_timer_t *t, int64_t l) {
    (void)t; (void)l;
    uint32_t now = ms_now();
    struct can2040_stats st; can2040_get_statistics(&cbus, &st);
    int n = snprintf(info_buf, sizeof info_buf, "up=%lus ZALANIE=%s SSR=%s rozl.MOS=%s ladowanie=%s%s%s | can rx=%lu tx=%lu/%lu err=%lu bms=%s(%lu ms)%s ids:",
                     (unsigned long)(now / 1000), leak_state == 3 ? "TAK(1+2)" : leak_state == 1 ? "TAK(1)" : leak_state == 2 ? "TAK(2)" : (leak_latched ? "BYLO(do resetu)" : "nie"),
                     ssr_state ? "ON" : "off", dsg_on ? "ON" : "OFF", charging ? "TAK" : (charging_raw ? "wykryte.." : "nie"),
                     USE_REED ? (reed_state ? " magnes=1" : " magnes=0") : "", power_cut_until ? " [ODCIECIE ZDALNE]" : "",
                     (unsigned long)st.rx_total, (unsigned long)st.tx_total, (unsigned long)st.tx_attempt,
                     (unsigned long)(st.parse_error + can_err), bms.online ? "OK" : "BRAK",
                     (unsigned long)(last_rx_ms ? now - last_rx_ms : 999999),
                     bus_backoff ? " [BRAK ACK - magistrala pusta, pytam 1/s]" : "");
    for (int i = 0; i < seen_n && n < (int)sizeof info_buf - 20; i++) n += snprintf(info_buf + n, sizeof info_buf - n, " %08lX(%lu)", (unsigned long)seen[i].id, (unsigned long)seen[i].n);
    if (!seen_n) n += snprintf(info_buf + n, sizeof info_buf - n, " zadnych ramek");
    // diagnostyka linii: % probek w stanie wysokim na RX (GP5) i TX (GP4) w 20 ms (RX 100% = nic nie nadaje/brak echa, 0% = linia trzymana nisko)
    int hrx = 0, htx = 0;
    for (int i = 0; i < 200; i++) { hrx += gpio_get(CAN_RX_PIN); htx += gpio_get(CAN_TX_PIN); sleep_us(100); }
    n += snprintf(info_buf + n, sizeof info_buf - n, " | RX(GP5) high=%d%% TX(GP4) high=%d%%", hrx / 2, htx / 2);
    rosidl_runtime_c__String__assign(&msg_info, info_buf);
    (void)rcl_publish(&pub_info, &msg_info, NULL);
}

#define CHECK(x) do { if ((x) != RCL_RET_OK) return false; } while (0)
static bool create_entities(void) {
    allocator = rcl_get_default_allocator();
    CHECK(rclc_support_init(&support, 0, NULL, &allocator));
    CHECK(rclc_node_init_default(&node, "rov_bms", "", &support));
    CHECK(rclc_publisher_init_default(&pub_bms, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rov_msgs, msg, BmsStatus), "/rov/bms/status"));
    CHECK(rclc_publisher_init_default(&pub_batt, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/rov/battery"));
    CHECK(rclc_publisher_init_default(&pub_info, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rov/bms/info"));
    CHECK(rclc_publisher_init_default(&pub_power, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/rov/power/state"));
    CHECK(rclc_publisher_init_default(&pub_leak, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/rov/leak"));
    CHECK(rclc_subscription_init_default(&sub_power, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/rov/power"));
    CHECK(rclc_subscription_init_default(&sub_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "/rov/led"));
    CHECK(rclc_subscription_init_default(&sub_leak_reset, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/rov/leak/reset"));
    CHECK(rclc_timer_init_default(&tmr_poll, &support, RCL_MS_TO_NS(POLL_MS), cb_poll));
    CHECK(rclc_timer_init_default(&tmr_pub, &support, RCL_MS_TO_NS(PUB_MS), cb_pub));
    CHECK(rclc_timer_init_default(&tmr_info, &support, RCL_MS_TO_NS(1000), cb_info));
    executor = rclc_executor_get_zero_initialized_executor();
    CHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    CHECK(rclc_executor_add_subscription(&executor, &sub_power, &msg_power_cmd, cb_power, ON_NEW_DATA));
    CHECK(rclc_executor_add_subscription(&executor, &sub_leak_reset, &msg_leak_reset, cb_leak_reset, ON_NEW_DATA));
    CHECK(rclc_executor_add_subscription(&executor, &sub_led, &msg_led, cb_led, ON_NEW_DATA));
    CHECK(rclc_executor_add_timer(&executor, &tmr_poll));
    CHECK(rclc_executor_add_timer(&executor, &tmr_pub));
    CHECK(rclc_executor_add_timer(&executor, &tmr_info));
    (void)rmw_uros_sync_session(1000);
    return true;
}
static void destroy_entities(void) {
    rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    (void)rcl_publisher_fini(&pub_bms, &node); (void)rcl_publisher_fini(&pub_batt, &node); (void)rcl_publisher_fini(&pub_info, &node);
    (void)rcl_publisher_fini(&pub_power, &node); (void)rcl_publisher_fini(&pub_leak, &node); (void)rcl_subscription_fini(&sub_power, &node); (void)rcl_subscription_fini(&sub_led, &node);
    (void)rcl_subscription_fini(&sub_leak_reset, &node);
    (void)rcl_timer_fini(&tmr_poll); (void)rcl_timer_fini(&tmr_pub); (void)rcl_timer_fini(&tmr_info);
    (void)rclc_executor_fini(&executor); (void)rcl_node_fini(&node); (void)rclc_support_fini(&support);
}

static void msgs_init(void) {
    rov_msgs__msg__BmsStatus__init(&bms);
    rosidl_runtime_c__String__assign(&bms.header.frame_id, "bms");
    sensor_msgs__msg__BatteryState__init(&batt);
    rosidl_runtime_c__String__assign(&batt.header.frame_id, "bms");
    rosidl_runtime_c__String__assign(&batt.location, "rov");
    rosidl_runtime_c__String__assign(&batt.serial_number, "daly");
    batt.cell_voltage.data = cell_buf; batt.cell_voltage.capacity = 48; batt.cell_voltage.size = 0;
    batt.cell_temperature.data = ctemp_buf; batt.cell_temperature.capacity = 16; batt.cell_temperature.size = 0;
    batt.design_capacity = 0; batt.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    std_msgs__msg__String__init(&msg_info);
    std_msgs__msg__Bool__init(&msg_power_cmd); std_msgs__msg__Bool__init(&msg_power_state);
    std_msgs__msg__ColorRGBA__init(&msg_led);
    std_msgs__msg__UInt8__init(&msg_leak);
    std_msgs__msg__Bool__init(&msg_leak_reset);
}

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } uros_state_t;

int main(void) {
    power_init();              // najpierw SSR wg kontaktronu - zanim cokolwiek innego
    stdio_init_all();
    // GPIO23 = PS przetwornicy Pico: 1 = tryb PWM (bez skwierczenia cewki przy malym, pulsujacym obciazeniu transceivera)
    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);
    can_init();
    gpio_init(LEAK1_PIN); gpio_set_dir(LEAK1_PIN, GPIO_IN); gpio_pull_down(LEAK1_PIN);   // odpiety czujnik = sucho
    gpio_init(LEAK2_PIN); gpio_set_dir(LEAK2_PIN, GPIO_IN); gpio_pull_down(LEAK2_PIN);
    led_init(); led_fill(0, 0, 0, 0);
    boot_t0 = ms_now();                // animacja startowa (kometa) przez BOOT_ANIM_MS
    msgs_init();
    rmw_uros_set_custom_transport(true, NULL, pico_usb_transport_open, pico_usb_transport_close, pico_usb_transport_write, pico_usb_transport_read);
    uros_state_t state = WAITING_AGENT;
    uint32_t last_ping = 0, last_sync = 0, last_poll = 0;
    while (true) {
        uint32_t now = ms_now();
        power_update(now);
        leak_update(now);
        led_anim(now);
        bus_watchdog(now);
        can_drain();
        // bez agenta tez odpytuj BMS (zeby dane byly swieze od razu po polaczeniu)
        if (state != AGENT_CONNECTED && now - last_poll >= POLL_MS) { last_poll = now; cb_poll(NULL, 0); }
        switch (state) {
        case WAITING_AGENT:
            if (stdio_usb_connected() && now - last_ping >= 500) { last_ping = now; if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) state = AGENT_AVAILABLE; }
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) { state = AGENT_CONNECTED; agent_ok = 1; last_sync = now; } else { destroy_entities(); state = WAITING_AGENT; }
            break;
        case AGENT_CONNECTED:
            if (now - last_ping >= PING_MS) { last_ping = now; if (!stdio_usb_connected() || rmw_uros_ping_agent(50, 2) != RMW_RET_OK) { state = AGENT_DISCONNECTED; break; } }
            if (now - last_sync >= 60000) { last_sync = now; (void)rmw_uros_sync_session(200); }
            (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
            break;
        case AGENT_DISCONNECTED:
            agent_ok = 0; destroy_entities(); state = WAITING_AGENT; break;
        }
        if (state != AGENT_CONNECTED) sleep_ms(1);
    }
}
