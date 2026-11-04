// ROV - firmware Pico 2 (RP2350) jako wezel micro-ROS (Humble) po USB CDC.
//
//  core 1: petla DShot 1 kHz (8x bidirectional DShot, PIO) + rotacyjne zadanie telemetrii KISS
//  core 0: micro-ROS (rclc executor), KISS z UART0/UART1, ADC pradu, IMU LSM6DSO32, MS5837/MS5803
//
//  Publikuje:   /rov/imu (sensor_msgs/Imu, 50 Hz, best effort)
//               /rov/imu/temperature (sensor_msgs/Temperature, 5 Hz)
//               /rov/pressure/ms5837, /rov/pressure/ms5803 (sensor_msgs/FluidPressure, 10 Hz)
//               /rov/temperature/ms5837, /rov/temperature/ms5803 (sensor_msgs/Temperature, 10 Hz)
//               /rov/depth (std_msgs/Float32, m, z MS5837 = czujnik zewnetrzny, 10 Hz)
//               /rov/esc/telemetry (rov_msgs/EscTelemetry, 20 Hz, reliable)
//  Subskrybuje: /rov/thrusters (rov_msgs/ThrusterCommand) - gaz -1..1, failsafe po 500 ms ciszy
//               /rov/esc/command (rov_msgs/EscCommand)   - komendy DShot / konfiguracja
//  Serwisy:     /rov/depth/zero, /rov/imu/calibrate (std_srvs/Trigger)
//
//  Failsafe: silniki = 0 gdy brak agenta, brak ThrusterCommand > 500 ms albo odlaczone USB.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>
#include <rov_msgs/msg/esc_telemetry.h>
#include <rov_msgs/msg/thruster_command.h>
#include <rov_msgs/msg/esc_command.h>

#include "microros_transport.h"
#include "dshot.h"
#include "kiss.h"
#include "imu.h"
#include "press.h"

// ---- pinout (jak w zweryfikowanym firmware testowym esc_test/main.c) ----
// ESC #1 na lewej krawedzi (GP9-13, CUR na GP26), ESC #2 na prawej (GP16-19, GP21, GP28)
static const uint8_t MOTOR_PINS[N_MOTORS] = { 9, 10, 11, 12, 16, 17, 18, 19 };
#define TLM0_RX_PIN 13   // UART0 RX (AUX) <- ESC #1 TLM
#define TLM1_RX_PIN 21   // UART1 RX (AUX) <- ESC #2 TLM
#define CUR0_PIN    27   // ADC1 <- ESC #1 CUR (przeniesione z GP26, 2026-11-04)
#define CUR1_PIN    28   // ADC2 <- ESC #2 CUR
#define I2C_SDA_PIN 14   // I2C1 SDA <- IMU LSM6DSO32 + MS5837 (0x76) + MS5803 (0x77)
#define I2C_SCL_PIN 15   // I2C1 SCL
#define IMU_INT_PIN  6   // LSM6DSO32 INT1 (DRDY zyro)

#define DSHOT_LOOP_US    1000   // 1 kHz
#define TLM_REQ_MS       10     // co ile ms zadanie telemetrii KISS na ESC (rotacja 4 silnikow)
#define CMD_TIMEOUT_MS   500    // failsafe: brak ThrusterCommand
#define IMU_PUB_MS       20
#define IMU_TEMP_PUB_MS  200
#define PRESS_PUB_MS     100
#define TLM_PUB_MS       50
#define PING_MS          250

// ---- stan wspoldzielony core0 <-> core1 ----
static volatile uint16_t g_throttle[N_MOTORS];   // 0 lub 48..2047
static volatile uint16_t g_cmd_val[N_MOTORS];
static volatile uint8_t  g_cmd_cnt[N_MOTORS];    // ile razy jeszcze wyslac komende
static volatile bool     g_edt = false;
static volatile uint32_t g_rate = 300000;
static volatile uint32_t g_rate_req = 0;
static volatile uint32_t g_loop_cnt = 0;
static volatile int8_t   g_tlm_req_motor[2] = { -1, -1 };
static dshot_tlm_t g_tlm[N_MOTORS];

// ---- core1: petla DShot ----
static void core1_main(void) {
    dshot_init(MOTOR_PINS, g_rate);
    absolute_time_t next = get_absolute_time();
    uint16_t val[N_MOTORS]; bool tel[N_MOTORS];
    uint32_t last_req_ms[2] = { 0, 0 };
    uint8_t rr[2] = { 0, 0 };
    while (true) {
        next = delayed_by_us(next, DSHOT_LOOP_US);
        if (g_rate_req) { dshot_set_rate(g_rate_req); g_rate = g_rate_req; g_rate_req = 0; }
        uint32_t now = to_ms_since_boot(get_absolute_time());
        for (int m = 0; m < N_MOTORS; m++) tel[m] = false;
        for (int e = 0; e < 2; e++) {
            if (now - last_req_ms[e] >= TLM_REQ_MS) {
                last_req_ms[e] = now;
                int m = e * 4 + rr[e]; rr[e] = (uint8_t)((rr[e] + 1) & 3);
                tel[m] = true; g_tlm_req_motor[e] = (int8_t)m;
            }
        }
        for (int m = 0; m < N_MOTORS; m++) {
            if (g_cmd_cnt[m]) { val[m] = g_cmd_val[m]; tel[m] = true; g_cmd_cnt[m]--; }
            else val[m] = g_throttle[m];
        }
        dshot_send_all(val, tel, g_edt, g_tlm);
        g_loop_cnt++;
        sleep_until(next);
    }
}

// ---- UART telemetria KISS ----
static volatile uint8_t u_ring[2][256];
static volatile uint8_t u_head[2], u_tail[2];
static void uart_rx_irq(int e) {
    uart_inst_t *u = e ? uart1 : uart0;
    while (uart_is_readable(u)) {
        uint8_t b = uart_getc(u);
        uint8_t h = u_head[e];
        u_ring[e][h] = b; u_head[e] = (uint8_t)(h + 1);
    }
}
static void uart0_irq(void) { uart_rx_irq(0); }
static void uart1_irq(void) { uart_rx_irq(1); }
static void tlm_uart_init(uart_inst_t *u, uint rx_pin, uint irq, irq_handler_t h) {
    uart_init(u, 115200);
    // RX standardowo na GPIO 1,5,9,13,...; na pinie TX (0,4,8,12,...) uzyj UART_AUX (zamiana TX/RX, tylko RP2350)
    gpio_set_function(rx_pin, (rx_pin % 4 == 1) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX);
    uart_set_format(u, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(u, true);
    irq_set_exclusive_handler(irq, h);
    irq_set_enabled(irq, true);
    uart_set_irq_enables(u, true, false);
}

// ---- stan core0 ----
static kiss_parser_t k_par[2];
static kiss_tlm_t    k_tlm[N_MOTORS];
static int      g_poles = 14;
static bool     g_3d = false;
static float    g_cur_mv_per_a = 25.0f;   // BF scale 250 = 25 mV/A (niezweryfikowane)
static float    g_cur_off_mv[2] = { 0, 0 };
static imu_t    g_imu;
static press_bus_t g_press;
static volatile uint32_t g_last_cmd_ms = 0;
static bool     g_failsafe = true;
static uint32_t g_agent_connected = 0;

static void stop_all(void) { for (int m = 0; m < N_MOTORS; m++) g_throttle[m] = 0; }
static bool any_running(void) { for (int m = 0; m < N_MOTORS; m++) if (g_throttle[m]) return true; return false; }

static uint16_t throttle_to_dshot(float v) {
    if (!(v == v)) return 0;                       // NaN
    if (v > 1.0f) v = 1.0f; if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < 0.005f) return 0;
    if (g_3d) {
        // 3D: 48..1047 = kierunek 1 (ujemne), 1048..2047 = kierunek 2 (dodatnie)
        if (v < 0) return (uint16_t)(48 + (int)(-v * 999.0f + 0.5f));
        return (uint16_t)(1048 + (int)(v * 999.0f + 0.5f));
    }
    if (v < 0) return 0;
    return (uint16_t)(48 + (int)(v * 1999.0f + 0.5f));
}

static float read_cur_mv(int ch) {
    adc_select_input((uint)((ch ? CUR1_PIN : CUR0_PIN) - 26));
    uint32_t acc = 0;
    for (int i = 0; i < 32; i++) acc += adc_read();
    return (float)acc / 32.0f * 3300.0f / 4095.0f;
}

static void esc_send_cmd(uint8_t mask, int cmd, int n) {
    if (any_running()) return;    // komendy DShot tylko przy zatrzymanych silnikach
    for (int m = 0; m < N_MOTORS; m++) if (mask & (1 << m)) { g_cmd_val[m] = (uint16_t)cmd; g_cmd_cnt[m] = (uint8_t)n; }
}

// ---- micro-ROS ----
static rcl_allocator_t    allocator;
static rclc_support_t     support;
static rcl_node_t         node;
static rclc_executor_t    executor;
static rcl_publisher_t    pub_imu, pub_imu_temp, pub_press[2], pub_temp[2], pub_depth, pub_tlm, pub_status;
static rcl_subscription_t sub_thr, sub_cmd;
static rcl_service_t      srv_zero, srv_imucal;
static rcl_timer_t        tmr_imu, tmr_press, tmr_tlm, tmr_status;
static std_msgs__msg__String msg_status;
static char status_buf[480];

static sensor_msgs__msg__Imu           msg_imu;
static sensor_msgs__msg__Temperature   msg_imu_temp, msg_temp[2];
static sensor_msgs__msg__FluidPressure msg_press[2];
static std_msgs__msg__Float32          msg_depth;
static rov_msgs__msg__EscTelemetry     msg_tlm;
static rov_msgs__msg__ThrusterCommand  msg_thr;
static rov_msgs__msg__EscCommand       msg_cmd;
static std_srvs__srv__Trigger_Request  req_zero, req_imucal;
static std_srvs__srv__Trigger_Response res_zero, res_imucal;

static uint32_t ms_now(void) { return to_ms_since_boot(get_absolute_time()); }

static void stamp(builtin_interfaces__msg__Time *t) {
    int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : (int64_t)time_us_64() * 1000;
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void cb_thrusters(const void *msgin) {
    const rov_msgs__msg__ThrusterCommand *m = msgin;
    for (int i = 0; i < N_MOTORS; i++) g_throttle[i] = throttle_to_dshot(m->throttle[i]);
    g_last_cmd_ms = ms_now();
}

static void cb_esc_command(const void *msgin) {
    const rov_msgs__msg__EscCommand *m = msgin;
    uint8_t mask = m->motor_mask, cmd = m->command, rep = m->repeat;
    switch (cmd) {
    case rov_msgs__msg__EscCommand__CMD_SET_POLES:    if (rep >= 2 && rep <= 64) g_poles = rep & ~1; return;
    case rov_msgs__msg__EscCommand__CMD_SET_RATE: {
        static const uint32_t rates[5] = { 0, 150000, 300000, 600000, 1200000 };
        if (rep >= 1 && rep <= 4) g_rate_req = rates[rep];
        return;
    }
    case rov_msgs__msg__EscCommand__CMD_SET_3D:       g_3d = rep != 0; stop_all(); return;
    case rov_msgs__msg__EscCommand__CMD_CURRENT_ZERO: for (int e = 0; e < 2; e++) g_cur_off_mv[e] = read_cur_mv(e); return;
    case rov_msgs__msg__EscCommand__CMD_CURRENT_SCALE: if (rep > 0) g_cur_mv_per_a = (float)rep; return;
    case rov_msgs__msg__EscCommand__CMD_SET_EDT:      g_edt = rep != 0; return;
    case rov_msgs__msg__EscCommand__CMD_STOP:         stop_all(); return;
    default: break;
    }
    if (cmd > 47) return;
    if (rep == 0) rep = 6;
    esc_send_cmd(mask, cmd, rep);
    if (cmd == rov_msgs__msg__EscCommand__DSHOT_3D_ON)  { g_3d = true;  stop_all(); }
    if (cmd == rov_msgs__msg__EscCommand__DSHOT_3D_OFF) { g_3d = false; stop_all(); }
    if (cmd == rov_msgs__msg__EscCommand__DSHOT_EDT_ON) g_edt = true;
}

static void cb_srv_zero(const void *req, void *resp) {
    (void)req;
    std_srvs__srv__Trigger_Response *r = resp;
    press_zero(&g_press);
    char buf[96];
    snprintf(buf, sizeof buf, "p0: MS5837=%.2f mbar, MS5803=%.2f mbar", g_press.s[0].p0_mbar, g_press.s[1].p0_mbar);
    r->success = g_press.s[0].present || g_press.s[1].present;
    rosidl_runtime_c__String__assign(&r->message, r->success ? buf : "brak czujnikow cisnienia");
}

static void cb_srv_imucal(const void *req, void *resp) {
    (void)req;
    std_srvs__srv__Trigger_Response *r = resp;
    r->success = g_imu.present;
    if (g_imu.present) imu_start_cal(&g_imu);
    rosidl_runtime_c__String__assign(&r->message, g_imu.present ? "kalibracja zyro: nie ruszac 2 s" : "brak IMU");
}

static void cb_tmr_imu(rcl_timer_t *t, int64_t last) {
    (void)t; (void)last;
    static uint32_t n;
    if (!g_imu.present) return;
    const float D2R = 0.017453293f, G = 9.80665f;
    stamp(&msg_imu.header.stamp);
    msg_imu.orientation.w = g_imu.q0; msg_imu.orientation.x = g_imu.q1;
    msg_imu.orientation.y = g_imu.q2; msg_imu.orientation.z = g_imu.q3;
    msg_imu.angular_velocity.x = g_imu.gx * D2R; msg_imu.angular_velocity.y = g_imu.gy * D2R; msg_imu.angular_velocity.z = g_imu.gz * D2R;
    msg_imu.linear_acceleration.x = g_imu.ax * G; msg_imu.linear_acceleration.y = g_imu.ay * G; msg_imu.linear_acceleration.z = g_imu.az * G;
    (void)rcl_publish(&pub_imu, &msg_imu, NULL);
    if (++n % (IMU_TEMP_PUB_MS / IMU_PUB_MS) == 0) {
        msg_imu_temp.header.stamp = msg_imu.header.stamp;
        msg_imu_temp.temperature = g_imu.temp_C;
        (void)rcl_publish(&pub_imu_temp, &msg_imu_temp, NULL);
    }
}

static void cb_tmr_press(rcl_timer_t *t, int64_t last) {
    (void)t; (void)last;
    for (int i = 0; i < 2; i++) {
        const press_t *s = &g_press.s[i];
        if (!s->present || !s->n_ok) continue;
        stamp(&msg_press[i].header.stamp);
        msg_press[i].fluid_pressure = s->p_mbar * 100.0;   // Pa
        msg_temp[i].header.stamp = msg_press[i].header.stamp;
        msg_temp[i].temperature = s->t_C;
        (void)rcl_publish(&pub_press[i], &msg_press[i], NULL);
        (void)rcl_publish(&pub_temp[i], &msg_temp[i], NULL);
    }
    // glebokosc: MS5837 (zewnetrzny) jesli jest, inaczej MS5803
    int d = g_press.s[0].present ? 0 : (g_press.s[1].present ? 1 : -1);
    if (d >= 0 && g_press.s[d].p0_mbar != 0) {
        msg_depth.data = press_depth_m(&g_press, d);
        (void)rcl_publish(&pub_depth, &msg_depth, NULL);
    }
}

static void cb_tmr_tlm(rcl_timer_t *t, int64_t last) {
    (void)t; (void)last;
    uint32_t now = ms_now();
    stamp(&msg_tlm.header.stamp);
    for (int m = 0; m < N_MOTORS; m++) {
        dshot_tlm_t d = g_tlm[m];
        kiss_tlm_t k = k_tlm[m];
        msg_tlm.dshot_value[m] = g_throttle[m];
        msg_tlm.erpm[m] = d.erpm;
        msg_tlm.rpm[m] = d.erpm / (uint32_t)(g_poles / 2);
        msg_tlm.bidir_ok[m] = d.n_ok && (now - d.last_ok_ms) < 200;
        msg_tlm.bidir_ok_count[m] = d.n_ok; msg_tlm.bidir_crc_errors[m] = d.n_crc; msg_tlm.bidir_no_reply[m] = d.n_none;
        msg_tlm.kiss_temp_c[m] = k.temp_C;
        msg_tlm.kiss_voltage_v[m] = k.volt_cV * 0.01f;
        msg_tlm.kiss_current_a[m] = k.curr_cA * 0.01f;
        msg_tlm.kiss_mah[m] = k.mAh;
        msg_tlm.kiss_erpm[m] = k.erpm;
        msg_tlm.kiss_age_ms[m] = k.valid ? now - k.t_ms : 0xFFFFFFFFu;
    }
    for (int e = 0; e < 2; e++) msg_tlm.esc_current_a[e] = (read_cur_mv(e) - g_cur_off_mv[e]) / g_cur_mv_per_a;
    msg_tlm.poles = (uint8_t)g_poles;
    msg_tlm.dshot_rate_kbit = (uint16_t)(g_rate / 1000);
    msg_tlm.mode_3d = g_3d;
    msg_tlm.failsafe = g_failsafe;
    msg_tlm.agent_connected = g_agent_connected != 0;
    msg_tlm.loop_count = g_loop_cnt;
    int32_t age = (int32_t)(now - g_last_cmd_ms);
    msg_tlm.cmd_age_ms = g_last_cmd_ms ? (uint32_t)(age < 0 ? 0 : age) : 0xFFFFFFFFu;
    (void)rcl_publish(&pub_tlm, &msg_tlm, NULL);
}

// skan I2C bit-bangiem (open-drain przez kierunek pinu) - wykrywa zamienione SDA/SCL
static int bb_scan(uint sda, uint scl, char *out, int outlen) {
    gpio_set_function(sda, GPIO_FUNC_SIO); gpio_set_function(scl, GPIO_FUNC_SIO);
    gpio_put(sda, 0); gpio_put(scl, 0); gpio_set_dir(sda, GPIO_IN); gpio_set_dir(scl, GPIO_IN);
    gpio_set_pulls(sda, true, false); gpio_set_pulls(scl, true, false);
    #define SDA_H gpio_set_dir(sda, GPIO_IN)
    #define SDA_L gpio_set_dir(sda, GPIO_OUT)
    #define SCL_H gpio_set_dir(scl, GPIO_IN)
    #define SCL_L gpio_set_dir(scl, GPIO_OUT)
    int n = 0, found = 0;
    for (int k = 0; k < 9; k++) { SCL_L; sleep_us(5); SCL_H; sleep_us(5); }   // bus recovery
    for (int a = 0x08; a < 0x78; a++) {
        SDA_H; SCL_H; sleep_us(5); SDA_L; sleep_us(5); SCL_L; sleep_us(5);      // START
        uint8_t b = (uint8_t)(a << 1);
        for (int i = 7; i >= 0; i--) { if (b & (1 << i)) SDA_H; else SDA_L; sleep_us(3); SCL_H; sleep_us(5); SCL_L; sleep_us(3); }
        SDA_H; sleep_us(3); SCL_H; sleep_us(5); int ack = !gpio_get(sda); SCL_L; sleep_us(3);
        SDA_L; sleep_us(5); SCL_H; sleep_us(5); SDA_H; sleep_us(10);           // STOP
        if (ack && n < outlen - 6) { n += snprintf(out + n, outlen - n, " %02X", a); found++; }
    }
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    return found;
}

// /rov/status: diagnostyka tekstowa 1 Hz (stan czujnikow, skan I2C gdy czegos brakuje)
static void cb_tmr_status(rcl_timer_t *t, int64_t last) {
    (void)t; (void)last;
    int n = snprintf(status_buf, sizeof status_buf, "up=%lus imu=%s(0x%02X n=%lu err=%lu%s) ms5837=%s(ok=%lu) ms5803=%s(ok=%lu)",
                     (unsigned long)(ms_now() / 1000), g_imu.present ? "OK" : "BRAK", g_imu.addr,
                     (unsigned long)g_imu.n_samples, (unsigned long)g_imu.n_err, g_imu.calibrating ? " CAL" : "",
                     g_press.s[0].present ? "OK" : "BRAK", (unsigned long)g_press.s[0].n_ok,
                     g_press.s[1].present ? "OK" : "BRAK", (unsigned long)g_press.s[1].n_ok);
    if (!g_imu.present || !g_press.s[0].present || !g_press.s[1].present) {
        // poziomy linii: z wewnetrznym pull-down (1 = sa zewnetrzne pull-upy) i z pull-up
        // (0 = linia trzymana nisko: zawieszony slave albo zwarcie do GND)
        int pd[2], pu[2]; const uint p2[2] = { I2C_SDA_PIN, I2C_SCL_PIN };
        for (int k = 0; k < 2; k++) {
            gpio_set_function(p2[k], GPIO_FUNC_SIO); gpio_set_dir(p2[k], GPIO_IN);
            gpio_set_pulls(p2[k], false, true); sleep_us(300); pd[k] = 0; for (int i = 0; i < 20; i++) { pd[k] += gpio_get(p2[k]); sleep_us(20); }
            gpio_set_pulls(p2[k], true, false);  sleep_us(300); pu[k] = 0; for (int i = 0; i < 20; i++) { pu[k] += gpio_get(p2[k]); sleep_us(20); }
        }
        bool sda_stuck = pu[0] < 10;
        if (sda_stuck) {   // odblokowanie I2C: 9 impulsow SCL + STOP (SDA jako wejscie)
            gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
            for (int i = 0; i < 9; i++) { gpio_put(I2C_SCL_PIN, 0); sleep_us(5); gpio_put(I2C_SCL_PIN, 1); sleep_us(5); }
            gpio_put(I2C_SDA_PIN, 0); gpio_set_dir(I2C_SDA_PIN, GPIO_OUT); sleep_us(5); gpio_set_dir(I2C_SDA_PIN, GPIO_IN); sleep_us(5);   // STOP
            gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
        }
        for (int k = 0; k < 2; k++) gpio_set_function(p2[k], GPIO_FUNC_I2C);
        if (sda_stuck) i2c_init(i2c1, 400000);
        n += snprintf(status_buf + n, sizeof status_buf - n, " | SDA pd=%d%% pu=%d%% SCL pd=%d%% pu=%d%%%s i2c1:",
                      pd[0] * 5, pu[0] * 5, pd[1] * 5, pu[1] * 5, sda_stuck ? " [SDA TRZYMANA NISKO -> recovery]" : (pd[0] < 10 ? " [SDA plywa: brak pull-upu/przewodu]" : ""));
        int found = 0;
        for (int a = 0x08; a < 0x78 && n < (int)sizeof status_buf - 8; a++) {
            uint8_t d;
            if (i2c_read_timeout_us(i2c1, (uint8_t)a, &d, 1, false, 400) >= 0) { n += snprintf(status_buf + n, sizeof status_buf - n, " 0x%02X", a); found++; }
        }
        if (!found) n += snprintf(status_buf + n, sizeof status_buf - n, " nic");
        if (!found) {   // nic po sprzetowym I2C: sprawdz bit-bangiem normalnie i z ZAMIENIONYMI liniami
            char o1[40], o2[40]; int f1 = bb_scan(I2C_SDA_PIN, I2C_SCL_PIN, o1, sizeof o1), f2 = bb_scan(I2C_SCL_PIN, I2C_SDA_PIN, o2, sizeof o2);
            i2c_init(i2c1, 400000);
            n += snprintf(status_buf + n, sizeof status_buf - n, " | bitbang SDA14/SCL15:%s | ZAMIENIONE SDA15/SCL14:%s%s",
                          f1 ? o1 : " nic", f2 ? o2 : " nic", f2 && !f1 ? " <- SDA i SCL SA ZAMIENIONE!" : "");
        }
    }
    // CUR: napiecie bez i z wewnetrznym pull-downem. Spada do ~0 z pull-downem = przewod wisi (nic nie steruje linia).
    for (int e = 0; e < 2 && n < (int)sizeof status_buf - 40; e++) {
        uint p = e ? CUR1_PIN : CUR0_PIN;
        float v0 = read_cur_mv(e);
        gpio_set_pulls(p, false, true); sleep_us(500); float v1 = read_cur_mv(e); gpio_set_pulls(p, false, false);
        n += snprintf(status_buf + n, sizeof status_buf - n, " | CUR%d=%.0fmV(pd %.0f)%s", e + 1, v0, v1, (v0 > 150 && v1 < 60) ? " WISI" : "");
    }
    rosidl_runtime_c__String__assign(&msg_status, status_buf);
    (void)rcl_publish(&pub_status, &msg_status, NULL);
}

static void msgs_init(void) {
    std_msgs__msg__String__init(&msg_status);
    sensor_msgs__msg__Imu__init(&msg_imu);
    rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, "imu_link");
    for (int i = 0; i < 9; i++) { msg_imu.orientation_covariance[i] = 0; msg_imu.angular_velocity_covariance[i] = 0; msg_imu.linear_acceleration_covariance[i] = 0; }
    msg_imu.orientation_covariance[0] = msg_imu.orientation_covariance[4] = 0.01; msg_imu.orientation_covariance[8] = 0.05;   // yaw dryfuje (brak magnetometru)
    msg_imu.angular_velocity_covariance[0] = msg_imu.angular_velocity_covariance[4] = msg_imu.angular_velocity_covariance[8] = 1e-4;
    msg_imu.linear_acceleration_covariance[0] = msg_imu.linear_acceleration_covariance[4] = msg_imu.linear_acceleration_covariance[8] = 1e-2;
    sensor_msgs__msg__Temperature__init(&msg_imu_temp);
    rosidl_runtime_c__String__assign(&msg_imu_temp.header.frame_id, "imu_link");
    static const char *fr[2] = { "ms5837", "ms5803" };
    for (int i = 0; i < 2; i++) {
        sensor_msgs__msg__FluidPressure__init(&msg_press[i]);
        sensor_msgs__msg__Temperature__init(&msg_temp[i]);
        rosidl_runtime_c__String__assign(&msg_press[i].header.frame_id, fr[i]);
        rosidl_runtime_c__String__assign(&msg_temp[i].header.frame_id, fr[i]);
    }
    std_msgs__msg__Float32__init(&msg_depth);
    rov_msgs__msg__EscTelemetry__init(&msg_tlm);
    rosidl_runtime_c__String__assign(&msg_tlm.header.frame_id, "esc");
    rov_msgs__msg__ThrusterCommand__init(&msg_thr);
    rov_msgs__msg__EscCommand__init(&msg_cmd);
    std_srvs__srv__Trigger_Request__init(&req_zero);   std_srvs__srv__Trigger_Response__init(&res_zero);
    std_srvs__srv__Trigger_Request__init(&req_imucal); std_srvs__srv__Trigger_Response__init(&res_imucal);
}

#define CHECK(x) do { if ((x) != RCL_RET_OK) return false; } while (0)

static bool create_entities(void) {
    allocator = rcl_get_default_allocator();
    CHECK(rclc_support_init(&support, 0, NULL, &allocator));
    CHECK(rclc_node_init_default(&node, "rov_pico", "", &support));

    CHECK(rclc_publisher_init_best_effort(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/rov/imu"));
    CHECK(rclc_publisher_init_best_effort(&pub_imu_temp, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "/rov/imu/temperature"));
    CHECK(rclc_publisher_init_best_effort(&pub_press[0], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure), "/rov/pressure/ms5837"));
    CHECK(rclc_publisher_init_best_effort(&pub_press[1], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure), "/rov/pressure/ms5803"));
    CHECK(rclc_publisher_init_best_effort(&pub_temp[0], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "/rov/temperature/ms5837"));
    CHECK(rclc_publisher_init_best_effort(&pub_temp[1], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "/rov/temperature/ms5803"));
    CHECK(rclc_publisher_init_best_effort(&pub_depth, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/rov/depth"));
    CHECK(rclc_publisher_init_default(&pub_tlm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rov_msgs, msg, EscTelemetry), "/rov/esc/telemetry"));
    CHECK(rclc_publisher_init_default(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rov/status"));

    CHECK(rclc_subscription_init_best_effort(&sub_thr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rov_msgs, msg, ThrusterCommand), "/rov/thrusters"));
    CHECK(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rov_msgs, msg, EscCommand), "/rov/esc/command"));

    CHECK(rclc_service_init_default(&srv_zero, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/rov/depth/zero"));
    CHECK(rclc_service_init_default(&srv_imucal, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/rov/imu/calibrate"));

    CHECK(rclc_timer_init_default(&tmr_imu, &support, RCL_MS_TO_NS(IMU_PUB_MS), cb_tmr_imu));
    CHECK(rclc_timer_init_default(&tmr_press, &support, RCL_MS_TO_NS(PRESS_PUB_MS), cb_tmr_press));
    CHECK(rclc_timer_init_default(&tmr_tlm, &support, RCL_MS_TO_NS(TLM_PUB_MS), cb_tmr_tlm));
    CHECK(rclc_timer_init_default(&tmr_status, &support, RCL_MS_TO_NS(1000), cb_tmr_status));

    executor = rclc_executor_get_zero_initialized_executor();
    CHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
    CHECK(rclc_executor_add_subscription(&executor, &sub_thr, &msg_thr, cb_thrusters, ON_NEW_DATA));
    CHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, cb_esc_command, ON_NEW_DATA));
    CHECK(rclc_executor_add_service(&executor, &srv_zero, &req_zero, &res_zero, cb_srv_zero));
    CHECK(rclc_executor_add_service(&executor, &srv_imucal, &req_imucal, &res_imucal, cb_srv_imucal));
    CHECK(rclc_executor_add_timer(&executor, &tmr_imu));
    CHECK(rclc_executor_add_timer(&executor, &tmr_press));
    CHECK(rclc_executor_add_timer(&executor, &tmr_tlm));
    CHECK(rclc_executor_add_timer(&executor, &tmr_status));

    (void)rmw_uros_sync_session(1000);
    return true;
}

static void destroy_entities(void) {
    rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    (void)rcl_publisher_fini(&pub_imu, &node);
    (void)rcl_publisher_fini(&pub_imu_temp, &node);
    for (int i = 0; i < 2; i++) { (void)rcl_publisher_fini(&pub_press[i], &node); (void)rcl_publisher_fini(&pub_temp[i], &node); }
    (void)rcl_publisher_fini(&pub_depth, &node);
    (void)rcl_publisher_fini(&pub_tlm, &node);
    (void)rcl_publisher_fini(&pub_status, &node);
    (void)rcl_subscription_fini(&sub_thr, &node);
    (void)rcl_subscription_fini(&sub_cmd, &node);
    (void)rcl_service_fini(&srv_zero, &node);
    (void)rcl_service_fini(&srv_imucal, &node);
    (void)rcl_timer_fini(&tmr_imu);
    (void)rcl_timer_fini(&tmr_press);
    (void)rcl_timer_fini(&tmr_tlm);
    (void)rcl_timer_fini(&tmr_status);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } uros_state_t;

int main(void) {
    stdio_init_all();
    // GPIO23 = PS przetwornicy Pico: 1 = tryb PWM (mniej tetnien na 3V3, cichsza cewka)
    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);
    adc_init(); adc_gpio_init(CUR0_PIN); adc_gpio_init(CUR1_PIN);
    tlm_uart_init(uart0, TLM0_RX_PIN, UART0_IRQ, uart0_irq);
    tlm_uart_init(uart1, TLM1_RX_PIN, UART1_IRQ, uart1_irq);
    multicore_launch_core1(core1_main);

    // I2C1: IMU + czujniki cisnienia. Najpierw odblokowanie magistrali (9 impulsow SCL + STOP),
    // gdyby ktorys slave zostal w srodku transakcji po resecie Pico i trzymal SDA nisko.
    gpio_init(I2C_SDA_PIN); gpio_init(I2C_SCL_PIN);
    gpio_set_dir(I2C_SDA_PIN, GPIO_IN); gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);
    gpio_put(I2C_SCL_PIN, 1); gpio_set_dir(I2C_SCL_PIN, GPIO_OUT); sleep_us(50);
    if (!gpio_get(I2C_SDA_PIN)) {
        for (int i = 0; i < 9; i++) { gpio_put(I2C_SCL_PIN, 0); sleep_us(5); gpio_put(I2C_SCL_PIN, 1); sleep_us(5); }
        gpio_put(I2C_SDA_PIN, 0); gpio_set_dir(I2C_SDA_PIN, GPIO_OUT); sleep_us(5); gpio_set_dir(I2C_SDA_PIN, GPIO_IN); sleep_us(5);
    }
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);
    gpio_init(IMU_INT_PIN); gpio_set_dir(IMU_INT_PIN, GPIO_IN); gpio_pull_down(IMU_INT_PIN);
    if (!imu_init(&g_imu, i2c1, 0x6A)) imu_init(&g_imu, i2c1, 0x6B);
    if (g_imu.present) imu_start_cal(&g_imu);
    press_init(&g_press, i2c1);
    uint32_t imu_last_us = time_us_32();

    msgs_init();
    rmw_uros_set_custom_transport(true, NULL,
        pico_usb_transport_open, pico_usb_transport_close, pico_usb_transport_write, pico_usb_transport_read);

    uros_state_t state = WAITING_AGENT;
    uint32_t last_ping = 0, last_retry_ms = 0, last_sync = 0;

    while (true) {
        uint32_t now = ms_now();

        // ---- KISS z obu UART ----
        for (int e = 0; e < 2; e++) {
            while (u_tail[e] != u_head[e]) {
                uint8_t b = u_ring[e][u_tail[e]]; u_tail[e]++;
                kiss_tlm_t k;
                if (kiss_feed(&k_par[e], b, &k)) {
                    int m = g_tlm_req_motor[e];
                    if (m >= 0) { k.n_ok = k_tlm[m].n_ok + 1; k_tlm[m] = k; }
                }
            }
        }

        // ---- IMU / cisnienie: ponawiaj inicjalizacje co 2 s gdy brak lub seria bledow ----
        if (now - last_retry_ms >= 2000) {
            if (!g_imu.present || g_imu.err_streak > 50) {
                last_retry_ms = now;
                bool was = g_imu.present;
                if (!imu_init(&g_imu, i2c1, 0x6A)) imu_init(&g_imu, i2c1, 0x6B);
                if (g_imu.present && !was) imu_start_cal(&g_imu);
            }
            if (!g_press.s[0].present || !g_press.s[1].present) {
                last_retry_ms = now;
                float p0a = g_press.s[0].p0_mbar, p0b = g_press.s[1].p0_mbar, rho = g_press.rho;
                bool a = g_press.s[0].present, b = g_press.s[1].present;
                press_init(&g_press, i2c1);
                if (a) g_press.s[0].p0_mbar = p0a;
                if (b) g_press.s[1].p0_mbar = p0b;
                g_press.rho = rho;
            }
        }
        if (g_imu.present) {
            uint32_t tu = time_us_32();
            if (gpio_get(IMU_INT_PIN) || tu - imu_last_us > 10000) {
                float dt = (float)(tu - imu_last_us) * 1e-6f; imu_last_us = tu;
                imu_read(&g_imu, i2c1, dt);
            }
        }
        press_poll(&g_press);
        // automatyczne zero glebokosci po pierwszym poprawnym odczycie (potem serwis /rov/depth/zero)
        for (int i = 0; i < 2; i++) if (g_press.s[i].present && g_press.s[i].n_ok > 5 && g_press.s[i].p0_mbar == 0) g_press.s[i].p0_mbar = g_press.s[i].p_mbar;

        // ---- micro-ROS: maszyna stanow polaczenia z agentem ----
        switch (state) {
        case WAITING_AGENT:
            if (stdio_usb_connected() && now - last_ping >= 500) {
                last_ping = now;
                if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
            }
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) { state = AGENT_CONNECTED; g_agent_connected = 1; last_sync = now; }
            else { destroy_entities(); state = WAITING_AGENT; }
            break;
        case AGENT_CONNECTED:
            if (now - last_ping >= PING_MS) {
                last_ping = now;
                if (!stdio_usb_connected() || rmw_uros_ping_agent(50, 2) != RMW_RET_OK) { state = AGENT_DISCONNECTED; break; }
            }
            if (now - last_sync >= 60000) { last_sync = now; (void)rmw_uros_sync_session(200); }
            (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            break;
        case AGENT_DISCONNECTED:
            g_agent_connected = 0;
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }

        // ---- failsafe ---- (czas liczony PO spin_some: komenda mogla przyjsc w tej petli, roznica ze znakiem)
        int32_t cmd_age = (int32_t)(ms_now() - g_last_cmd_ms);
        bool fs = (state != AGENT_CONNECTED) || !g_last_cmd_ms || cmd_age > (int32_t)CMD_TIMEOUT_MS;
        if (fs && any_running()) stop_all();
        g_failsafe = fs;

        if (state != AGENT_CONNECTED) sleep_ms(1);
    }
}
