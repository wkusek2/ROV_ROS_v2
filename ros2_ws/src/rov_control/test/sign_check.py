#!/usr/bin/env python3
"""Weryfikacja znakow i zachowania stabilizacji ROV - PRAWDZIWY kod z repo (allocation, ThrusterMixer.allocate_cmd,
filtr + PID AttitudeController) spiety z NIEZALEZNA fizyka bryly sztywnej. Wynik: wykresy PNG.

  source ros2_ws/install/setup.bash
  python3 ros2_ws/src/rov_control/test/sign_check.py docs/stabilizacja/po

Fizyka nie uzywa konwencji znakow z repo. Jedyne zalozenia:
  - pojazd FRD (x przod, y prawo, z dol), swiat NED (z w dol); moment tau = r x F, I dw/dt = tau - w x Iw - D w
  - "prawa burta w dol" / "dziob w gore" / "dziob w lewo" sprawdzane GEOMETRYCZNIE (polozenie punktow kadluba)
  - IMU zamontowane zgodnie z imu_axes z config/attitude.yaml ("x,z,-y"): to jedyne zalozenie o sprzecie
"Stary kod" (przed poprawka 2026-11) jest emulowany odwroceniem K i M w mikserze - dla petli rownowazne
staremu regulatorowi (roll i czlon D z odwroconym znakiem) + staremu mikserowi (M = -pitch). Emulacja daje te same
liczby co uruchomienie starego kodu (docs/stabilizacja/przed/).
Exit code 1, gdy obecny kod nie przechodzi testow znakow (do uzycia w CI / przed wgraniem).
"""
import math
import os
import sys
import types

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from rov_control import allocation as al  # noqa: E402
import rov_control.attitude_controller as acm  # noqa: E402
import rov_control.thruster_mixer as tmm  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

G = 9.81
IMU_AXES = "x,z,-y"
GAINS = {"yaw": [0.02, 0.002, 0.004], "roll": [0.02, 0.002, 0.004], "pitch": [0.02, 0.002, 0.004], "depth": [1.5, 0.2, 0.5]}
LEGACY = {"pitch": ("M", -1), "roll": ("K", -1)}


# ---------------- prawdziwy mikser (bez ROS) ----------------
def make_mixer():
    m = tmm.ThrusterMixer.__new__(tmm.ThrusterMixer)
    m.B = al.build_B(); m.Bp = al.pinv_weighted(m.B, None)
    m.f_fwd, m.f_rev, m.expo = 40.0, 30.0, 1.0
    m.tau_max = al.dof_limits(m.Bp, m.B, m.f_fwd, m.f_rev)
    return m


def mixer_forces(mixer, cmd6, gain, dof_map=None):
    """cmd6 (semantyka GUI) -> sily 8 pednikow [N] (kolejnosc al.NAMES) przez PRAWDZIWE ThrusterMixer.allocate_cmd."""
    saved = dict(tmm.DOF_OF_AXIS)
    if dof_map:
        tmm.DOF_OF_AXIS.update(dof_map)
    try:
        _, f, _ = tmm.ThrusterMixer.allocate_cmd(mixer, list(cmd6), gain)
    finally:
        tmm.DOF_OF_AXIS.clear(); tmm.DOF_OF_AXIS.update(saved)
    return np.asarray(f)


# ---------------- prawdziwy regulator (bez ROS, zegar symulacji) ----------------
SIMT = [0.0]
acm.time = types.SimpleNamespace(monotonic=lambda: SIMT[0], time=lambda: SIMT[0])


class Pub:
    last = None

    def publish(self, m):
        self.last = m


def make_ctrl(level=True, yaw_hold=True):
    c = acm.AttitudeController.__new__(acm.AttitudeController)
    c.R = acm.axis_map(IMU_AXES); c.acc_w = 0.02; c.deadband = 0.08; c.pilot_timeout = 0.5
    c.pid = {k: acm.PID(*g, out_lim=0.6, i_lim=0.3) for k, g in GAINS.items()}
    c.q = np.array([1.0, 0, 0, 0]); c.q_ref = np.array([1.0, 0, 0, 0]); c.yaw_ref = 0.0
    c.gyro_b = np.zeros(3); c.acc_b = np.zeros(3); c.last_imu = None; c.imu_t = 0.0; c.n_imu = 0
    c.rpy = [0.0, 0.0, 0.0]; c.depth = None; c.depth_t = 0.0
    c.mode = {"yaw_hold": yaw_hold, "level": level, "depth_hold": False}
    c.sp = {"yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0, "depth_m": 0.0}
    c.pilot = [0.0] * 6; c.pilot_t = 0.0
    c.corr = {"yaw": 0.0, "roll": 0.0, "pitch": 0.0, "heave": 0.0}
    c.suspended = {"yaw": False, "level": False, "depth": False}
    c.pub_att, c.pub_cmd, c.pub_dbg, c.pub_st = Pub(), Pub(), Pub(), Pub()
    return c


def ctrl_cmd(c):
    tw = c.pub_cmd.last
    return [tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z] if tw else [0.0] * 6


# ---------------- niezalezna fizyka ----------------
FLU = np.diag([1.0, -1.0, -1.0])


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def expm_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = skew(w / th)
    return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k


def geo_angles(R):
    """[deg] z polozenia punktow (R: cialo FRD -> swiat NED): roll>0 prawa burta nizej, pitch>0 dziob wyzej, yaw>0 dziob w lewo."""
    right, nose = R @ np.array([0, 1.0, 0]), R @ np.array([1.0, 0, 0])
    return (math.degrees(math.asin(np.clip(right[2], -1, 1))), math.degrees(math.asin(np.clip(-nose[2], -1, 1))),
            -math.degrees(math.atan2(nose[1], nose[0])))


def imu_msg(R, w_frd):
    Rmap = acm.axis_map(IMU_AXES)
    f_frd = R.T @ np.array([0, 0, -G])
    gs = Rmap.T @ (FLU @ w_frd); as_ = Rmap.T @ (FLU @ f_frd)
    m = Imu()
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gs)
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, as_)
    return m


def simulate(gain, legacy=False, T=12.0, dist=(0, 0, 0), heave=None, mismatch=None, bg=0.02, level=True, yaw_hold=True):
    """Pojazd w wodzie z regulatorem (poziom + kurs), pilot 0. dist: impuls momentu [Nm] FRD przez 0.5 s od t=1 s.
    heave: (t0, t1, wartosc) komenda pilota heave. mismatch[8]: wspolczynnik ciagu pednikow (podrobki). bg: CB nad CG [m]."""
    mixer = make_mixer(); c = make_ctrl(level, yaw_hold)
    dof_map = LEGACY if legacy else None
    k_thr = np.ones(8) if mismatch is None else np.asarray(mismatch)
    I = np.diag([0.9, 1.5, 1.8]); D = np.diag([3.0, 4.0, 4.0]); W = 250.0
    R = np.eye(3); w = np.zeros(3); f_act = np.zeros(8); f_cmd = np.zeros(8)
    dt = 0.005; t = 0.0; next_imu = next_ctrl = 0.0
    log = {k: [] for k in ("t", "roll", "pitch", "yaw")}
    while t < T:
        SIMT[0] = t
        if t >= next_imu:
            acm.AttitudeController.on_imu(c, imu_msg(R, w)); next_imu += 0.02
        if t >= next_ctrl:
            hv = heave[2] if heave and heave[0] <= t < heave[1] else 0.0
            c.pilot = [0.0, 0.0, hv, 0.0, 0.0, 0.0]; c.pilot_t = t
            acm.AttitudeController.tick(c)
            f_cmd = mixer_forces(mixer, ctrl_cmd(c), gain, dof_map); next_ctrl += 0.05
        f_act += (f_cmd - f_act) * dt / 0.12
        tau = (mixer.B @ (f_act * k_thr))[3:]
        tau = tau + np.cross(np.array([0, 0, -bg]), R.T @ np.array([0, 0, -W])) - D @ w
        if 1.0 <= t < 1.5:
            tau = tau + np.asarray(dist, float)
        w = w + np.linalg.solve(I, tau - np.cross(w, I @ w)) * dt
        R = R @ expm_so3(w * dt)
        for k, v in zip(log, (t,) + geo_angles(R)):
            log[k].append(v)
        if max(abs(log["roll"][-1]), abs(log["pitch"][-1])) > 85:
            break
        t += dt
    return {k: np.array(v) for k, v in log.items()}


# ---------------- testy statyczne (znaki) ----------------
def static_case(mixer, kind):
    """-> (sily f, tekst, ok) dla: pilot pitch+/roll+/yaw+ oraz regulator przy przechyle prawa burta w dol / dziob w gore."""
    B = mixer.B
    if kind in ("pilot_pitch", "pilot_roll", "pilot_yaw"):
        idx = {"pilot_roll": 3, "pilot_pitch": 4, "pilot_yaw": 5}[kind]
        cmd = [0.0] * 6; cmd[idx] = 0.5
        f = mixer_forces(mixer, cmd, 1.0); tau = B @ f
        R = expm_so3(tau[3:] * 1e-3)
        ro, pi, ya = geo_angles(R)
        res = {"pilot_roll": ("prawa burta idzie", ro > 0, "W DOL", "W GORE", f"K = {tau[3]:+.1f} Nm"),
               "pilot_pitch": ("dziob idzie", pi > 0, "W GORE", "W DOL", f"M = {tau[4]:+.1f} Nm"),
               "pilot_yaw": ("dziob idzie", ya > 0, "W LEWO", "W PRAWO", f"N = {tau[5]:+.1f} Nm")}[kind]
        txt = f"moment {res[4]}  ->  {res[0]} {res[2] if res[1] else res[3]}"
        return f, txt, res[1]
    axis = {"tilt_roll": np.array([math.radians(10), 0, 0]), "tilt_pitch": np.array([0, math.radians(10), 0])}[kind]
    R10 = expm_so3(axis)
    c = make_ctrl(level=True, yaw_hold=False)
    for k in range(400):
        SIMT[0] = k * 0.02; acm.AttitudeController.on_imu(c, imu_msg(R10, np.zeros(3)))
    c.pilot_t = SIMT[0]; acm.AttitudeController.tick(c)
    f = mixer_forces(mixer, ctrl_cmd(c), 1.0); tau = B @ f
    R = R10 @ expm_so3(tau[3:] * 1e-3)
    ro, pi, _ = geo_angles(R)
    if kind == "tilt_roll":
        ok = ro < 10.0 - 1e-6
        txt = (f"regulator odczytal roll = {c.rpy[0]:+.1f} st. (+ = prawa w dol)\nK = {tau[3]:+.2f} Nm -> prawa burta "
               f"{'wraca W GORE (dobrze)' if ok else 'DALEJ W DOL (!)'}")
    else:
        ok = pi < 10.0 - 1e-6
        txt = (f"regulator odczytal pitch = {c.rpy[1]:+.1f} st. (+ = dziob w gore)\nM = {tau[4]:+.2f} Nm -> dziob "
               f"{'wraca W DOL (dobrze)' if ok else 'DALEJ W GORE (!)'}")
    return f, txt, ok


def draw_view(ax, mixer, f, plane, title, note, ok, scale=0.012):
    P = np.array([al.THRUSTERS[n][:3] for n in al.NAMES]) / 1000.0
    i, j = {"side": (0, 2), "front": (1, 2)}[plane]
    ax.set_title(title, fontsize=10)
    hw = 0.33 if plane == "side" else 0.2
    ax.add_patch(plt.Polygon([(-hw, -0.08), (hw, -0.08), (hw, 0.08), (-hw, 0.08)], closed=True, color="#ddd"))
    vec = (mixer.B[:3] * f).T
    for k in range(8):
        ax.plot(P[k, i], P[k, j], "ko", ms=4)
        ax.annotate("", xy=(P[k, i] + vec[k, i] * scale, P[k, j] + vec[k, j] * scale), xytext=(P[k, i], P[k, j]),
                    arrowprops=dict(arrowstyle="->", color="tab:red" if vec[k, j] > 0 else "tab:blue", lw=2))
    ax.set_xlim(-0.6, 0.6); ax.set_ylim(0.45, -0.45); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]   RUFA <- -> DZIOB" if plane == "side" else "y [m]   LEWA <- -> PRAWA burta (widok od rufy)")
    if plane == "side":
        ax.text(0.36, 0.0, "DZIOB", fontsize=8)
    ax.set_ylabel("gora / dol")
    ax.text(0.02, 0.02, note + "\ncz. = pcha w dol, nieb. = w gore", transform=ax.transAxes, fontsize=9, va="bottom",
            bbox=dict(boxstyle="round", fc="#d8f5d0" if ok else "#ffd6d6"))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    mixer = make_mixer()

    # ---- 1: statyka (obecny kod) ----
    fig, axs = plt.subplots(2, 3, figsize=(20, 10))
    checks = []
    for ax, (kind, plane, title, sc) in zip(axs.flat, [
            ("pilot_pitch", "side", "Pilot: pitch +0.5 (GUI: dziob w gore)", 0.012),
            ("pilot_roll", "front", "Pilot: roll +0.5 (GUI: prawa burta w dol)", 0.012),
            ("pilot_yaw", "front", "Pilot: yaw +0.5 (GUI: w lewo) - widok od rufy, sily poziome", 0.012),
            ("tilt_roll", "front", "Pojazd: PRAWA burta w dol 10 st., regulator 'poziom'", 0.06),
            ("tilt_pitch", "side", "Pojazd: DZIOB w gore 10 st., regulator 'poziom'", 0.06)]):
        f, txt, ok = static_case(mixer, kind)
        checks.append((kind, ok, txt.replace("\n", " | ")))
        draw_view(ax, mixer, f, plane, title, txt, ok, sc)
    axs.flat[-1].axis("off")
    axs.flat[-1].text(0.0, 0.5, "\n".join(f"{'OK ' if ok else 'ZLE'}  {k}" for k, ok, _ in checks), fontsize=14, family="monospace",
                      va="center")
    fig.suptitle("Kierunki dzialania pednikow - OBECNY kod (zielone = dobrze, czerwone = zle), sprawdzone geometrycznie", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(out, "1_pedniki.png"), dpi=85); plt.close(fig)

    # ---- 2 i 3: zamknieta petla, obecny vs stary ----
    rng = np.random.default_rng(7)
    weak = rng.uniform(0.8, 1.0, 8)          # podrobki T200: 80..100% nominalnego ciagu, kazda inna
    cases = [("roll", dict(dist=(6.0, 0, 0)), "Impuls ROLL 6 Nm / 0.5 s", ["roll"]),
             ("pitch", dict(dist=(0, 8.0, 0)), "Impuls PITCH 8 Nm / 0.5 s", ["pitch"]),
             ("yaw", dict(dist=(0, 0, 6.0)), "Impuls YAW 6 Nm / 0.5 s", ["yaw"]),
             ("zanurzanie", dict(heave=(1.0, 8.0, -0.8), mismatch=weak, T=12.0),
              "Zanurzanie heave -0.8 przez 7 s, pedniki 80-100% ciagu", ["roll", "pitch", "yaw"])]
    summary = []
    fig, axs = plt.subplots(len(cases), 2, figsize=(16, 4 * len(cases)), sharex=True)
    for row, (name, kw, title, keys) in enumerate(cases):
        for col, legacy in enumerate((True, False)):
            ax = axs[row, col]
            for gain, ls in ((0.3, "-"), (1.0, ":")):
                lg = simulate(gain, legacy=legacy, **kw)
                for key, colr in zip(keys, ("tab:blue", "tab:orange", "tab:green")):
                    ax.plot(lg["t"], lg[key], ls, color=colr if len(keys) > 1 else ("tab:blue" if gain == 0.3 else "tab:red"), lw=2,
                            label=f"{key} gain {gain}")
                    summary.append((name, "STARY" if legacy else "OBECNY", gain, key,
                                    float(np.max(np.abs(lg[key]))), float(lg[key][-1]), float(lg["t"][-1])))
            lg0 = simulate(0.0, legacy=legacy, level=False, yaw_hold=False, **kw)
            for key in keys:
                ax.plot(lg0["t"], lg0[key], color="gray", lw=1, label=f"{key} bez regulatora" if key == keys[0] else None)
            ax.axvspan(*((1.0, 1.5) if "dist" in kw else kw["heave"][:2]), color="orange", alpha=0.15)
            ax.set_title(f"{'STARY kod (przed poprawka)' if legacy else 'OBECNY kod (po poprawce)'} - {title}", fontsize=10)
            ax.set_ylabel("kat [deg] (geometrycznie)"); ax.set_ylim(-90, 90); ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=2)
    axs[-1, 0].set_xlabel("t [s]"); axs[-1, 1].set_xlabel("t [s]")
    fig.suptitle("Porownanie: stary vs obecny kod. Prawdziwy filtr+PID+mikser, niezalezna fizyka (wypor 2 cm nad CG, pedniki 120 ms)", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(out, "2_porownanie.png"), dpi=85); plt.close(fig)

    with open(os.path.join(out, "wyniki.txt"), "w") as fh:
        fh.write("TESTY ZNAKOW (obecny kod)\n")
        for k, ok, txt in checks:
            fh.write(f"  {'OK ' if ok else 'ZLE'} {k:12s} {txt}\n")
        fh.write("\nZAMKNIETA PETLA (max |kat|, kat koncowy, koniec symulacji; koniec < T = wywrotka > 85 st.)\n")
        for s in summary:
            fh.write("  %-11s %-6s gain %.1f %-6s max %6.1f  koniec %6.1f  t_end %5.1f s\n" % s)
    print(open(os.path.join(out, "wyniki.txt")).read())
    sys.exit(0 if all(ok for _, ok, _ in checks) else 1)


if __name__ == "__main__":
    main()
