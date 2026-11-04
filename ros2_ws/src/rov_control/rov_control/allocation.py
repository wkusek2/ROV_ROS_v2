"""Kinematyka / alokacja pednikow ROV (8 pednikow), numpy.

Uklad pojazdu FRD (prawoskretny): x do przodu, y w prawo, z w dol, poczatek w srodku geometrycznym.
tau = [X, Y, Z, K, M, N] = sily [N] wzdluz x, y, z i momenty [Nm] roll, pitch, yaw (Fossen).
Kat psi: kierunek, w ktorym pednik pcha POJAZD przy dodatnim sygnale, w plaszczyznie x-y, od +x, dodatni ku +y.
Kat theta: pochylenie ciagu wzgledem x-y, dodatni gdy pcha ku +z (w dol).
d = (cos th cos psi, cos th sin psi, sin th);  B[:, i] = [d_i ; r_i x d_i],  r_i = p_i - r_CG;  tau = B f.
"""
import math

import numpy as np

BETA_DEG = 22.4
# nazwa: (x, y, z [mm], psi [deg], theta [deg]).  P/T = przod/tyl, G/D = gorny/dolny, L/P = lewy/prawy
THRUSTERS = {
    "PGL": (293.317, -170.429, -52.304, 22.4, +BETA_DEG),
    "PGP": (293.317, 170.429, -52.304, -22.4, +BETA_DEG),
    "TGL": (-293.317, -170.429, -52.304, -22.4, -BETA_DEG),
    "TGP": (-293.317, 170.429, -52.304, 22.4, -BETA_DEG),
    "PDL": (293.317, -170.429, 52.304, 22.4, -BETA_DEG),
    "PDP": (293.317, 170.429, 52.304, -22.4, -BETA_DEG),
    "TDL": (-293.317, -170.429, 52.304, -22.4, +BETA_DEG),
    "TDP": (-293.317, 170.429, 52.304, 22.4, +BETA_DEG),
}
NAMES = list(THRUSTERS.keys())
# odpowiedniki slotow z GUI (przod/tyl, lewy/prawy, gora/dol)
SLOT_OF = {"PGL": "FLU", "PGP": "FRU", "TGL": "RLU", "TGP": "RRU", "PDL": "FLD", "PDP": "FRD", "TDL": "RLD", "TDP": "RRD"}
NAME_OF_SLOT = {v: k for k, v in SLOT_OF.items()}
DOF = ["X", "Y", "Z", "K", "M", "N"]


def build_B(thrusters=THRUSTERS, r_cg_mm=(0.0, 0.0, 0.0), beta_deg=None):
    """Macierz alokacji 6x8 (pozycje w mm -> m). beta_deg nadpisuje |theta| dla wszystkich (zachowuje znaki)."""
    B = np.zeros((6, len(thrusters)))
    r_cg = np.asarray(r_cg_mm, float) / 1000.0
    for i, (name, (x, y, z, psi, th)) in enumerate(thrusters.items()):
        if beta_deg is not None:
            th = math.copysign(beta_deg, th)
        ps, t = math.radians(psi), math.radians(th)
        d = np.array([math.cos(t) * math.cos(ps), math.cos(t) * math.sin(ps), math.sin(t)])
        r = np.array([x, y, z]) / 1000.0 - r_cg
        B[:3, i] = d
        B[3:, i] = np.cross(r, d)
    return B


def analyze(B):
    """Rzad, wartosci osobliwe, liczba uwarunkowania, sterowalnosc 6 DOF."""
    sv = np.linalg.svd(B, compute_uv=False)
    rank = int(np.sum(sv > 1e-9 * sv[0]))
    return {"rank": rank, "singular_values": [round(float(s), 4) for s in sv],
            "cond": round(float(sv[0] / sv[-1]), 2) if sv[-1] > 1e-12 else float("inf"), "controllable": rank == 6}


def pinv_weighted(B, W=None):
    """Pseudoodwrotnosc (Moore-Penrose) lub wazona: B+ = W^-1 B^T (B W^-1 B^T)^-1 (W = macierz wag 8x8, wieksza waga = mniej uzywany pednik)."""
    if W is None:
        return np.linalg.pinv(B)
    Wi = np.linalg.inv(np.asarray(W, float))
    return Wi @ B.T @ np.linalg.inv(B @ Wi @ B.T)


def allocate(Bp, tau):
    """f = B+ tau  [N]"""
    return Bp @ np.asarray(tau, float)


def saturate(f, f_max_fwd, f_max_rev):
    """Jesli ktoras sila przekracza limit, skaluj CALY wektor (kierunek tau zachowany). Zwraca (f, skala, ostrzezenie)."""
    f = np.asarray(f, float)
    ratios = np.where(f >= 0, f / f_max_fwd, -f / f_max_rev)
    peak = float(np.max(ratios)) if f.size else 0.0
    if peak > 1.0:
        return f / peak, 1.0 / peak, f"saturacja: przeskalowano x{1.0 / peak:.3f}"
    return f, 1.0, ""


def force_to_cmd(f, f_max_fwd, f_max_rev, exponent=1.0, table_fwd=None, table_rev=None):
    """Sila [N] -> sygnal -1..1 (per pednik). Domyslnie f = Fmax * cmd^exponent (exponent 1 = liniowo; T200 ~1.5-1.8).
    table_fwd/table_rev: opcjonalnie lista (cmd, sila_N) rosnaca - wtedy interpolacja z tabeli producenta."""
    f = np.asarray(f, float)
    out = np.zeros_like(f)
    for i, v in enumerate(f):
        if v >= 0:
            out[i] = np.interp(v, [p[1] for p in table_fwd], [p[0] for p in table_fwd]) if table_fwd else min(v / f_max_fwd, 1.0) ** (1.0 / exponent)
        else:
            out[i] = -(np.interp(-v, [p[1] for p in table_rev], [p[0] for p in table_rev]) if table_rev else min(-v / f_max_rev, 1.0) ** (1.0 / exponent))
    return out


def dof_limits(Bp, B, f_max_fwd, f_max_rev):
    """Maksymalna osiagalna sila/moment w kazdym DOF osobno (skalowanie pseudoodwrotnosci do limitow pednikow)."""
    out = {}
    for k, name in enumerate(DOF):
        e = np.zeros(6); e[k] = 1.0
        f = allocate(Bp, e)
        ratios = np.where(f >= 0, f / f_max_fwd, -f / f_max_rev)
        peak = float(np.max(ratios))
        out[name] = round(1.0 / peak, 2) if peak > 0 else 0.0
    return out


def verify_pure_moves(Bp, B):
    """Dla kazdego czystego DOF: f, blad odtworzenia |B f - e| i max sprzezenie w innych DOF."""
    res = {}
    for k, name in enumerate(DOF):
        e = np.zeros(6); e[k] = 1.0
        f = allocate(Bp, e)
        tau = B @ f
        err = float(np.linalg.norm(tau - e))
        coupling = float(np.max(np.abs(np.delete(tau, k))))
        res[name] = {"f": [round(float(v), 4) for v in f], "err": round(err, 6), "coupling": round(coupling, 6)}
    return res


def describe(B):
    """Krotki opis slowny: ktory pednik wplywa na ktory DOF (na podstawie znakow w B)."""
    lines = []
    for k, dof in enumerate(DOF):
        pos = [NAMES[i] for i in range(B.shape[1]) if B[k, i] > 1e-6]
        neg = [NAMES[i] for i in range(B.shape[1]) if B[k, i] < -1e-6]
        lines.append(f"{dof}: + {' '.join(pos) or '-'} | - {' '.join(neg) or '-'}")
    return lines


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=140)
    B = build_B()
    print("B (wiersze X Y Z K M N; kolumny", NAMES, ")\n", B)
    print(analyze(B))
    Bp = pinv_weighted(B)
    print("limity DOF:", dof_limits(Bp, B, 40.0, 30.0))
    for dof, r in verify_pure_moves(Bp, B).items():
        print(dof, r)
    print("\n".join(describe(B)))
