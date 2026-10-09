"""Inductance of PCB excitation rings, modelled as coaxial circular filaments.

Each turn of a spiral is approximated by a circle at its mean radius. The
self term uses the round-wire formula with the geometric mean distance of a
rectangular track cross-section; mutual terms use the exact elliptic-integral
expression for coaxial circular loops.
"""
import math

from scipy.special import ellipe, ellipk

MU0 = 4e-7 * math.pi


def self_l(r, w, t=0.035e-3):
    """Self inductance of one circular track turn, radius r, width w (SI units)."""
    gmd = 0.2235 * (w + t)
    return MU0 * r * (math.log(8 * r / gmd) - 2.0)


def mutual(a, b, d):
    """Mutual inductance of two coaxial circles of radius a, b separated axially by d."""
    m = 4 * a * b / ((a + b) ** 2 + d ** 2)  # k^2
    k = math.sqrt(m)
    return MU0 * math.sqrt(a * b) * ((2 / k - k) * ellipk(m) - (2 / k) * ellipe(m))


def coil_l(turns, w):
    """turns: list of (radius_m, z_m, sign). Returns total inductance in H."""
    total = 0.0
    for i, (ri, zi, si) in enumerate(turns):
        for j, (rj, zj, sj) in enumerate(turns):
            if i == j:
                total += self_l(ri, w)
            else:
                total += si * sj * mutual(ri, rj, abs(zi - zj))
    return total


def ring(r_start, n, pitch, z_layers, sign, inward=True):
    """n turns per layer, on every layer in z_layers, starting at r_start (mm)."""
    out = []
    for z in z_layers:
        for k in range(n):
            r = r_start - k * pitch if inward else r_start + k * pitch
            out.append((r * 1e-3, z * 1e-3, sign))
    return out


if __name__ == "__main__":
    W, P = 0.2e-3, 0.4
    L12 = (0.0, 0.2)  # L1 and L2 heights in a 1.6 mm 4-layer stack (mm)
    print("Outer track (outer ring CCW @ 28.6 inward, inner ring CW @ 18.8 outward)")
    for n_o in range(2, 7):
        for n_i in range(2, 6):
            t = ring(28.6, n_o, P, L12, +1) + ring(18.8, n_i, P, L12, -1, inward=False)
            print(f"  n_out={n_o} n_in={n_i}: L = {coil_l(t, W)*1e6:5.2f} uH")
    print("Inner track (outer ring CCW @ 16.9 inward, inner ring CW @ 9.0 outward)")
    for n_o in range(2, 7):
        for n_i in range(2, 6):
            t = ring(16.9, n_o, P, L12, +1) + ring(9.0, n_i, P, L12, -1, inward=False)
            print(f"  n_out={n_o} n_in={n_i}: L = {coil_l(t, W)*1e6:5.2f} uH")
