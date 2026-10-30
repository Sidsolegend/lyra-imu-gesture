#!/usr/bin/env python3
"""
verify_stl_shells.py  --  read-only STL shell / Z-extent inspector for LYRA.

Parses an STL (auto-detects ASCII vs binary), welds coincident vertices,
groups triangles into connected shells via union-find, and reports per shell:
triangle count, z_min, z_max.

Dependencies: numpy only (numpy-stl / trimesh / admesh are NOT required).

Usage:
    python3 verify_stl_shells.py file1.stl [file2.stl ...]
"""

import sys
import struct
import numpy as np

WELD_TOL = 1e-4  # mm; vertices closer than this are treated as the same point


def is_binary_stl(path):
    """Detect binary vs ASCII STL.

    ASCII STLs begin with the token 'solid'. But some binary STLs also start
    with 'solid' in their 80-byte header, so we corroborate: if the file starts
    with 'solid' AND contains 'facet' as a text token early on, treat as ASCII.
    Otherwise fall back to the 84 + 50*N byte-size check for binary.
    """
    with open(path, "rb") as f:
        head = f.read(512)
    stripped = head.lstrip()
    if stripped[:5].lower() == b"solid":
        # Could still be binary with a misleading header; confirm ASCII markers.
        if b"facet" in head or b"vertex" in head:
            return False  # ASCII
    # Corroborate with the binary size formula.
    import os
    size = os.path.getsize(path)
    if size >= 84:
        with open(path, "rb") as f:
            f.seek(80)
            n = struct.unpack("<I", f.read(4))[0]
        if size == 84 + n * 50:
            return True  # binary
    # Default: if it began with 'solid', assume ASCII.
    return stripped[:5].lower() != b"solid"


def parse_ascii_stl(path):
    """Return an (M, 3, 3) array of triangle vertices from an ASCII STL."""
    tris = []
    cur = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if s.startswith("vertex"):
                parts = s.split()
                cur.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(cur) == 3:
                    tris.append(cur)
                    cur = []
    if not tris:
        raise ValueError(f"No triangles parsed from ASCII STL: {path}")
    return np.asarray(tris, dtype=np.float64)


def parse_binary_stl(path):
    """Return an (M, 3, 3) array of triangle vertices from a binary STL."""
    with open(path, "rb") as f:
        f.read(80)  # header
        n = struct.unpack("<I", f.read(4))[0]
        data = f.read(n * 50)
    tris = np.empty((n, 3, 3), dtype=np.float64)
    for i in range(n):
        off = i * 50
        # 12 floats = normal(3) + v0(3) + v1(3) + v2(3); skip normal.
        vals = struct.unpack_from("<12f", data, off)
        tris[i, 0] = vals[3:6]
        tris[i, 1] = vals[6:9]
        tris[i, 2] = vals[9:12]
    if n == 0:
        raise ValueError(f"No triangles parsed from binary STL: {path}")
    return tris


def load_triangles(path):
    if is_binary_stl(path):
        return parse_binary_stl(path), "binary"
    return parse_ascii_stl(path), "ascii"


def weld_vertices(tris, tol=WELD_TOL):
    """Map each triangle corner to a welded vertex id (rounded-grid quantization).

    Returns (tri_vert_ids: (M,3) int array, n_unique).
    """
    verts = tris.reshape(-1, 3)
    # Quantize to a grid of size `tol`; vertices within ~tol collapse together.
    keys = np.round(verts / tol).astype(np.int64)
    # Map each unique quantized key to a compact id.
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    return inv.reshape(-1, 3), int(inv.max()) + 1


class UnionFind:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, x):
        root = x
        while self.p[root] != root:
            root = self.p[root]
        while self.p[x] != root:
            self.p[x], x = root, self.p[x]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def connected_shells(tri_vert_ids):
    """Union-find triangles that share any welded vertex into shells.

    Returns a list of shells, each a list of triangle indices.
    """
    m = tri_vert_ids.shape[0]
    uf = UnionFind(m)
    # Bucket triangles by vertex id; union all triangles sharing a vertex.
    from collections import defaultdict
    vert_to_tris = defaultdict(list)
    for ti in range(m):
        for vid in tri_vert_ids[ti]:
            vert_to_tris[vid].append(ti)
    for tri_list in vert_to_tris.values():
        first = tri_list[0]
        for other in tri_list[1:]:
            uf.union(first, other)
    shells = defaultdict(list)
    for ti in range(m):
        shells[uf.find(ti)].append(ti)
    return list(shells.values())


def analyze(path):
    tris, fmt = load_triangles(path)
    tri_vert_ids, n_unique = weld_vertices(tris)
    shells = connected_shells(tri_vert_ids)
    # Sort shells by z_min for stable, readable output.
    reports = []
    for tidx in shells:
        pts = tris[tidx].reshape(-1, 3)
        reports.append({
            "tris": len(tidx),
            "z_min": float(pts[:, 2].min()),
            "z_max": float(pts[:, 2].max()),
        })
    reports.sort(key=lambda r: r["z_min"])
    return fmt, len(tris), n_unique, reports


def main(argv):
    if len(argv) < 2:
        print("usage: python3 verify_stl_shells.py file1.stl [file2.stl ...]")
        return 1
    for path in argv[1:]:
        try:
            fmt, ntri, nvert, reports = analyze(path)
        except Exception as e:  # noqa: BLE001
            print(f"\n{path}\n  ERROR: {e}")
            continue
        print(f"\n{path}")
        print(f"  format={fmt}  triangles={ntri}  welded_vertices={nvert}  shells={len(reports)}")
        print(f"  {'shell':>5}  {'tris':>8}  {'z_min':>10}  {'z_max':>10}  {'z_span':>10}")
        for i, r in enumerate(reports):
            span = r["z_max"] - r["z_min"]
            print(f"  {i:>5}  {r['tris']:>8}  {r['z_min']:>10.4f}  {r['z_max']:>10.4f}  {span:>10.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
