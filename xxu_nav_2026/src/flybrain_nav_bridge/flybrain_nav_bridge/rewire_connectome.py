"""Create an isolated degree-preserving MaleCNS connectivity null control.

Run this script with the MaleCNS virtual environment.  It never mutates source
assets and refuses to overwrite an existing destination directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import shutil
import sys
import tempfile
from typing import Iterable

import numpy as np
from scipy import sparse


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def csr_has_edge(matrix: sparse.csr_matrix, row: int, col: int) -> bool:
    """Fast enough sparse lookup without materializing a 25M-edge hash set."""
    start, end = matrix.indptr[row], matrix.indptr[row + 1]
    index = np.searchsorted(matrix.indices[start:end], col)
    return index < end - start and matrix.indices[start + index] == col


def degree_preserving_rewire(
    matrix: sparse.spmatrix,
    *,
    seed: int,
    target_swaps: int,
    max_attempts: int | None = None,
) -> tuple[sparse.csr_matrix, dict[str, int]]:
    """Use directed double-edge swaps while preserving row/column degrees.

    For two existing edges ``post_a <- pre_i`` and ``post_b <- pre_j``, a
    successful swap becomes ``post_a <- pre_j`` and ``post_b <- pre_i``.  New
    self loops and duplicate edges are rejected.  Values stay attached to their
    selected edge slots, preserving the complete weight-value multiset.
    """
    original = matrix.tocsr(copy=True)
    original.sum_duplicates()
    original.sort_indices()
    if original.nnz < 2:
        raise ValueError("at least two edges are required for a degree-preserving swap")
    if target_swaps <= 0:
        raise ValueError("target_swaps must be positive")

    coo = original.tocoo(copy=True)
    rows = coo.row.astype(np.int64, copy=False)
    cols = coo.col.astype(np.int64, copy=False)
    rng = random.Random(seed)
    maximum_attempts = max_attempts if max_attempts is not None else target_swaps * 50
    # Track only the sparse delta, rather than a Python set of every original
    # edge. This bounds memory while still preventing duplicate edges across
    # consecutive swaps.
    added: set[tuple[int, int]] = set()
    removed: set[tuple[int, int]] = set()

    def current_edge_exists(row: int, col: int) -> bool:
        key = (row, col)
        if key in added:
            return True
        if key in removed:
            return False
        return csr_has_edge(original, row, col)

    def remove_current(key: tuple[int, int]) -> None:
        if key in added:
            added.remove(key)
        else:
            removed.add(key)

    def add_current(key: tuple[int, int]) -> None:
        if key in removed:
            removed.remove(key)
        else:
            added.add(key)

    successful = 0
    attempts = 0
    while successful < target_swaps and attempts < maximum_attempts:
        attempts += 1
        first = rng.randrange(coo.nnz)
        second = rng.randrange(coo.nnz - 1)
        if second >= first:
            second += 1
        row_a, col_i = int(rows[first]), int(cols[first])
        row_b, col_j = int(rows[second]), int(cols[second])
        if row_a == row_b or col_i == col_j:
            continue
        new_a = (row_a, col_j)
        new_b = (row_b, col_i)
        old_a = (row_a, col_i)
        old_b = (row_b, col_j)
        if new_a == new_b or new_a[0] == new_a[1] or new_b[0] == new_b[1]:
            continue
        # Existing old edges are momentarily removed, so a proposed new edge
        # equal to the counterpart old edge is still rejected as a no-op.
        if current_edge_exists(*new_a) or current_edge_exists(*new_b):
            continue
        remove_current(old_a)
        remove_current(old_b)
        add_current(new_a)
        add_current(new_b)
        cols[first] = col_j
        cols[second] = col_i
        successful += 1

    rewired = sparse.coo_matrix((coo.data, (rows, cols)), shape=coo.shape).tocsr()
    rewired.sum_duplicates()
    rewired.sort_indices()
    if rewired.nnz != original.nnz:
        raise RuntimeError("rewiring changed connection count")
    if not np.array_equal(np.diff(rewired.indptr), np.diff(original.indptr)):
        raise RuntimeError("rewiring changed postsynaptic in-degree")
    if not np.array_equal(np.bincount(rewired.indices, minlength=original.shape[1]), np.bincount(original.indices, minlength=original.shape[1])):
        raise RuntimeError("rewiring changed presynaptic out-degree")
    # New loops are rejected before every swap. No full edge set is built here:
    # materializing all 25M pairs would defeat the memory-safe delta design.
    if successful < target_swaps:
        raise RuntimeError(f"only completed {successful}/{target_swaps} swaps after {attempts} attempts")
    return rewired, {"attempts": attempts, "successful_swaps": successful}


def create_rewired_asset(source_dir: Path, output_dir: Path, seed: int, swaps: int) -> dict:
    source_dir = source_dir.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    weights_path = source_dir / "weights.npz"
    brain_path = source_dir / "brain.npz"
    if not weights_path.is_file() or not brain_path.is_file():
        raise FileNotFoundError("source_dir must contain brain.npz and weights.npz")
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing output directory: {output_dir}")
    output_dir.mkdir(parents=True)
    try:
        original = sparse.load_npz(weights_path)
        rewired, stats = degree_preserving_rewire(original, seed=seed, target_swaps=swaps)
        shutil.copy2(brain_path, output_dir / "brain.npz")
        sparse.save_npz(output_dir / "weights.npz", rewired, compressed=True)
        report = {
            "method": "directed_double_edge_swap",
            "seed": seed,
            "target_swaps": swaps,
            **stats,
            "shape": list(original.shape),
            "connection_count": int(original.nnz),
            "preserved": ["per-neuron in-degree", "per-neuron out-degree", "connection count", "weight-value multiset", "timestep and I/O external to asset"],
            "destroyed": ["biological presynaptic/postsynaptic pairing", "structured connectivity motifs"],
            "source": {
                "brain_sha256": sha256_file(brain_path),
                "weights_sha256": sha256_file(weights_path),
            },
            "output": {
                "brain_sha256": sha256_file(output_dir / "brain.npz"),
                "weights_sha256": sha256_file(output_dir / "weights.npz"),
            },
        }
        (output_dir / "rewire_manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return report
    except Exception:
        # Preserve evidence but do not leave an apparently valid partial asset.
        shutil.rmtree(output_dir)
        raise


def _self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="flybrain-rewire-") as temporary:
        root = Path(temporary)
        source = root / "source"
        source.mkdir()
        # A dense-enough, loop-free directed graph makes many swaps available.
        rows = np.array([0, 0, 1, 1, 2, 2, 3, 3, 4, 4], dtype=np.int32)
        cols = np.array([1, 2, 2, 3, 3, 4, 4, 0, 0, 1], dtype=np.int32)
        sparse.save_npz(source / "weights.npz", sparse.csr_matrix((np.arange(10.0) + 1.0, (rows, cols)), shape=(5, 5)))
        np.savez(source / "brain.npz", placeholder=np.array([1], dtype=np.int8))
        report = create_rewired_asset(source, root / "output", seed=17, swaps=3)
        assert report["successful_swaps"] == 3
        assert (root / "output" / "rewire_manifest.json").is_file()
        print("degree_preserving_rewire_self_test=passed")


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Original directory containing brain.npz and weights.npz")
    parser.add_argument("--output", type=Path, help="New empty output directory, for example ~/fly_brain/rewired/seed_1000")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--swaps", type=int, default=100_000)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        _self_test()
        return
    if args.source is None or args.output is None:
        parser.error("--source and --output are required unless --self-test is used")
    report = create_rewired_asset(args.source, args.output, args.seed, args.swaps)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
