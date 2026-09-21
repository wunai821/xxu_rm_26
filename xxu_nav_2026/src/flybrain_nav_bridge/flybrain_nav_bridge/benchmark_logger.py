"""Append-only runtime evidence for reproducible benchmark trials."""

from __future__ import annotations

import json
from pathlib import Path
from threading import Lock
from typing import Any, Mapping


class BenchmarkLogger:
    """Write compact JSONL samples without overwriting any prior trial data."""

    def __init__(self, root_dir: str, enabled: bool = False) -> None:
        self.enabled = bool(enabled)
        self._lock = Lock()
        self._file = None
        if self.enabled:
            raw_dir = Path(root_dir).expanduser() / "raw" / "runtime"
            raw_dir.mkdir(parents=True, exist_ok=True)
            self._file = (raw_dir / "flybrain_runtime.jsonl").open("a", encoding="utf-8")

    def record(self, payload: Mapping[str, Any]) -> None:
        if self._file is None:
            return
        with self._lock:
            self._file.write(json.dumps(payload, allow_nan=False, sort_keys=True) + "\n")
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            with self._lock:
                self._file.close()
                self._file = None
