#!/usr/bin/env python3
"""Fetch only the pinned third-party CAD assets, verifying every file's SHA-256.

The lock file contains metadata, not redistributed model or mesh contents.
Existing matching assets are reused; modified assets are never overwritten.
Only the Python standard library is required, including for offline checks.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
import time
from urllib.error import URLError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
LOCK = Path(__file__).with_name("asset-lock.json")
REPOSITORY = "dycsrl/Micro-Wheeled_leg-Robot-Model"
COMMIT = "a330459b2d3e4567974bb6415abb51086fa5d683"


def load_lock() -> list[dict]:
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    if (lock.get("schema_version"), lock.get("repository"), lock.get("commit")) != (
        1, REPOSITORY, COMMIT
    ):
        raise ValueError("Asset lock does not match the pinned repository and revision")
    entries = lock["files"]
    seen: set[str] = set()
    for entry in entries:
        for field in ("source", "destination"):
            path = PurePosixPath(entry[field])
            if path.is_absolute() or ".." in path.parts or "\\" in entry[field]:
                raise ValueError(f"Unsafe asset path: {entry[field]!r}")
        if entry["destination"] in seen:
            raise ValueError(f"Duplicate asset destination: {entry['destination']}")
        seen.add(entry["destination"])
        if not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]) or entry["size"] <= 0:
            raise ValueError(f"Invalid hash or size: {entry['source']}")
    return entries


def validate(content: bytes, entry: dict) -> None:
    if len(content) != entry["size"] or hashlib.sha256(content).hexdigest() != entry["sha256"]:
        raise ValueError(f"SHA-256/size mismatch: {entry['destination']}")


def missing_assets(destination: Path, entries: list[dict]) -> list[dict]:
    missing = []
    for entry in entries:
        path = destination / entry["destination"]
        if path.exists():
            validate(path.read_bytes(), entry)
        else:
            missing.append(entry)
    return missing


def read_asset(entry: dict, source: Path | None) -> bytes:
    if source is not None:
        content = (source / entry["source"]).read_bytes()
    else:
        url = f"https://raw.githubusercontent.com/{REPOSITORY}/{COMMIT}/{entry['source']}"
        request = Request(url, headers={"User-Agent": "shibo-wheellegged-asset-fetch/1"})
        for attempt in range(3):
            try:
                with urlopen(request, timeout=30) as response:
                    # Read at most the locked size plus one byte; hashes remain authoritative.
                    content = response.read(entry["size"] + 1)
                break
            except (URLError, TimeoutError):
                if attempt == 2:
                    raise
                time.sleep(attempt + 1)
    validate(content, entry)
    return content


def install_asset(entry: dict, destination: Path, source: Path | None) -> None:
    content = read_asset(entry, source)
    target = destination / entry["destination"]
    target.parent.mkdir(parents=True, exist_ok=True)
    # A hard-link installation is atomic and cannot replace a file created meanwhile.
    with tempfile.NamedTemporaryFile(dir=target.parent, prefix=".asset-", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.chmod(temporary, 0o644)
        try:
            os.link(temporary, target)
        except FileExistsError:
            validate(target.read_bytes(), entry)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=ROOT / "assets" / "cad",
                        help="Asset directory (default: project assets/cad)")
    parser.add_argument("--check", action="store_true", help="Verify local assets without network or writes")
    parser.add_argument("--source-dir", type=Path, help="Read files from an existing upstream checkout instead of the network")
    parser.add_argument("--jobs", type=int, default=8, choices=range(1, 17), metavar="1..16")
    args = parser.parse_args(argv)
    try:
        entries = load_lock()
        missing = missing_assets(args.destination, entries)
        if args.check:
            if missing:
                print(f"Missing {len(missing)}/{len(entries)} assets; first: {missing[0]['destination']}", file=sys.stderr)
                return 1
        elif missing:
            print(f"Fetching {len(missing)} files from {REPOSITORY}@{COMMIT[:12]}", flush=True)
            with ThreadPoolExecutor(max_workers=args.jobs) as pool:
                futures = [pool.submit(install_asset, entry, args.destination, args.source_dir) for entry in missing]
                for future in as_completed(futures):
                    future.result()
            if missing_assets(args.destination, entries):
                raise RuntimeError("Asset installation is incomplete")
        total = sum(entry["size"] for entry in entries)
        print(f"Verified {len(entries)} assets ({total:,} bytes): {args.destination.resolve()}")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"Asset verification failed: {error}", file=sys.stderr)
        print("Existing modified files were not overwritten. Restore or move them before retrying.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
