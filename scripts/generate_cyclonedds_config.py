#!/usr/bin/env python3
"""Render and validate the per-run CycloneDDS XML configuration."""

from __future__ import annotations

import argparse
import ipaddress
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def ipv4_address(value: str) -> str:
    """Return a normalized IPv4 address or raise an argparse error."""
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{value!r} is not a valid IPv4 address") from error
    if parsed.version != 4:
        raise argparse.ArgumentTypeError(f"{value!r} is not a valid IPv4 address")
    return str(parsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--pc-address", required=True, type=ipv4_address)
    parser.add_argument("--peer-address", required=True, type=ipv4_address)
    return parser.parse_args()


def render(template: Path, pc_address: str, peer_address: str) -> str:
    source = template.read_text(encoding="utf-8")
    if source.count("@PC_ADDRESS@") != 1 or source.count("@PEER_ADDRESS@") != 1:
        raise ValueError("CycloneDDS template must contain each address placeholder exactly once")
    rendered = source.replace("@PC_ADDRESS@", pc_address).replace(
        "@PEER_ADDRESS@", peer_address
    )
    ET.fromstring(rendered)
    return rendered


def atomic_write(output: Path, content: str) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=output.parent, delete=False
        ) as stream:
            temporary_name = stream.name
            stream.write(content)
        os.replace(temporary_name, output)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main() -> None:
    args = parse_args()
    atomic_write(
        args.output,
        render(args.template, args.pc_address, args.peer_address),
    )


if __name__ == "__main__":
    main()
