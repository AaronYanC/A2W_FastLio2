#!/usr/bin/env python3
"""Select the most specific directly connected IPv4 interface for a peer."""

from __future__ import annotations

import argparse
import ipaddress
import sys


def peer_address(value: str) -> ipaddress.IPv4Address:
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{value!r} is not a valid IPv4 address") from error
    if not isinstance(address, ipaddress.IPv4Address):
        raise argparse.ArgumentTypeError(f"{value!r} is not a valid IPv4 address")
    return address


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peer", required=True, type=peer_address)
    return parser.parse_args()


def select_interface(
    peer: ipaddress.IPv4Address, lines: list[str]
) -> tuple[str, ipaddress.IPv4Address] | None:
    candidates: list[tuple[int, str, ipaddress.IPv4Address]] = []
    for line in lines:
        fields = line.split()
        if len(fields) < 4 or "inet" not in fields:
            continue
        inet_index = fields.index("inet")
        if inet_index + 1 >= len(fields):
            continue
        try:
            local = ipaddress.ip_interface(fields[inet_index + 1])
        except ValueError:
            continue
        if not isinstance(local, ipaddress.IPv4Interface) or peer not in local.network:
            continue
        interface = fields[1].rstrip(":").split("@", maxsplit=1)[0]
        candidates.append((local.network.prefixlen, interface, local.ip))

    if not candidates:
        return None
    _, interface, address = max(candidates, key=lambda item: item[0])
    return interface, address


def main() -> int:
    args = parse_args()
    selected = select_interface(args.peer, list(sys.stdin))
    if selected is None:
        return 1
    interface, address = selected
    print(interface, address)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
