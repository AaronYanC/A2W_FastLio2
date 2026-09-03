#!/usr/bin/env python3
"""Behavior tests for the per-run CycloneDDS configuration generator."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GENERATOR = REPOSITORY_ROOT / "scripts" / "generate_cyclonedds_config.py"
SELECTOR = REPOSITORY_ROOT / "scripts" / "select_network_interface.py"
TEMPLATE_TEXT = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface address="@PC_ADDRESS@"/></Interfaces></General>
    <Discovery><Peers><Peer address="@PEER_ADDRESS@"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
"""


class RuntimeConfigTests(unittest.TestCase):
    def run_generator(
        self, directory: Path, pc_address: str, peer_address: str
    ) -> tuple[subprocess.CompletedProcess[str], Path]:
        template = directory / "cyclonedds.xml.in"
        output = directory / "runtime" / "cyclonedds.xml"
        template.write_text(TEMPLATE_TEXT, encoding="utf-8")
        result = subprocess.run(
            [
                sys.executable,
                str(GENERATOR),
                "--template",
                str(template),
                "--output",
                str(output),
                "--pc-address",
                pc_address,
                "--peer-address",
                peer_address,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        return result, output

    def test_writes_parseable_xml_with_selected_addresses(self) -> None:
        """A wrong placeholder replacement would configure DDS on the wrong network."""
        with tempfile.TemporaryDirectory() as temp_dir:
            result, output = self.run_generator(
                Path(temp_dir), "192.168.123.77", "192.168.123.164"
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            root = ET.parse(output).getroot()
            interface = root.find(".//{*}NetworkInterface")
            peer = root.find(".//{*}Peer")
            self.assertIsNotNone(interface)
            self.assertIsNotNone(peer)
            self.assertEqual(interface.attrib["address"], "192.168.123.77")
            self.assertEqual(peer.attrib["address"], "192.168.123.164")

    def test_rejects_invalid_ipv4_without_creating_output(self) -> None:
        """Accepting malformed input would defer a clear error to CycloneDDS startup."""
        with tempfile.TemporaryDirectory() as temp_dir:
            result, output = self.run_generator(
                Path(temp_dir), "not-an-ip", "192.168.123.164"
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("valid IPv4", result.stderr)
            self.assertFalse(output.exists())


class NetworkSelectorTests(unittest.TestCase):
    def test_prefers_direct_subnet_over_tunnel_route(self) -> None:
        """A tunnel route must not hide the Ethernet address on the robot subnet."""
        addresses = """2: enp49s0    inet 192.168.123.100/24 brd 192.168.123.255 scope global enp49s0
9: FlClash    inet 28.0.0.1/8 scope global FlClash
"""
        result = subprocess.run(
            [sys.executable, str(SELECTOR), "--peer", "192.168.123.164"],
            input=addresses,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "enp49s0 192.168.123.100")

    def test_reports_no_match_when_peer_is_not_directly_connected(self) -> None:
        """Inventing a direct match would select an unrelated PC interface."""
        addresses = "9: FlClash    inet 28.0.0.1/8 scope global FlClash\n"
        result = subprocess.run(
            [sys.executable, str(SELECTOR), "--peer", "192.168.123.164"],
            input=addresses,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
