#!/usr/bin/env python3

import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[1]


class ScanContextConfigurationTests(unittest.TestCase):
    def test_stage2_profile_is_complete_and_valid(self):
        config_path = (
            REPOSITORY_ROOT
            / "src"
            / "a2w_fastlio_mapping"
            / "config"
            / "scan_context.yaml"
        )
        with config_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)

        parameters = document["/**"]["ros__parameters"]["scan_context"]
        self.assertEqual(
            set(parameters),
            {
                "rings",
                "sectors",
                "max_radius_m",
                "sensor_height_m",
                "top_k",
                "exclude_recent",
                "max_distance",
            },
        )
        self.assertGreater(parameters["rings"], 0)
        self.assertGreater(parameters["sectors"], 0)
        self.assertGreater(parameters["max_radius_m"], 0.0)
        self.assertGreater(parameters["top_k"], 1)
        self.assertGreaterEqual(parameters["exclude_recent"], 0)
        self.assertGreater(parameters["max_distance"], 0.0)
        self.assertLessEqual(parameters["max_distance"], 1.0)


class PackageDependencyTests(unittest.TestCase):
    def test_internal_package_graph_is_acyclic(self):
        packages = {}
        for package_xml in (REPOSITORY_ROOT / "src").glob("*/package.xml"):
            root = ET.parse(package_xml).getroot()
            name = root.findtext("name")
            dependencies = {
                element.text
                for tag in ("depend", "build_depend", "exec_depend")
                for element in root.findall(tag)
                if element.text
            }
            packages[name] = dependencies

        internal = set(packages)
        graph = {name: dependencies & internal for name, dependencies in packages.items()}
        visiting = set()
        visited = set()

        def visit(name):
            if name in visiting:
                self.fail(f"internal package dependency cycle reaches {name}")
            if name in visited:
                return
            visiting.add(name)
            for dependency in graph[name]:
                visit(dependency)
            visiting.remove(name)
            visited.add(name)

        for package_name in graph:
            visit(package_name)

        common_dependencies = graph["a2w_fastlio_common"]
        self.assertFalse(
            common_dependencies
            & {
                "a2w_fastlio_mapping",
                "a2w_fastlio_map",
                "a2w_fastlio_localization",
            }
        )


class AlgorithmBoundaryTests(unittest.TestCase):
    @staticmethod
    def concrete_algorithm_references(source_root):
        forbidden = ("scan_context_place_recognition.hpp", "scancontext_tro/")
        references = []
        for source in source_root.rglob("*"):
            if source.suffix not in {".cpp", ".cc", ".h", ".hpp"}:
                continue
            content = source.read_text(encoding="utf-8")
            if any(name in content for name in forbidden):
                references.append(source)
        return references

    def test_boundary_guard_detects_a_concrete_upper_layer_include(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "bad.cpp"
            source.write_text(
                '#include "a2w_fastlio_common/scan_context_place_recognition.hpp"\n',
                encoding="utf-8",
            )
            self.assertEqual(self.concrete_algorithm_references(pathlib.Path(directory)), [source])

    def test_upper_layers_do_not_reference_scan_context_implementation(self):
        upper_layers = [
            REPOSITORY_ROOT / "src" / "a2w_fastlio_mapping",
            REPOSITORY_ROOT / "src" / "a2w_fastlio_localization",
        ]
        references = []
        for layer in upper_layers:
            if layer.exists():
                references.extend(self.concrete_algorithm_references(layer))
        self.assertEqual(references, [])


if __name__ == "__main__":
    unittest.main()
