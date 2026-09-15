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


class RegistrationConfigurationTests(unittest.TestCase):
    @staticmethod
    def parameters(filename):
        config_path = (
            REPOSITORY_ROOT
            / "src"
            / "a2w_fastlio_mapping"
            / "config"
            / filename
        )
        with config_path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)["/**"]["ros__parameters"]

    def test_registration_profile_covers_shared_pipeline(self):
        parameters = self.parameters("registration.yaml")
        self.assertEqual(set(parameters), {"local_map", "quatro", "nano_gicp", "pipeline"})
        self.assertEqual(
            set(parameters["local_map"]),
            {"neighbor_keyframes_before", "neighbor_keyframes_after", "voxel_leaf_m", "max_points"},
        )
        self.assertEqual(
            set(parameters["quatro"]),
            {
                "fpfh_normal_radius_m", "fpfh_radius_m", "noise_bound_m",
                "rotation_gnc_factor", "rotation_cost_threshold",
                "rotation_max_iterations", "estimate_scale", "optimized_matching",
                "descriptor_distance_threshold", "maximum_correspondences", "minimum_points",
            },
        )
        self.assertEqual(
            set(parameters["nano_gicp"]),
            {
                "maximum_correspondence_distance_m", "thread_count",
                "correspondence_randomness", "maximum_iterations",
                "transformation_epsilon", "rotation_epsilon", "regularization_method",
                "fitness_score_max_range_m", "minimum_points",
            },
        )
        self.assertEqual(set(parameters["pipeline"]), {"evidence_distance_m"})
        self.assertGreater(parameters["local_map"]["max_points"], 0)
        self.assertGreater(parameters["quatro"]["minimum_points"], 2)
        self.assertGreater(parameters["nano_gicp"]["minimum_points"], 2)
        self.assertGreater(parameters["pipeline"]["evidence_distance_m"], 0.0)

    def test_loop_validation_profile_covers_all_rejection_gates(self):
        parameters = self.parameters("loop_validation.yaml")["loop_validation"]
        self.assertEqual(
            set(parameters),
            {
                "maximum_fitness", "minimum_overlap", "minimum_correspondences",
                "maximum_translation_jump_m", "maximum_rotation_jump_rad",
                "minimum_candidate_distance_separation",
            },
        )
        self.assertGreaterEqual(parameters["maximum_fitness"], 0.0)
        self.assertGreaterEqual(parameters["minimum_overlap"], 0.0)
        self.assertLessEqual(parameters["minimum_overlap"], 1.0)
        self.assertGreater(parameters["minimum_correspondences"], 0)

    def test_pose_graph_profile_covers_graph_and_loop_coordinator(self):
        parameters = self.parameters("pose_graph.yaml")
        self.assertEqual(set(parameters), {"pose_graph", "loop_pipeline"})
        self.assertEqual(
            set(parameters["pose_graph"]),
            {
                "prior_rotation_sigma_rad", "prior_translation_sigma_m",
                "odometry_rotation_sigma_rad", "odometry_translation_sigma_m",
                "loop_rotation_sigma_rad", "loop_translation_sigma_m",
                "robust_kernel", "robust_kernel_scale",
                "relinearization_threshold", "relinearization_skip",
            },
        )
        self.assertIn(parameters["pose_graph"]["robust_kernel"], {"huber", "cauchy"})
        for name, value in parameters["pose_graph"].items():
            if name != "robust_kernel":
                self.assertGreater(value, 0, name)
        self.assertEqual(
            set(parameters["loop_pipeline"]),
            {
                "minimum_keyframes_between_accepted_loops", "queue_capacity",
            },
        )
        self.assertGreater(parameters["loop_pipeline"]["queue_capacity"], 0)


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
        self.assertIn("gtsam", packages["a2w_fastlio_mapping"])
        self.assertEqual(graph["a2w_fastlio_map"], {"a2w_fastlio_common"})
        self.assertTrue(
            {"a2w_fastlio_common", "a2w_fastlio_map", "a2w_fastlio_msgs"}
            <= graph["a2w_fastlio_mapping"]
        )
        self.assertTrue(
            {"a2w_fastlio_common", "a2w_fastlio_map", "a2w_fastlio_msgs"}
            <= graph["a2w_fastlio_localization"]
        )
        self.assertNotIn("a2w_fastlio_mapping", graph["a2w_fastlio_localization"])


class MapBundleContractTests(unittest.TestCase):
    def test_map_profile_marks_hardware_validation_pending(self):
        config_path = REPOSITORY_ROOT / "src" / "a2w_fastlio_map" / "config" / "map.yaml"
        with config_path.open("r", encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
        self.assertEqual(parameters["save_map_bundle_service"], "/mapping/save_map_bundle")
        self.assertEqual(parameters["save_queue_capacity"], 1)
        self.assertNotIn("hardware_validated", parameters)

    def test_full_map_is_only_a_bundle_file_not_a_mapping_topic(self):
        mapping_sources = REPOSITORY_ROOT / "src" / "a2w_fastlio_mapping"
        forbidden = 'create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/optimized_map"'
        for source in mapping_sources.rglob("*.cpp"):
            self.assertNotIn(forbidden, source.read_text(encoding="utf-8"))


class MappingOutputContractTests(unittest.TestCase):
    def test_mapping_topics_profile_uses_preview_not_full_map_dds(self):
        config_path = (
            REPOSITORY_ROOT / "src" / "a2w_fastlio_mapping" / "config" /
            "mapping_topics.yaml"
        )
        with config_path.open("r", encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        self.assertEqual(parameters["optimized_map_preview_topic"],
                         "/mapping/optimized_map_preview")
        self.assertNotIn("optimized_map_topic", parameters)
        self.assertEqual(parameters["preview_qos_durability"], "transient_local")
        self.assertEqual(parameters["preview_qos_depth"], 1)
        self.assertGreater(parameters["preview_voxel_leaf_m"], 0.0)
        self.assertGreater(parameters["preview_max_points"], 0)

    def test_mapping_launch_has_one_owner_and_no_localization_backend(self):
        launch_path = (
            REPOSITORY_ROOT / "src" / "a2w_fastlio2_bringup" / "launch" /
            "mapping.launch.py"
        )
        source = launch_path.read_text(encoding="utf-8")
        self.assertEqual(source.count('executable="mapping_backend_node"'), 1)
        self.assertEqual(source.count('executable="mapping_ingress_node"'), 1)
        self.assertNotIn("localization_backend", source)
        self.assertNotIn("localization.launch.py", source)


class LocalizationContractTests(unittest.TestCase):
    def test_localization_profile_has_one_owner_and_no_mapping_backend(self):
        launch_path = (
            REPOSITORY_ROOT / "src" / "a2w_fastlio2_bringup" / "launch" /
            "localization.launch.py"
        )
        source = launch_path.read_text(encoding="utf-8")
        self.assertEqual(source.count('executable="localization_node"'), 1)
        self.assertNotIn('executable="mapping_backend_node"', source)
        self.assertNotIn('executable="mapping_ingress_node"', source)

    def test_localization_matching_profile_exposes_every_registration_gate(self):
        config_path = (
            REPOSITORY_ROOT / "src" / "a2w_fastlio_localization" / "config" /
            "map_matching.yaml"
        )
        with config_path.open("r", encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
        self.assertEqual(set(parameters), {"local_map", "quatro", "nano_gicp", "validation", "pipeline"})
        self.assertEqual(
            set(parameters["quatro"]),
            {
                "fpfh_normal_radius_m", "fpfh_radius_m", "noise_bound_m",
                "rotation_gnc_factor", "rotation_cost_threshold",
                "rotation_max_iterations", "estimate_scale", "optimized_matching",
                "descriptor_distance_threshold", "maximum_correspondences", "minimum_points",
            },
        )
        self.assertEqual(
            set(parameters["nano_gicp"]),
            {
                "maximum_correspondence_distance_m", "thread_count",
                "correspondence_randomness", "maximum_iterations",
                "transformation_epsilon", "rotation_epsilon", "regularization_method",
                "fitness_score_max_range_m", "minimum_points",
            },
        )
        self.assertEqual(
            set(parameters["validation"]),
            {
                "maximum_fitness", "minimum_overlap", "minimum_correspondences",
                "maximum_translation_jump_m", "maximum_rotation_jump_rad",
                "minimum_candidate_distance_separation",
            },
        )


class AlgorithmBoundaryTests(unittest.TestCase):
    @staticmethod
    def concrete_algorithm_references(source_root):
        forbidden = (
            "scan_context_place_recognition.hpp",
            "quatro_registration.hpp",
            "nano_gicp_registration.hpp",
            "scancontext_tro/",
            "quatro/",
            "nano_gicp/",
            "teaser/",
        )
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
