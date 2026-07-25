"""Strict canonical model-catalog consumer tests."""

import json
import shutil
import tempfile
import unittest
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from myarm_m750_moveit_config.model_catalog import load_model_catalog


def _description_share() -> Path:
    return Path(get_package_share_directory("myarm_m750_description"))


def _copy_catalog_tree(destination: Path) -> None:
    source = _description_share()
    shutil.copytree(source / "config", destination / "config")
    shutil.copytree(source / "urdf", destination / "urdf")


class ModelCatalogTest(unittest.TestCase):
    """Ensure YAML is owned by a strict consumer and agrees with JSON."""

    def test_loads_verified_planning_variants(self) -> None:
        catalog = load_model_catalog(_description_share())

        self.assertEqual(catalog.base_link, "base_link")
        self.assertEqual(catalog.end_link, "tool0")
        self.assertEqual(len(catalog.kinematic_contract_sha256), 64)
        self.assertEqual(
            catalog.planning_variant("lightweight").collision_geometry,
            "primitive",
        )
        with self.assertRaisesRegex(ValueError, "full or lightweight"):
            catalog.planning_variant("kinematic")

    def test_rejects_unknown_yaml_field(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            package_root = Path(directory)
            _copy_catalog_tree(package_root)
            catalog_path = package_root / "config" / "model.yaml"
            document = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
            document["model"]["implicit_fallback"] = True
            catalog_path.write_text(
                yaml.safe_dump(document),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "unknown"):
                load_model_catalog(package_root)

    def test_rejects_variant_contract_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            package_root = Path(directory)
            _copy_catalog_tree(package_root)
            manifest_path = package_root / "config" / "model_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["variants"]["lightweight"][
                "kinematic_contract_sha256"
            ] = "0" * 64
            manifest_path.write_text(
                json.dumps(manifest),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "different kinematic"):
                load_model_catalog(package_root)


if __name__ == "__main__":
    unittest.main()
