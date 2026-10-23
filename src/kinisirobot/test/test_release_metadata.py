"""Release metadata and package-boundary regression checks."""

import ast
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from kinisirobot import __version__


def package_version(path):
    """Read one ROS package version from package.xml."""
    return ET.parse(path).getroot().findtext("version")


def test_kinisirobot_versions_match_and_metadata_has_no_placeholders():
    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert package_version(PACKAGE_ROOT / "package.xml") == __version__
    assert f'version="{__version__}"' in setup
    assert "TODO" not in manifest
    assert "TODO" not in setup


def test_roskinisi_does_not_depend_on_robot_application():
    driver_root = REPOSITORY_ROOT / "src" / "roskinisi"
    manifest = ET.parse(driver_root / "package.xml").getroot()
    dependencies = {element.text for element in manifest if element.tag.endswith("depend")}
    assert "kinisirobot" not in dependencies

    forbidden = ("kinisirobot", "nav2_", "slam_toolbox", "rplidar_ros")
    for source in (driver_root / "roskinisi").glob("*.py"):
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
        assert not any(name.startswith(forbidden) for name in imports), source


def test_release_files_and_installed_assets_are_declared():
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    required = [
        "CHANGELOG.rst", "LICENSE", "README.md", 'glob("config/*.yaml")',
        'glob("description/*")', 'glob("docs/*")', 'glob("launch/*.launch.py")',
        'glob("maps/*")', 'glob("web/*")',
    ]
    assert all(item in setup for item in required)
    assert re.search(r"kinisi_web_bridge\s*=", setup)
    assert re.search(r"kinisi_lidar_supervisor\s*=", setup)
