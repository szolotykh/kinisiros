"""Keep the colcon/pip installation path aligned with the tested SDK baseline."""

import ast
from pathlib import Path

from packaging.requirements import Requirement
from pykinisi import __version__ as SDK_VERSION


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_sdk_dependency_matches_setup_and_installed_version():
    """Colcon does not install pip dependencies; both declarations must agree."""
    requirement = (PACKAGE_ROOT / 'requirements.txt').read_text(encoding='utf-8').strip()
    setup = ast.parse((PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8'))
    call = next(node for node in ast.walk(setup)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'setup')
    dependencies = ast.literal_eval(next(
        keyword.value for keyword in call.keywords if keyword.arg == 'install_requires'))
    assert requirement in dependencies
    specifier = Requirement(requirement).specifier
    assert SDK_VERSION in specifier, 'Install the declared SDK before running ROS tests'
    assert '2.1.0' not in specifier
    assert '2.5.0' in specifier
    assert '3.0.0' not in specifier
