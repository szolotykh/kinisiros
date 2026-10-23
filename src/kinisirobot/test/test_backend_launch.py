"""Validate launch composition without importing ROS launch on development hosts."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('backend,expected', [
    ('false', {'controller_server', 'planner_server', 'behavior_server',
               'bt_navigator', 'lifecycle_manager_navigation'}),
    ('true', {'controller_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'lifecycle_manager_navigation', 'slam_toolbox',
              'map_server', 'amcl', 'lifecycle_manager_localization'}),
])
def test_managed_mode_does_not_launch_a_second_map_provider(backend, expected):
    """Run the actual launch builder with small stand-ins for ROS launch types."""
    path = Path(__file__).resolve().parents[1] / 'launch/navigation.launch.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == 'launch_setup')

    class Configuration:
        """Resolve just the launch arguments needed by the builder."""

        def __init__(self, name):
            """Retain the unresolved launch argument name."""
            self.name = name

        def perform(self, context):
            """Read the test's launch context."""
            return context.get(self.name, '')

    namespace = dict(LaunchConfiguration=Configuration, Node=lambda **kw: kw,
                     IfCondition=lambda arg: arg, UnlessCondition=lambda arg: arg)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
    nodes = namespace['launch_setup']({'backend': backend})
    assert {node['name'] for node in nodes} == expected


def test_base_launch_uses_lidar_supervisor_instead_of_direct_driver():
    """Keep restart and scan-health policy in the robot application."""
    path = Path(__file__).resolve().parents[1] / 'launch/rsp.launch.py'
    source = path.read_text(encoding='utf-8')
    tree = ast.parse(source)
    node_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
    ]

    def keyword(call, name):
        value = next(item.value for item in call.keywords if item.arg == name)
        return value.value if isinstance(value, ast.Constant) else None

    launched = {(keyword(call, 'package'), keyword(call, 'executable'))
                for call in node_calls}
    assert ('kinisirobot', 'kinisi_lidar_supervisor') in launched
    assert not any(package == 'rplidar_ros' for package, _ in launched)
