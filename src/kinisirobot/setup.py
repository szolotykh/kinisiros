"""Install the Kinisi Omni robot application as a ROS 2 Python package."""

from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "kinisirobot"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml", "README.md", "CHANGELOG.rst", "LICENSE"]),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*.yaml")),
        ("share/" + PACKAGE_NAME + "/description", glob("description/*")),
        ("share/" + PACKAGE_NAME + "/docs", glob("docs/*")),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/maps", glob("maps/*")),
        ("share/" + PACKAGE_NAME + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="Sergey",
    maintainer_email="szolotykh88@gmail.com",
    description="ROS 2 application for the Kinisi Omni robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "kinisi_web_bridge = kinisirobot.web_bridge:main",
            "kinisi_lidar_supervisor = kinisirobot.lidar_supervisor:main",
        ],
    },
)
