import os
from glob import glob

from setuptools import find_packages, setup

package_name = "agv_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enes Erdoğan",
    maintainer_email="190140116+EnesErdogans@users.noreply.github.com",
    description="AGV WebSocket gateway (ROS 2 <-> ground station)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gateway_node = agv_gateway.gateway_node:main",
            "ws_cli_test = agv_gateway.tools.ws_cli_test:main",
        ],
    },
)
