from glob import glob
from setuptools import find_packages, setup

package_name = "module_robot_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Module Robot Team",
    maintainer_email="robot@localhost",
    description="Command arbitration and safety policy",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["safety_node = module_robot_safety.safety_node:main"]},
)
