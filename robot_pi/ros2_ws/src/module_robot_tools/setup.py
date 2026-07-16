from glob import glob
from setuptools import find_packages, setup


package_name = "module_robot_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Module Robot Maintainers",
    maintainer_email="robot@localhost",
    description="Read-only readiness diagnostics and a zero-only helper.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "readiness_monitor = module_robot_tools.readiness_monitor:main",
            "publish_zero = module_robot_tools.zero_command:main",
        ],
    },
)
