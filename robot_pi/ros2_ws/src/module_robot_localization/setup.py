from glob import glob
from setuptools import find_packages, setup


package_name = "module_robot_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Module Robot Maintainers",
    maintainer_email="robot@localhost",
    description="Disabled-by-default localization templates and guarded heading initializer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "module_robot_heading_initializer = "
            "module_robot_localization.heading_initializer:main",
        ],
    },
)
