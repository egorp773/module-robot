from glob import glob
from setuptools import find_packages, setup


package_name = "module_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Module Robot Maintainers",
    maintainer_email="robot@localhost",
    description="Fail-closed launch orchestration for the modular robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autonomy_preflight = module_robot_bringup.preflight:main",
        ],
    },
)
