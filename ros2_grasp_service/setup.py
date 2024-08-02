from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup
from glob import glob
import os

package_name = "ros2_grasp_service"

generate_parameter_module("grasp_service_parameters", "ros2_grasp_service/grasp_service_parameters.yaml")

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wiktor Bajor",
    maintainer_email="wiktorbajor1@gmail.com",
    description="Simple grasp service",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ros2_grasp_service = ros2_grasp_service.grasp_service:main"],
    },
)
