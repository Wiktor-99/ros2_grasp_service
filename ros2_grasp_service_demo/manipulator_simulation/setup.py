from setuptools import find_packages, setup
from glob import glob
import os

package_name = "manipulator_simulation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (
            os.path.join("share", package_name, "models/ground_plane"),
            glob("models/ground_plane/model.sdf"),
        ),
        (
            os.path.join("share", package_name, "models/ground_plane"),
            glob("models/ground_plane/model.config"),
        ),
        (os.path.join("share", package_name, "models/sun"), glob("models/sun/model.sdf")),
        (os.path.join("share", package_name, "models/sun"), glob("models/sun/model.config")),
        (
            os.path.join("share", package_name, "models/sun/materials/textures"),
            glob("models/sun/materials/textures/*.jpg"),
        ),
        (
            os.path.join("share", package_name, "models/sun/meshes"),
            glob("models/sun/meshes/*.dae"),
        ),
        (os.path.join("share", package_name, "models/big_box"), glob("models/big_box/*.*")),
        (
            os.path.join("share", package_name, "models/big_box/meshes"),
            glob("models/big_box/meshes/*"),
        ),
        (
            os.path.join("share", package_name, "models/big_box/materials/textures"),
            glob("models/big_box/materials/textures/*"),
        ),
        (
            os.path.join("share", package_name, "models/asphalt_plane"),
            glob("models/asphalt_plane/*.*"),
        ),
        (
            os.path.join("share", package_name, "models/asphalt_plane/materials/textures"),
            glob("models/asphalt_plane/materials/textures/*"),
        ),
        (
            os.path.join("share", package_name, "models/asphalt_plane/materials/scripts"),
            glob("models/asphalt_plane/materials/scripts/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wiktor Bajor",
    maintainer_email="wiktorbajor1@gmail.com",
    description="Gazebo simulation package for OpenManipulator an custom 6DoF manipulator",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
