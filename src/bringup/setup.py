from setuptools import find_packages, setup
import os
from glob import glob

package_name = "bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@yourcompany.com",
    description="System-level launch entrypoints for the autonomy workspace.",
    license="Apache-2.0",
)
