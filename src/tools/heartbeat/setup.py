from setuptools import find_packages, setup
import os

package_name = "heartbeat"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install the executable script into lib/<package>
        (os.path.join("lib", package_name), ["scripts/heartbeat"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@yourcompany.com",
    description="Minimal node used to validate workspace build and bringup launch.",
    license="Apache-2.0",
)
