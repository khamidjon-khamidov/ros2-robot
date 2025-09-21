from setuptools import find_packages, setup
import os
from glob import glob

package_name = "IAS0220_246075IVSM"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Include both .urdf and .xacro files in urdf folder
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf") + glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hamidjon",
    maintainer_email="hamidovhamid1998@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'test_node= IAS0220_246075IVSM.test_node:main'
            ],
    },
)
