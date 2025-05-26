from glob import glob
import os
from setuptools import find_packages, setup

package_name = "cv_control_for_landing"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="haechan",
    maintainer_email="eojin333c@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",

    entry_points={
        "console_scripts": [
                'yolo_center_publisher = cv_control_for_landing.yolo_center_publisher:main',
                'realsense_viewer = cv_control_for_landing.realsense_viewer:main',
            ],
    },
)
