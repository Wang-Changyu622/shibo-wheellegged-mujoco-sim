from glob import glob
from setuptools import find_packages, setup

setup(
    name="wheellegged_ros",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/wheellegged_ros"]),
        ("share/wheellegged_ros", ["package.xml"]),
        ("share/wheellegged_ros/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    description="ROS 2 adapter for the shibo-wheellegged MuJoCo simulation",
    maintainer="shibo-wheellegged contributors",
    maintainer_email="260433968+Wang-Changyu622@users.noreply.github.com",
    license="MIT",
    entry_points={"console_scripts": ["sim_node = wheellegged_ros.sim_node:main"]},
)
