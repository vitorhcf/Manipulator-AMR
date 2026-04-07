from glob import glob
import os

from setuptools import find_packages, setup


package_name = "my_bot_gz"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "models", "my_bot"), glob("models/my_bot/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitor",
    maintainer_email="vitor@example.com",
    description="Gazebo Sim bringup package for the my_bot differential drive robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "encoder_angles_sim = my_bot_gz.encoder_angles_sim:main",
        ],
    },
)
