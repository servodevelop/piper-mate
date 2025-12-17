from setuptools import find_packages, setup
from glob import glob

package_name = "star_piper"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ny",
    maintainer_email="1994524450@qq.com",
    description="Examples of FsRobo_A1",
    license="Apache License 2.0",
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        "console_scripts": [
            "driver = star_piper.star_piper:main",
        ],
    },
)
