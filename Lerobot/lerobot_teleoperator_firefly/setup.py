from setuptools import setup, find_packages

setup(
    name="lerobot_teleoperator_firefly",
    version="0.0.4",
    description="LeRobot firefly integration",
    author="Welt-liu",
    author_email="1994524450@qq.com",
    packages=find_packages(),
    install_requires=[
        "lerobot",
        "fashionstar-uart-sdk>=1.3.6"
    ],
    python_requires=">=3.10",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
)
