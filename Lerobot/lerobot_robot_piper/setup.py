from setuptools import setup, find_packages

setup(
    name="lerobot_robot_piper",
    version="0.0.1",
    description="LeRobot robot_piper integration",
    author="Welt-liu",
    author_email="1994524450@qq.com",
    packages=find_packages(),
    install_requires=[
        "lerobot",
        "fashionstar-uart-sdk>=1.3.6",
        "python-can",
        "piper_sdk"
    ],
    python_requires=">=3.10",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
)
