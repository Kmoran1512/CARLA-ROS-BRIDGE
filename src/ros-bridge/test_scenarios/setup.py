import os
from glob import glob
from setuptools import setup

package_name = "test_scenarios"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/scenarios",
            [
                "scenarios/(LS)Bike-Child(X).json",
                "scenarios/(LS)Bike-Police(X).json",
                "scenarios/(LS)Bike-Terrorist(X).json",
                "scenarios/(RS)Bike-Child(X).json",
                "scenarios/(RS)Bike-Police(X).json",
                "scenarios/(RS)Bike-Terrorist(X).json",
                #
                "scenarios/(LS)Child-Bike(X).json",
                "scenarios/(LS)Child-Child(X).json",
                "scenarios/(LS)Child-Police(X).json",
                "scenarios/(LS)Child-Terrorist(X).json",
                "scenarios/(LS)Police-Bike(X).json",
                "scenarios/(LS)Police-Child(X).json",
                "scenarios/(LS)Police-Police(X).json",
                "scenarios/(LS)Police-Terrorist(X).json",
                "scenarios/(LS)Terrorist-Bike(X).json",
                "scenarios/(LS)Terrorist-Child(X).json",
                "scenarios/(LS)Terrorist-Police(X).json",
                "scenarios/(LS)Terrorist-Child(X).json",
                #
                "scenarios/(RS)Child-Bike(X).json",
                "scenarios/(RS)Child-Child(X).json",
                "scenarios/(RS)Child-Police(X).json",
                "scenarios/(RS)Child-Terrorist(X).json",
                "scenarios/(RS)Police-Bike(X).json",
                "scenarios/(RS)Police-Child(X).json",
                "scenarios/(RS)Police-Police(X).json",
                "scenarios/(RS)Police-Terrorist(X).json",
                "scenario/(RS)Terrorist-Bike(X).json",
                "scenarios/(RS)Terrorist-Child(X).json",
                "scenarios/(RS)Terrorist-Police(X).json",
                "scenarios/(RS)Terrorist-Terrorist(X).json",
            ],
        ),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kyle",
    maintainer_email="kmoran.1512@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["test_scenarios = test_scenarios.test_scenarios:main"]
    },
    package_dir={"": "src"},
)
