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
                "scenarios/judge-judge.json",
                "scenarios/judge-judge(cross).json",
                "scenarios/kid-terrorist.json",
                "scenarios/terrorist-kid.json",
                "scenarios/walk_away.json",
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
