import os
from glob import glob
from setuptools import setup

package_name = "record_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/bag_config.yaml",
                "config/column_headers.txt",
                "config/semantic_map.json",
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
        "console_scripts": [
            "bag_remap = record_node.bag_remap:main",
            "record_node = record_node.record_node:main",
            "img_publisher = record_node.img_publisher:main",
            "semantic_boxes = record_node.semantic_boxes:main",
        ]
    },
)
