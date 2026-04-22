from glob import glob
import os

from setuptools import find_packages, setup


package_name = "yolo_detector"


def collect_files(root_dir, pattern):
    data_files = []
    for root, _, _ in os.walk(root_dir):
        files = glob(os.path.join(root, pattern))
        if not files:
            continue
        rel_root = os.path.relpath(root, root_dir)
        install_dir = os.path.join("share", package_name, root_dir)
        if rel_root != ".":
            install_dir = os.path.join(install_dir, rel_root)
        data_files.append((install_dir, files))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + collect_files("launch", "*.py") + collect_files("config", "*.yaml"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="venom",
    maintainer_email="liyihan.xyz@gmail.com",
    description="YOLO detector node publishing standardized perception outputs.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "yolo_node = yolo_detector.yolo_node:main",
        ],
    },
)
