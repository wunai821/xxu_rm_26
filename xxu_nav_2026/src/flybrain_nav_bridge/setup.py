from glob import glob

from setuptools import setup


package_name = "flybrain_nav_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/models/flybrain_obstacle", glob("models/flybrain_obstacle/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="naiwu",
    maintainer_email="naiwu@example.com",
    description="Safe ROS 2 shadow and fusion bridge for MaleCNS threat-response experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "flybrain_node = flybrain_nav_bridge.flybrain_node:main",
            "scenario_manager = flybrain_nav_bridge.scenario_manager:main",
            "benchmark_metrics = flybrain_nav_bridge.benchmark_metrics:main",
            "benchmark_batch = flybrain_nav_bridge.benchmark_batch:main",
            "validate_benchmark = flybrain_nav_bridge.validate_benchmark:main",
            "freeze_benchmark = flybrain_nav_bridge.freeze_benchmark:main",
        ],
    },
)
