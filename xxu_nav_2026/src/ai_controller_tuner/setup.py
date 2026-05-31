from setuptools import setup
from glob import glob

package_name = "ai_controller_tuner"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="naiwu",
    maintainer_email="naiwu@example.com",
    description="AI-assisted runtime tuning node for the pb omni PID pursuit controller.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "ai_controller_tuner = ai_controller_tuner.ai_controller_tuner:main",
            "ai_controller_goal_sender = ai_controller_tuner.goal_sender:main",
        ],
    },
)
