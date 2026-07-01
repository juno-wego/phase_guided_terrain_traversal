from glob import glob
from setuptools import setup

package_name = "wego_pgtt"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wego",
    maintainer_email="wego@example.com",
    description="PGTT mode management for the real Go2 stair-climbing stack.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_mode_server = wego_pgtt.policy_mode_server:main",
        ],
    },
)
