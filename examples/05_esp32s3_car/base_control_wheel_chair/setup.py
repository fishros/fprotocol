from setuptools import find_packages, setup

package_name = "base_control_wheel_chair"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/serial_keyboard.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fishros",
    maintainer_email="87068644+fishros@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "robot_serial = base_control_wheel_chair.robot_ros2_serial:main",
            "keyboard_teleop = base_control_wheel_chair.keyboard_teleop:main",
        ],
    },
)
