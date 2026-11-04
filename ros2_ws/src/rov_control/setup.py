from setuptools import setup

package_name = "rov_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mixer.yaml", "config/attitude.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wkusek",
    maintainer_email="wkusek2@gmail.com",
    description="Mikser pednikow ROV",
    license="GPL-3.0",
    entry_points={"console_scripts": [
        "thruster_mixer = rov_control.thruster_mixer:main",
        "event_logger = rov_control.event_logger:main",
        "attitude_controller = rov_control.attitude_controller:main",
        "sim_pico = rov_control.sim_pico:main",
    ]},
)
