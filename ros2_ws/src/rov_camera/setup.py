from setuptools import setup

package_name = "rov_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wkusek",
    maintainer_email="wkusek2@gmail.com",
    description="Kamera IMX708 ROV",
    license="GPL-3.0",
    entry_points={"console_scripts": ["camera_node = rov_camera.camera_node:main"]},
)
