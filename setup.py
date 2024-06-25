from setuptools import find_packages, setup

package_name = "zed_gst"

setup(
    name=package_name,
    version="0.1.4",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="linus",
    maintainer_email="linuskirkwood@gmail.com",
    description="Runs a gstreamer pipeline the publishes video from a Zed 2 camera.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["zed_cam = zed_gst.zed_cam:main"],
    },
)
