from setuptools import find_packages, setup

package_name = "orbslam_helpers"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="amarburg@uw.edu",
    description="TODO: Package description",
    license="BSD 3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_info_injector = orbslam_helpers.camera_info_injector:main"
        ],
    },
)
