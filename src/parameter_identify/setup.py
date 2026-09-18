from setuptools import find_packages, setup

package_name = "parameter_identify"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/identify.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyz",
    maintainer_email="cloud1202@qq.com",
    description="Offline dynamic parameter identification tools for the robotic arm.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "identify_arm = parameter_identify.identify_arm:main",
        ],
    },
)
