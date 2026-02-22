from glob import glob
from setuptools import find_packages, setup

package_name = "hand_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seb",
    maintainer_email="seb@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
    data_files=[
        # ... Other data files
        # Include all launch files.
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*")),
        (f"share/{package_name}/urdf", glob("urdf/*")),
        (f"share/{package_name}/meshes", glob("meshes/*")),
        (f"share/{package_name}/rviz", glob("rviz/*")),
    ],
)
