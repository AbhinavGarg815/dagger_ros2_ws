import os
import subprocess

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = "dagger_gateway"
HERE = os.path.dirname(os.path.abspath(__file__))


class GenerateThenBuild(build_py):
    """api_pb2.py is generated from proto/ on every build, never committed."""

    def run(self):
        subprocess.check_call(["bash", os.path.join(HERE, "..", "..", "tools", "gen_proto.sh")])
        super().run()


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    cmdclass={"build_py": GenerateThenBuild},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson@todo.todo",
    description="The operator app's gateway: protobuf over WebSocket on :8090",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gateway = dagger_gateway.gateway:main",
        ],
    },
)
