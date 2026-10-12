import os
import subprocess

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = "dagger_gateway"
HERE = os.path.dirname(os.path.abspath(__file__))


class GenerateThenBuild(build_py):
    """api_pb2.py (from proto/) and build_info.py (the version HelloReply reports)
    are generated on every build, never committed."""

    def run(self):
        subprocess.check_call(["bash", os.path.join(HERE, "..", "..", "tools", "gen_proto.sh")])
        rev = subprocess.run(["git", "-C", HERE, "describe", "--always", "--dirty"],
                             capture_output=True, text=True).stdout.strip() or "unknown"
        with open(os.path.join(HERE, package_name, "build_info.py"), "w") as f:
            f.write(f'VERSION = "gateway 0.1.0 ({rev})"\n')
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
