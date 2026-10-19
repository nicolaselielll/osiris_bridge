from setuptools import setup, find_packages
import pathlib

HERE = pathlib.Path(__file__).parent
long_description = (HERE / "README.md").read_text(encoding="utf-8")

setup(
    name='osiris-bridge',
    version='0.5.0',
    description='Osiris bridge: connects a ROS 2 robot (Humble and later) to Osiris',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/nicolaselielll/osiris-bridge',
    author='Nicolas Tuomaala',
    author_email='nicolas.tuomaala00@gmail.com',
    packages=find_packages(),
    package_data={
        'osiris_agent': [
            'bin/graph_watcher_x86_64',
            'bin/graph_watcher_aarch64',
            'bin/graph_watcher_aarch64_lyrical',
            'bin/graph_watcher_x86_64_jazzy',
            'bin/graph_watcher_aarch64_jazzy',
        ],
    },
    license='Apache-2.0',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3 :: Only',
        'Operating System :: OS Independent',
    ],
    keywords='ros2 humble jazzy kilted lyrical robotics monitoring bridge osiris',
    install_requires=[
        'websockets',
        'psutil',
        'pyzmq',
        'msgpack',
        'Pillow',
    ],
    extras_require={
        'ros': ['rclpy'],
    },
    entry_points={
        'console_scripts': [
            'osiris = osiris_agent.agent_node:main',
        ],
    },
)