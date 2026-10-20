# Copyright (c) 2026 David Smith
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob

from setuptools import find_packages, setup

package_name = 'r2_mocks'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='David Smith',
    maintainer_email='1734595+daveismith@users.noreply.github.com',
    description="Stand-ins for hardware R2's personality stack talks to: dome_mock.",
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dome_mock = r2_mocks.dome_mock:main',
        ],
    },
)
