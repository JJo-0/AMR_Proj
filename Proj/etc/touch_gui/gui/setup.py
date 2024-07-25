from setuptools import setup
import os
from glob import glob

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='vscode@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz2_gui = gui.rviz2_gui:main'
        ],
    },
    data_files=[
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
)