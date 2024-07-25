from setuptools import setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'gui.rviz2_gui',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz2_gui = gui.rviz2_gui:main',
        ],
    },
)