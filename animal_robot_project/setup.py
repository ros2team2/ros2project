from setuptools import find_packages, setup

package_name = 'animal_robot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_bringup.launch.py']),
        ('share/' + package_name + '/models', ['models/best.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lansional',
    maintainer_email='lansional@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtleautocontroller = animal_robot_project.turtleautocontroller:main'
        ],
    },
)
