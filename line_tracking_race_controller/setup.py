from setuptools import find_packages, setup

package_name = 'line_tracking_race_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brairlab',
    maintainer_email='g.zerini1@studenti.unipi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = line_tracking_race_controller.planner_node:main',
            'control_node = line_tracking_race_controller.control_node:main',
            'control_node_nl = line_tracking_race_controller.control_node_nl:main',
            'planner_node_nl = line_tracking_race_controller.planner_node_nl:main',
            'visualizer = line_tracking_race_controller.visualizer:main',
            'control_node_sel = line_tracking_race_controller.control_node_sel:main',
            'planner_node_sel = line_tracking_race_controller.planner_node_sel:main',
        ],
    },
)
