"""Optional ament package; standalone module execution is also supported."""
from setuptools import setup

setup(name='tb3_grasp_ros2', version='0.1.0', packages=['tb3_grasp_ros2'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/tb3_grasp_ros2']),
                  ('share/tb3_grasp_ros2', ['package.xml'])],
      install_requires=['setuptools', 'numpy', 'python-fcl==0.7.0.11'],
      zip_safe=False, maintainer='mobility', maintainer_email='mobility@example.com',
      description='Independent guarded grasp planner bridge', license='Apache-2.0',
      entry_points={'console_scripts': ['grasp_bridge = tb3_grasp_ros2.bridge:main']})
