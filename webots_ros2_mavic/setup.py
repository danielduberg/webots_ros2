"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'webots_ros2_mavic'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/mavic_world.wbt', 'worlds/.mavic_world.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/mavic_webots.urdf'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/Mavic2Pro.proto'
]))
data_files.append(('share/' + package_name + '/meshes', ['meshes/body.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/body_lenses.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/body_metal_parts.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/camera_chassis.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/camera_lens.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/camera_pitch.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/camera_yaw.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/front_led.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_a.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_a_joint.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_a_plates.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_b.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_b_joint.obj']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/helix_b_plates.obj']))
data_files.append(('share/' + package_name + '/textures', ['textures/fast_helix.png']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='2023.1.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Mavic 2 Pro robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
