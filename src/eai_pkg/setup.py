from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'eai_pkg'
submodules = "eai_pkg/openvino"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eai',
    maintainer_email='eai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # zyt's nodes
            'realsense_node = eai_pkg.realsense_node:main',
            'layout_node = eai_pkg.layout_node:main',
            'center_node = eai_pkg.center_node:main',
            'arm_node = eai_pkg.arm_node:main',
            'arm_node_test = eai_pkg.arm_node_test:main',
            'speaker_node = eai_pkg.speaker_node:main',
            #hs的节点
            'user_cmd_node = eai_pkg.user_cmd_node:main',
            'ai_node = eai_pkg.ai_node:main',
            'microphone_node = eai_pkg.microphone_node:main',
            'self_learning_node = eai_pkg.self_learning_node:main',
            'asrpro_node = eai_pkg.asrpro_node:main',
            #hs的测试节点
            'self_learning_client = eai_pkg.test_self_learning:main',
            'ai_cmd_client = eai_pkg.test_prompt:main',
        ],
    },
)
