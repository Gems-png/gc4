from setuptools import find_packages, setup
from glob import glob

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gc480p.json']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gems',
    maintainer_email='2425721741@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "raw_image_pub=cv.raw_image_pub:main",
            "tag_pose=cv.tag_pose:main",
            "cam_pos=cv.cam_pos:main",
            "tag_image_pub=cv.tag_image_pub:main",
        ],
    },
)
