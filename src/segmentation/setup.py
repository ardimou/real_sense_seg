from setuptools import find_packages, setup

package_name = 'segmentation'

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
    maintainer='argyris',
    maintainer_email='argyris@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['rgb_publisher = segmentation.publisher_rgb:main',
            'rgb_subscriber = segmentation.subscriber_rgb:main',
            'seg_publisher = segmentation.seg_subscriber:main',
            'seg_subscriber = segmentation.seg_publisher:main',
        ],
    },
)
