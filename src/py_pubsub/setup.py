from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='charan',
    maintainer_email='charan@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'multi_publisher = py_pubsub.multi_publisher:main',
            'multi_subscriber = py_pubsub.multi_subscriber:main',
            'draw_circles = py_pubsub.draw_circles:main',
            'zigzag_clean = py_pubsub.zigzag_clean:main',
            'draw_letters = py_pubsub.draw_letters:main',
            'navigate_turtlebot3 = py_pubsub.navigate_turtlebot3:main',
        ],
    },
)
