from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='ypearson',
    maintainer_email='yvan.pearson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'py_node = {package_name}.my_first_node:main',
            f'robot_news_station = {package_name}.robot_news_station:main',
            f'smartphone = {package_name}.smartphone:main',
            f'number_publisher = {package_name}.number_publisher:main',
            f'number_counter = {package_name}.number_counter:main',
            f'add_two_ints_server = {package_name}.add_two_ints_server:main',
            f'add_two_ints_client_no_oop = {package_name}.add_two_ints_client_no_oop:main',
            f'add_two_ints_client = {package_name}.add_two_ints_client:main',
            f'hw_status_publisher = {package_name}.hw_status_publisher:main',
            f'battery_node = {package_name}.battery:main',
            f'led_node = {package_name}.led:main',
            'image_publisher = my_py_pkg.image_publisher:main',
            'image_subscriber = my_py_pkg.image_subscriber:main',
        ],
    },
)

# executable_name = package_name.file_name.main_function
