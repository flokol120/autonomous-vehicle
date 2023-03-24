from setuptools import setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'default = ' + package_name  + '.' + package_name + ':default', # Eveything enabled
                'debug = ' + package_name  + '.' + package_name + ':debug', # Everything enabled, debug enabled
                'image = ' + package_name  + '.' + package_name + ':image', # Image processing, no steering
                'image_debug = ' + package_name  + '.' + package_name + ':image_debug', # Image processing, no steering, debug enabled
        ]
    },
)