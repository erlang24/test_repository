from setuptools import setup

package_name = 'obu_json'

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
    maintainer='erlang',
    maintainer_email='erlang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # !!!!!!!!!!!!!!!!!!!
            "obu_handle_node = obu_json.obu_handle_node:main",
            "decode_node = obu_json.decode_node:main",
            "encode_node = obu_json.encode_node:main",
        ],
    },
)
