from setuptools import setup

package_name = 'ae_line_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/urdf', ['urdf/ae_njl5901ar_8ch.xacro']),
        (f'share/{package_name}/launch', ['launch/line_sensor.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Keiichiro Kobayashi',
    maintainer_email='s23c1050@s.chibakoudai.jp',
    description='8-channel reflectance line sensor simulated in Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_sensor_node = ae_line_sensor.line_sensor_node:main',
        ],
    },
)

