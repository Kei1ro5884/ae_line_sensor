from setuptools import setup, find_packages
package_name = 'ae_line_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),

    #
    # ★―― ここが肝心 ――★
    #   lib/ae_line_sensor/line_sensor_node  という
    #   拡張子なしラッパーを必ずインストールさせる
    #
    data_files=[
        ('lib/' + package_name,            # ← libexec ディレクトリ
         ['scripts/line_sensor_node']),    # ← 後述のラッパーファイル
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/urdf',
         ['urdf/ae_njl5901ar_8ch.xacro']),
        (f'share/{package_name}/launch',
         ['launch/line_sensor.launch.py']),
    ],

    # ふつうの console_script（bin/ にも置かれる）
    entry_points={
        'console_scripts': [
            'line_sensor_node = ae_line_sensor.line_sensor_node:main',
        ],
    },

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keiichiro Kobayashi',
    maintainer_email='s23c1050@s.chibakoudai.jp',
    description='AE-NJL5901AR-8CH reflectance sensor (Gazebo + Python)',
    license='MIT',
)

