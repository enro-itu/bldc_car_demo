from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bldc_car_demo')

    world_path = os.path.join(pkg_share, 'models', 'worlds', 'flat_world.sdf')
    models_root = os.path.join(pkg_share, 'models')  # model:// aramaları için

    use_gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    # Gazebo'nun model arama yoluna paketimizin models/ klasörünü ekleyelim
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_root + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # Dünyayı aç (model world içinde include edilecek)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        set_model_path,
        gz_sim,
        # NOT: ROS düğümlerini world açıldıktan sonra ayrı terminalde de başlatabilirsin,
        # ama istersen buraya tekrar ekleyebiliriz. Şimdilik param dosyası hatasını çözeceğiz.
    ])
