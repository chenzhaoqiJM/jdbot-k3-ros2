import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = False  # 是否使用仿真时间，真实机器人环境下通常为 False

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,  # 是否使用 /clock 时间源
            'frame_id': 'base_link',  # RTAB-Map 处理传感器数据时使用的基准坐标系，这里与雷达点云 frame_id 保持一致
            'odom_frame_id': 'odom',  # 里程计坐标系名称，RTAB-Map 会用它与 map 建立关系
            'map_frame_id': 'map',  # 全局地图坐标系名称
            'subscribe_depth': False,  # 不订阅深度图
            'subscribe_rgb': False,  # 不订阅 RGB 图像
            'subscribe_scan': False,  # 不订阅 2D LaserScan
            'subscribe_scan_cloud': True,  # 订阅 3D 点云形式的激光数据（PointCloud2）
            'subscribe_odom_info': False,  # 不订阅额外的里程计调试/信息话题
            'approx_sync': True,  # 使用近似时间同步，允许 odom 与点云时间戳有小范围误差
            'queue_size': 30,  # 消息同步队列长度，适当增大可减少因时间抖动导致的不同步
            'wait_for_transform': 0.2,  # 等待 TF 变换的超时时间，单位秒
            'Reg/Strategy': '1',  # 配准策略，1 表示使用 ICP 类方法，适合激光/点云配准
            'Icp/PointToPlane': 'false',  # ICP 是否使用点到平面误差，这里关闭，使用点到点方式更稳妥
            'Icp/VoxelSize': '0.1',  # ICP 前点云体素降采样尺寸，单位米
            'Icp/MaxCorrespondenceDistance': '1.0',  # ICP 最近邻匹配的最大对应点距离，单位米
            'Grid/FromDepth': 'false',  # 不从深度图生成栅格地图
            'Grid/3D': 'true',  # 启用 3D 栅格/三维地图构建
            'Grid/RangeMax': '20.0',  # 建图时使用的最大点云距离，超出该距离的点忽略，单位米
            'Grid/CellSize': '0.1',  # 栅格地图分辨率，单位米
            'Mem/IncrementalMemory': 'true',  # 启用增量建图，持续向地图中加入新节点
            'Mem/InitWMWithAllNodes': 'false',  # 启动时不把所有节点一次性加载到工作内存中
            'RGBD/NeighborLinkRefining': 'true',  # 对相邻节点之间的约束进行细化，提高局部一致性
            'RGBD/ProximityBySpace': 'true',  # 依据空间接近关系尝试建立闭环/近邻约束
            'RGBD/AngularUpdate': '0.05',  # 角度变化超过该阈值才触发一次更新，单位弧度
            'RGBD/LinearUpdate': '0.05',  # 线位移超过该阈值才触发一次更新，单位米
            'Rtabmap/DetectionRate': '10.0',  # RTAB-Map 主循环的目标处理频率，单位 Hz
            'Optimizer/GravitySigma': '0',  # 重力约束权重，0 表示不使用 IMU 重力方向约束
            'publish_tf': True,  # 是否发布 map->odom 的 TF 变换
        }],
        remappings=[
            ('odom', '/odom'),  # 将 RTAB-Map 内部订阅的 odom 重映射到系统中的 /odom
            ('scan_cloud', '/lidar_points'),  # 将 RTAB-Map 的 scan_cloud 输入重映射到 Hesai 输出点云话题
        ],
        arguments=['-d'],  # 调试模式启动，通常会输出更多日志并使用数据库模式
    )

    return LaunchDescription([
        rtabmap,
    ])
