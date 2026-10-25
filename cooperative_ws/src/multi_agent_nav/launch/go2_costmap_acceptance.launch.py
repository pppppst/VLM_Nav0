"""Isolated static acceptance: costmaps only; no actions, velocity, AMCL or bridge."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vlm_nav.go2_nav2_config import load_nav2_parameters
from vlm_nav.go2_keepout import KEEP_OUT_NODES, configure_keepout

PREFIX='/go2_costmap_acceptance'

def build(context):
    vlm=Path(get_package_share_directory('vlm_nav'))
    coop=Path(get_package_share_directory('multi_agent_nav'))
    nav2=Path(get_package_share_directory('nav2_bringup'))
    params=load_nav2_parameters(nav2/'params/nav2_params.yaml',vlm/'config/nav2_go2.yaml',
        vlm/'config/nav2_go2_static_map.yaml',coop/'config/nav2_go2_cooperative_costmap.yaml')
    keepout=configure_keepout(params,LaunchConfiguration('keepout_mask',default='').perform(context))
    scoped={}
    names=[]
    for name in ('local_costmap','global_costmap'):
        p=params[name][name]['ros__parameters'];p['use_sim_time']=False
        full_name=PREFIX+'/'+name+'/'+name
        scoped[full_name]={'ros__parameters':p};names.append(full_name)
    if keepout:names=[*('/'+name for name in KEEP_OUT_NODES),*names]
    with tempfile.NamedTemporaryFile(mode='w',prefix='go2_costmap_acceptance_',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(scoped,f);parameter_file=f.name
    return [Node(package='multi_agent_nav',executable='multi_agent_costmap_acceptance',
        parameters=[parameter_file],output='screen'),
        *keepout,
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
            name='lifecycle_manager_costmaps',namespace=PREFIX,
            # Humble standalone Costmap2DROS does not create lifecycle bonds.
            # No motion server is present; consumers must reject stale payloads.
            parameters=[{'use_sim_time':False,'autostart':True,'bond_timeout':0.0,
                         'node_names':names}],output='screen')]

def generate_launch_description():
    vlm=Path(get_package_share_directory('vlm_nav'))
    return LaunchDescription([
        DeclareLaunchArgument('keepout_mask',default_value=''),
        SetEnvironmentVariable('ROS_DOMAIN_ID','0'),
        SetEnvironmentVariable('GO2_NET_IFACE','enp4s0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY','0'),
        SetEnvironmentVariable('CYCLONEDDS_URI','file://'+str(vlm/'config/cyclonedds_go2.xml')),
        OpaqueFunction(function=build)])
