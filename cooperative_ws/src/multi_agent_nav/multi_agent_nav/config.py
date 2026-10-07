from pathlib import Path
import math
import yaml

def load_config(directory=None):
    directory = Path(directory) if directory else Path(__file__).resolve().parents[1] / 'config'
    if not directory.is_dir():
        from ament_index_python.packages import get_package_share_directory
        directory=Path(get_package_share_directory('multi_agent_nav'))/'config'
    return {name: yaml.safe_load((directory / f'{name}.yaml').read_text())
            for name in ('alignment', 'scene', 'robot_go2', 'robot_ranger')}

def live_gates(config):
    errors = []
    alignment, scene = config['alignment'], config['scene']
    if alignment.get('simulation') or scene.get('simulation'):
        errors.append('synthetic configuration cannot run live')
    if not alignment.get('validated') or not alignment.get('report'):
        errors.append('validated alignment report missing')
    else:
        from .calibration import fit_alignment
        report=alignment['report']
        try:
            recomputed=fit_alignment(*(report[k] for k in ('source_points','common_points','held_out_pairs','thresholds')))
            identities=report['identities']
            valid=recomputed['validated'] and all(
                identities[r]=={k:alignment['robots'][r][k] for k in ('local_map_id','map_version')} for r in ('go2','ranger'))
            transform=alignment['robots']['ranger']['transform']
            valid=valid and transform is not None and all(abs(a-b)<1e-8 for a,b in zip(transform,recomputed['transform']))
            if not valid:errors.append('alignment report/transform/map identities do not validate')
        except (KeyError,TypeError,ValueError):errors.append('alignment report cannot be independently recomputed')
    if not scene.get('enabled'):
        errors.append('scene disabled: field coordinates missing')
    for robot in ('go2', 'ranger'):
        entry = alignment['robots'][robot]
        if not entry.get('local_map_id') or not entry.get('map_version') or entry.get('transform') is None:
            errors.append(f'{robot}: map identity/transform missing')
        cfg = config[f'robot_{robot}']
        if not cfg.get('map_path'):
            errors.append(f'{robot}: map path missing')
    if not scene.get('resources') or not scene.get('exit_points'):
        errors.append('resource/exit geometry missing')
    go2=config['robot_go2']
    for flag in ('path_source_verified','camera_calibrated','peer_obstacle_verified','vision_validated'):
        if not go2.get(flag):errors.append(f'Go2 {flag} field verification required')
    if not go2.get('path_topic'):
        errors.append('executing Nav2 path source missing')
    if go2.get('stop_margin') is None or not math.isfinite(go2['stop_margin']) or go2['stop_margin']<0:
        errors.append('measured stop margin missing')
    if alignment['robots']['go2'].get('transform')!=[0.0,0.0,0.0]:
        errors.append('lab_ref must equal Go2 map numerically')
    return errors
