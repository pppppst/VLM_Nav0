from pathlib import Path
import yaml

def load_config(directory=None):
    directory = Path(directory) if directory else Path(__file__).resolve().parents[1] / 'config'
    return {name: yaml.safe_load((directory / f'{name}.yaml').read_text())
            for name in ('alignment', 'scene', 'robot_go2', 'robot_ranger')}

def live_gates(config):
    errors = []
    alignment, scene = config['alignment'], config['scene']
    if alignment.get('simulation') or scene.get('simulation'):
        errors.append('synthetic configuration cannot run live')
    if not alignment.get('validated') or not alignment.get('report'):
        errors.append('validated alignment report missing')
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
    return errors
