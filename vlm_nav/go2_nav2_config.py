"""Use the same effective parameter order for Nav2 and isolated costmap acceptance."""
from pathlib import Path
import yaml


def deep_merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            deep_merge(destination[key], value)
        else:
            destination[key] = value


def load_nav2_parameters(default_file, overrides_file, amcl_file=None, costmap_file=None):
    params = {}
    for path in (default_file, overrides_file, amcl_file, costmap_file):
        if not path:
            continue
        data = yaml.safe_load(Path(path).read_text())
        if path == costmap_file and set(data) - {'global_costmap', 'local_costmap'}:
            raise ValueError('final costmap overlay may only change costmap nodes')
        deep_merge(params, data)
    return params
