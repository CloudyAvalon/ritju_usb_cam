"""Read committed GPU camera parameters, including reset-to-default snapshots."""

import hashlib
import os
from pathlib import Path
import time

import yaml


def get_dynamic_parameter_file():
    config_dir = os.getenv('CAPELLA_CORE_CONFIG_DIR', os.getenv('CAPELLA_DATA_DIR', '/data'))
    return os.getenv('CAPELLA_RUNTIME_PARAMS_FILE',
                     str(Path(config_dir) / 'generated/agent_overrides_params.yaml'))


def get_node_parameters(node_name):
    """Keep standalone defaults, but require any explicitly requested snapshot."""
    path = Path(get_dynamic_parameter_file())
    if 'CAPELLA_RUNTIME_PARAMS_FILE' not in os.environ and not path.exists():
        return {}
    timeout = max(0.0, float(os.getenv('CAPELLA_RUNTIME_PARAMS_WAIT_TIMEOUT', '0')))
    deadline = time.monotonic() + timeout
    while True:
        try:
            manifest_path = path.with_name(path.name + '.manifest')
            manifest_bytes = manifest_path.read_bytes()
            manifest = yaml.safe_load(manifest_bytes)
            contents = path.read_bytes()
            if hashlib.sha256(contents).hexdigest() != manifest['overrides_sha256']:
                raise ValueError('manifest does not match overrides')
            runtime_hash = manifest.get('runtime_sha256')
            if runtime_hash:
                runtime = Path(manifest.get('runtime_file') or path.with_name('runtime_params.yaml'))
                contents = runtime.read_bytes()
                if hashlib.sha256(contents).hexdigest() != runtime_hash:
                    raise ValueError('runtime snapshot does not match manifest')
            document = yaml.safe_load(contents)
            if not isinstance(document, dict):
                raise ValueError('snapshot must be a mapping')
            if manifest_path.read_bytes() != manifest_bytes:
                raise ValueError('manifest changed during read')
            node = document.get(node_name, document.get('/' + node_name, {}))
            parameters = node.get('ros__parameters', {})
            if not isinstance(parameters, dict):
                raise ValueError('node parameters must be a mapping')
            return parameters
        except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as exc:
            if time.monotonic() >= deadline:
                raise RuntimeError(f'dynamic parameters unavailable: {exc}') from exc
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
