"""Select which composable nodes the back-camera container hosts.

``USB_CAM_CONTAINER_COMPONENTS`` chooses the mix:

  camera                    only ``usb_cam::UsbCamNode``
  camera+stream             ``usb_cam`` + ``cpp_video_stream::ImageStreamNode``
  camera+stream+apriltag    all three (default: previous behaviour)

``+`` or ``,`` may separate the tokens and a few aliases are accepted
(``usb_cam`` for ``camera``, ``cpp_stream`` for ``stream``, ...).  ``camera``
is mandatory: without it the container would not publish any image.

Keeping the three nodes in one container is the deployed layout, but the
recorder and the charger detector are optional on robots that do not need
them; this switch avoids starting nodes that would just idle.
"""

import os

ENV_NAME = 'USB_CAM_CONTAINER_COMPONENTS'
DEFAULT = 'camera+stream+apriltag'

_ORDER = ('camera', 'stream', 'apriltag')
_ALIASES = {
    'usb_cam': 'camera',
    'cpp_stream': 'stream',
    'cpp_video_stream': 'stream',
    'image_stream': 'stream',
    'apriltag_ros': 'apriltag',
}


def selected_components(raw=None):
    """Return the enabled components in canonical order.

    Args:
        raw: explicit value (mainly for tests); defaults to the environment.

    Raises:
        ValueError: empty value, unknown token, or missing ``camera``.
    """
    value = raw if raw is not None else os.environ.get(ENV_NAME, DEFAULT)
    tokens = [token.strip().lower()
              for token in value.replace('+', ',').split(',') if token.strip()]
    if not tokens:
        raise ValueError(f'{ENV_NAME} is empty; expected e.g. {DEFAULT}')

    selected = set()
    for token in tokens:
        component = _ALIASES.get(token, token)
        if component not in _ORDER:
            raise ValueError(
                f'{ENV_NAME}={value!r}: unknown component {token!r}; expected a '
                f'combination of {", ".join(_ORDER)}')
        selected.add(component)

    if 'camera' not in selected:
        raise ValueError(f'{ENV_NAME}={value!r} must include "camera"')

    return [component for component in _ORDER if component in selected]
