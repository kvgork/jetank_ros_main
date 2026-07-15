"""
Accessors for the JeTank cross-package topic contract (config/topics.yaml).

The contract file is the single source of truth for topic names shared
between packages (camera image streams, sock detections). Launch files in
this package import these helpers instead of hardcoding topic strings, so a
cross-package topic rename is a one-file edit (``config/topics.yaml``).

The consumer nodes (``jetank_detection``, ``jetank_web_control``) declare
the same values as parameter defaults; those remain the fallback when a
node is launched without the jetank_ros_main launch files.
"""

import os

from ament_index_python.packages import get_package_share_directory

import yaml


def contract_path():
    """Return the absolute path of the installed topics.yaml contract file."""
    return os.path.join(get_package_share_directory('jetank_ros_main'),
                        'config', 'topics.yaml')


def _load():
    """Parse topics.yaml and verify the repeated values stay consistent."""
    with open(contract_path(), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    det = data['sock_detector']['ros__parameters']
    cap = data['frame_capture']['ros__parameters']
    web = data['web_control_node']['ros__parameters']
    if cap['input_image_topic'] != det['input_image_topic']:
        raise ValueError(
            'topics.yaml: frame_capture.input_image_topic must match '
            'sock_detector.input_image_topic')
    if web['detections_topic'] != det['detections_topic']:
        raise ValueError(
            'topics.yaml: web_control_node.detections_topic must match '
            'sock_detector.detections_topic')
    if web['image_topic'] != det['input_image_topic'] + '/compressed':
        raise ValueError(
            'topics.yaml: web_control_node.image_topic must be the '
            'compressed transport of sock_detector.input_image_topic')
    return data


def node_topic_params(node_name):
    """Return the topic parameter dict for one node section of the contract."""
    return dict(_load()[node_name]['ros__parameters'])


def camera_left_raw():
    """Return the left camera raw image topic (sim and real robot)."""
    return _load()['sock_detector']['ros__parameters']['input_image_topic']


def camera_left_compressed():
    """Return the left camera compressed image topic (real robot stream)."""
    return _load()['web_control_node']['ros__parameters']['image_topic']


def camera_namespace():
    """Return the camera namespace (first segment of the raw image topic)."""
    return camera_left_raw().lstrip('/').split('/')[0]


def detections_socks():
    """Return the sock detection output topic (Detection2DArray)."""
    return _load()['sock_detector']['ros__parameters']['detections_topic']


def detections_socks_debug():
    """Return the annotated sock-detection debug image topic."""
    return _load()['sock_detector']['ros__parameters']['debug_image_topic']
