import os
import sys


def ensure():
    if os.environ.get('DEMO_ROS_REEXEC') == '1':
        return
    prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    if '/ws/install' in prefix:
        return
    os.environ['DEMO_ROS_REEXEC'] = '1'
    script = sys.argv[0]
    args = ' '.join(_quote(arg) for arg in sys.argv[1:])
    command = (
        'source /opt/ros/jazzy/setup.bash && '
        'source /ws/install/setup.bash && '
        f'exec python3 {_quote(script)} {args}'
    )
    os.execvp('bash', ['bash', '-c', command])


def _quote(text):
    return "'" + text.replace("'", "'\\''") + "'"
