"""
Lease files between main.py and the VIO process: main.py touches a file in the telemetry directory while a
client wants something (a stream, the features, the pose, the status); the VIO process polls the file's age and
publishes only while it is fresh. depthai-free.
"""

from pathlib import Path


def fresh(path, now, lease_s):
    """True while the lease file was touched within lease_s of now; False if it does not exist."""
    try:
        return now - Path(path).stat().st_mtime <= lease_s
    except OSError:
        return False


def touch(directory, name):
    """Renew the lease file `name` in `directory` (created if missing); errors are ignored."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / name).touch()
    except OSError:
        pass
