import argparse
import logging

from .types import DisplayLayout
from .wm import FootronWindowManager


def _log_level(arg):
    level = getattr(logging, arg.upper(), None)
    if level is None:
        raise ValueError(f"Invalid log level '{arg}'")
    return level


parser = argparse.ArgumentParser()
parser.add_argument(
    "--layout",
    help="set display layout ('fullscreen' (default), 'fit4k', 'production')",
    type=DisplayLayout,
    default=DisplayLayout.Fullscreen,
)

log_level_group = parser.add_mutually_exclusive_group()
log_level_group.add_argument(
    "--level",
    help="set log level ('debug', 'info' (default), 'warning', 'error', 'critical')",
    type=_log_level,
)
log_level_group.add_argument(
    "-v",
    help="set log level to verbose",
    action="store_const",
    const=logging.DEBUG,
)

args = parser.parse_args()

logging.basicConfig(level=args.v or args.level or logging.INFO)

wm = FootronWindowManager(args.layout)
wm.start()
