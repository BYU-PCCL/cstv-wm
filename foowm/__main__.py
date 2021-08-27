import argparse
import logging
import queue
import threading

import zmq

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
    help="set display layout ('fullscreen' (default), 'production')",
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
logger = logging.getLogger(__name__)


# # TODO: Should this be moved into its own file?
def messaging_loop(wm: FootronWindowManager):
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind("tcp://127.0.0.1:5557")

    while True:
        try:
            message = socket.recv_json()
            logging.debug(f"Received request: {message}")
            wm.message_queue.put(message)
        except Exception as e:
            logger.exception(e)

        while True:
            try:
                _ = wm.response_queue.get_nowait()
                socket.send_json({"status": "ok"})
            except queue.Empty:
                break


wm = FootronWindowManager(args.layout)
messaging_thread = threading.Thread(target=messaging_loop, args=(wm,))
messaging_thread.start()
wm.start()
