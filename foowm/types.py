import dataclasses
import enum
from typing import Optional

from Xlib import Xutil
from Xlib.protocol import rq
from Xlib.xobject import icccm
from Xlib.xobject.drawable import Window


class ClientType(enum.Enum):
    Placard = enum.auto()
    # A developer-defined "offscreen source" provides a video source to be composed into
    # a visible window, like a browser. This behavior enables a use case where a
    # developer wants to display a live GPU/CUDA-generated visualization alongside UI
    # elements using the layout/styling capabilities of a web browser.
    OffscreenSource = enum.auto()
    # Things we pile up offscreen because we don't want to see them.
    # @vinhowe: I call this a "hack" because we should try to eliminate all cases where
    # we need this behavior. Matching titles for windows we don't we control is fraught
    # because all it takes is an application update for this hack to stop working.
    OffscreenHack = enum.auto()


class DisplayLayout(enum.Enum):
    Fullscreen = "fullscreen"
    Production = "production"


@dataclasses.dataclass
class WindowGeometry:
    x: int
    y: int
    width: int
    height: int


@dataclasses.dataclass
class Client:
    window: Window
    parent: Window
    geometry: WindowGeometry
    desired_geometry: WindowGeometry
    title: str
    type: Optional[ClientType]
    floating: bool


# Apparently python-xlib won't give us access to x and y on the standard
# WMNormalHints struct. Despite the deprecation notice on XSizeHints in Xutil.h, some
# windows--like Chromium's "* is sharing your screen" floating notification bar--use x
# and y to position themselves absolutely.
ExtendedWMNormalHints = rq.Struct(
    rq.Card32("flags"),
    rq.Int32("x", default=0),
    rq.Int32("y", default=0),
    rq.Int32("width", default=0),
    rq.Int32("height", default=0),
    rq.Int32("min_width", default=0),
    rq.Int32("min_height", default=0),
    rq.Int32("max_width", default=0),
    rq.Int32("max_height", default=0),
    rq.Int32("width_inc", default=0),
    rq.Int32("height_inc", default=0),
    rq.Object("min_aspect", icccm.Aspect, default=(0, 0)),
    rq.Object("max_aspect", icccm.Aspect, default=(0, 0)),
    rq.Int32("base_width", default=0),
    rq.Int32("base_height", default=0),
    rq.Int32("win_gravity", default=0),
)

# We put this here instead of constants because it relates directly to the type above
# and Xlib is probably never going to change it--constants.py defines constants for our
# application specifically
WM_NORMAL_HINTS_FIELD_BITMASK_MAP = {
    "user-specified": {
        Xutil.USPosition: ["x", "y"],
        Xutil.USSize: ["width", "height"],
    },
    "program-specified": {
        Xutil.PPosition: ["x", "y"],
        Xutil.PSize: ["width", "height"],
        Xutil.PMinSize: ["min_width", "min_height"],
        Xutil.PMaxSize: ["max_width", "max_height"],
        Xutil.PResizeInc: ["width_inc", "height_inc"],
        Xutil.PAspect: ["min_aspect", "max_aspect"],
        Xutil.PBaseSize: ["base_width", "base_height"],
        Xutil.PWinGravity: ["win_gravity"],
    },
}


# Using classes here because I'm suspicious that Python enums are slow
# https://stackoverflow.com/a/35541980 (we should profile to confirm this is actually
# a problem, though)
class NetAtom:
    """This class contains string keys for all EWMH atoms berry supports, with a few
    extras we need. The subset of these atoms that we support is defined in
    SUPPORTED_NET_ATOMS in .constants"""

    Supported = "_NET_SUPPORTED"
    NumberOfDesktops = "_NET_NUMBER_OF_DESKTOPS"
    ActiveWindow = "_NET_ACTIVE_WINDOW"
    WmStateFullscreen = "_NET_WM_STATE_FULLSCREEN"
    CurrentDesktop = "_NET_CURRENT_DESKTOP"
    ClientList = "_NET_CLIENT_LIST"
    WmCheck = "_NET_SUPPORTING_WM_CHECK"
    WmState = "_NET_WM_STATE"
    WmName = "_NET_WM_NAME"
    WmWindowType = "_NET_WM_WINDOW_TYPE"
    WmWindowTypeMenu = "_NET_WM_WINDOW_TYPE_MENU"
    WmWindowTypeDock = "_NET_WM_WINDOW_TYPE_DOCK"
    WmWindowTypeToolbar = "_NET_WM_WINDOW_TYPE_TOOLBAR"
    WmWindowTypeDialog = "_NET_WM_WINDOW_TYPE_DIALOG"
    WmWindowTypeUtility = "_NET_WM_WINDOW_TYPE_UTILITY"
    WmWindowTypeSplash = "_NET_WM_WINDOW_TYPE_SPLASH"
    WmStateModal = "_NET_WM_STATE_MODAL"
    WmStateAbove = "_NET_WM_STATE_ABOVE"
    WmStateSticky = "_NET_WM_STATE_STICKY"
    WmDesktop = "_NET_WM_DESKTOP"
    WmMoveResize = "_NET_MOVERESIZE_WINDOW"
    WmFrameExtents = "_NET_FRAME_EXTENTS"
    DesktopNames = "_NET_DESKTOP_NAMES"
    DesktopViewport = "_NET_DESKTOP_VIEWPORT"
    WorkArea = "_NET_WORKAREA"


class WmAtom:
    DeleteWindow = "WM_DELETE_WINDOW"
    Protocols = "WM_TAKE_FOCUS"
    TakeFocus = "WM_PROTOCOLS"
    Name = "WM_NAME"
    State = "WM_STATE"
    NormalHints = "WM_NORMAL_HINTS"
