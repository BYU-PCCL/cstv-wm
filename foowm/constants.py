from .types import NetAtom, WmAtom, ClientType, DisplayLayout

WM_NAME = "foowm"

OFFSCREEN_SOURCE_WINDOW_NAME = "FOOTRON_SOURCE_WINDOW"
PLACARD_WINDOW_NAME = "FOOTRON_PLACARD"
LOADER_WINDOW_NAME = "FOOTRON_LOADING"
# A list of regex patterns for titles of windows that we want to dump offscreen
# because we don't have a more elegant way to get rid of them.
OFFSCREEN_HACK_WINDOW_PATTERNS = [
    # Chrome screen sharing notification
    r".*is sharing (your screen|a window)\.?"
]

UTF8_STRING_ATOM = "UTF8_STRING"
SUPPORTED_NET_ATOMS = [
    NetAtom.Supported,
    NetAtom.ActiveWindow,
    NetAtom.WmState,
    NetAtom.WmStateFullscreen,
    NetAtom.WmName,
    NetAtom.ClientList,
    NetAtom.WmCheck,
    NetAtom.WmWindowType,
    NetAtom.WmWindowTypeDock,
    NetAtom.WmWindowTypeToolbar,
    NetAtom.WmWindowTypeMenu,
    NetAtom.WmWindowTypeSplash,
    NetAtom.WmWindowTypeDialog,
    NetAtom.WmWindowTypeUtility,
    NetAtom.WmStateModal,
    NetAtom.WmStateAbove,
    NetAtom.WmStateSticky,
    NetAtom.WorkArea,
]
SUPPORTED_WM_ATOMS = [
    WmAtom.DeleteWindow,
    WmAtom.Protocols,
    WmAtom.Name,
    WmAtom.State,
    WmAtom.NormalHints,
]
FLOATING_WINDOW_TYPES = [
    NetAtom.WmWindowTypeDock,
    NetAtom.WmWindowTypeToolbar,
    NetAtom.WmWindowTypeUtility,
    NetAtom.WmWindowTypeDialog,
    NetAtom.WmWindowTypeMenu,
    NetAtom.WmWindowTypeSplash,
]
FLOATING_WINDOW_STATES = [
    NetAtom.WmStateModal,
    NetAtom.WmStateAbove,
    NetAtom.WmStateSticky,
]

PRODUCTION_WIDTH = 2736
PRODUCTION_HEIGHT = 1216
# Width required to make a 16:9 box
PRODUCTION_VIEWPORT_WIDTH = 2162
PRODUCTION_PLACARD_WIDTH = PRODUCTION_WIDTH - PRODUCTION_VIEWPORT_WIDTH

VIEWPORT_GEOMETRY = {
    DisplayLayout.Fullscreen: lambda *, width, height, **_: (0, 0, width, height),
    DisplayLayout.Center: lambda *, width, height, fullscreen, **_: (
        ((width - PRODUCTION_WIDTH) // 2)
        + (0 if fullscreen else PRODUCTION_PLACARD_WIDTH),
        (height - PRODUCTION_HEIGHT) // 2,
        PRODUCTION_WIDTH if fullscreen else PRODUCTION_VIEWPORT_WIDTH,
        PRODUCTION_HEIGHT,
    ),
    DisplayLayout.Production: lambda *, fullscreen, **_: (
        0 if fullscreen else PRODUCTION_PLACARD_WIDTH,
        0,
        PRODUCTION_WIDTH if fullscreen else PRODUCTION_VIEWPORT_WIDTH,
        PRODUCTION_HEIGHT,
    ),
}

LAYOUT_GEOMETRY = {
    ClientType.Placard: {
        DisplayLayout.Fullscreen: lambda *, width, height, **_: (
            0,
            0,
            int(width * 0.2),
            height,
        ),
        DisplayLayout.Center: lambda *, width, height, **_: (
            (width - PRODUCTION_WIDTH) // 2,
            (height - PRODUCTION_HEIGHT) // 2,
            PRODUCTION_PLACARD_WIDTH,
            PRODUCTION_HEIGHT,
        ),
        DisplayLayout.Production: (0, 0, PRODUCTION_PLACARD_WIDTH, PRODUCTION_HEIGHT),
    },
    ClientType.Loader: VIEWPORT_GEOMETRY,
    ClientType.OffscreenSource: lambda *, width, geometry, **_: (
        width,
        0,
        geometry.width,
        geometry.height,
    ),
    ClientType.OffscreenHack: lambda *, height, geometry, **_: (
        0,
        height,
        geometry.width,
        geometry.height,
    ),
    None: VIEWPORT_GEOMETRY,
}
