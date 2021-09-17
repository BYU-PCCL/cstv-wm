from .types import NetAtom, WmAtom, ClientType, DisplayScenario, DisplayLayout

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

DEFAULT_CLEAR_TYPES = [
    ClientType.OffscreenHack,
    ClientType.OffscreenSource,
    ClientType.Loader,
    None,
]

PRODUCTION_DISPLAY_WIDTH = 2736
# Width required to make a 16:9 box
PRODUCTION_HD_WIDTH = 2162
PRODUCTION_WIDE_WIDTH = 2392

PRODUCTION_HEIGHT = 1216
PRODUCTION_PLACARD_WIDTH = PRODUCTION_DISPLAY_WIDTH - PRODUCTION_HD_WIDTH

PRODUCTION_LAYOUT_WIDTHS = {
    DisplayLayout.Hd: PRODUCTION_HD_WIDTH,
    DisplayLayout.Wide: PRODUCTION_WIDE_WIDTH,
    DisplayLayout.Full: PRODUCTION_DISPLAY_WIDTH,
}

PRODUCTION_PLACARD_OFFSETS = {
    DisplayLayout.Hd: PRODUCTION_PLACARD_WIDTH,
    DisplayLayout.Wide: 344,
    DisplayLayout.Full: 0,
}

VIEWPORT_GEOMETRY = {
    DisplayScenario.Fullscreen: lambda *, width, height, **_: (0, 0, width, height),
    DisplayScenario.Center: lambda *, width, height, layout, **_: (
        ((width - PRODUCTION_DISPLAY_WIDTH) // 2)
        + (PRODUCTION_PLACARD_OFFSETS[layout]),
        (height - PRODUCTION_HEIGHT) // 2,
        PRODUCTION_LAYOUT_WIDTHS[layout],
        PRODUCTION_HEIGHT,
    ),
    DisplayScenario.Production: lambda *, layout, **_: (
        0 if layout else PRODUCTION_PLACARD_WIDTH,
        0,
        PRODUCTION_LAYOUT_WIDTHS[layout],
        PRODUCTION_HEIGHT,
    ),
}

LAYOUT_GEOMETRY = {
    ClientType.Placard: {
        DisplayScenario.Fullscreen: lambda *, width, height, **_: (
            0,
            0,
            int(width * 0.2),
            height,
        ),
        DisplayScenario.Center: lambda *, width, height, **_: (
            (width - PRODUCTION_DISPLAY_WIDTH) // 2,
            (height - PRODUCTION_HEIGHT) // 2,
            PRODUCTION_PLACARD_WIDTH,
            PRODUCTION_HEIGHT,
        ),
        DisplayScenario.Production: (0, 0, PRODUCTION_PLACARD_WIDTH, PRODUCTION_HEIGHT),
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
