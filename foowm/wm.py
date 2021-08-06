import logging
import re
from typing import Dict, Callable, Optional, Any

from Xlib.display import Display
from Xlib import X, error, Xatom, Xutil
from Xlib.protocol.display import Screen
from Xlib.protocol import event
from Xlib.xobject.drawable import Window

from .constants import (
    WM_NAME,
    OFFSCREEN_SOURCE_WINDOW_NAME,
    PLACARD_WINDOW_NAME,
    OFFSCREEN_HACK_WINDOW_PATTERNS,
    UTF8_STRING_ATOM,
    SUPPORTED_NET_ATOMS,
    SUPPORTED_WM_ATOMS,
    FLOATING_WINDOW_TYPES,
    LAYOUT_GEOMETRY,
    FLOATING_WINDOW_STATES,
)
from .types import (
    Client,
    ClientType,
    WindowGeometry,
    NetAtom,
    WmAtom,
    ExtendedWMNormalHints,
    DisplayLayout,
)
from .util import debug_log_size_hints, debug_log_window_geometry, debug_value_change

logger = logging.getLogger(WM_NAME)


class FootronWindowManager:
    _display: Display
    _screen: Screen
    _root: Window
    _check: Window
    _event_handlers: Dict[int, Callable[[Any], None]]
    _net_atoms: Dict[str, int]
    _wm_atoms: Dict[str, int]
    _utf8_atom: int
    _width: int
    _height: int

    _debug_logging: bool

    _display_layout: DisplayLayout
    _placard: Optional[Client]
    _clients: Dict[int, Client]

    def __init__(self, display_layout: DisplayLayout):
        self._net_atoms = {}
        self._wm_atoms = {}
        self._event_handlers = {
            X.MapRequest: self._handle_map_request,
            X.UnmapNotify: self._handle_unmap_notify,
            X.ConfigureNotify: self._handle_configure_notify,
            X.ConfigureRequest: self._handle_configure_request,
            X.PropertyNotify: self._handle_property_notify,
            X.EnterNotify: self._handle_enter_notify,
        }

        self._debug_logging = logging.root.level < logging.INFO

        self._display_layout = display_layout
        self._placard = None
        self._clients = {}

    def start(self):
        self._setup()
        self._loop()

    @property
    def clients(self):
        return self._clients

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def placard(self):
        return self._placard

    def _setup(self):
        logger.debug("Starting setup")

        # Based on berry's setup method in wm.c (
        # https://github.com/JLErvin/berry/blob/master/wm.c) except that we don't
        # handle input or focus because our display doesn't support mouse or touch
        # input
        self._display = Display()
        self._screen = self._display.screen()
        self._root = self._screen.root
        self._width = self._screen.width_in_pixels
        self._height = self._screen.height_in_pixels

        self._root.change_attributes(
            event_mask=X.StructureNotifyMask
            | X.SubstructureRedirectMask
            | X.SubstructureNotifyMask
            | X.ButtonPressMask
        )

        # Presumably X.CopyFromParent here will set this window to use root's depth,
        # which seems fine. If we have an issue with this somehow, we might need to
        # mess with the depth parameter, because I just guessed.
        self._check = self._root.create_window(0, 0, 1, 1, 0, X.CopyFromParent)

        self._utf8_atom = self._display.intern_atom(UTF8_STRING_ATOM)
        for atom_str in SUPPORTED_NET_ATOMS:
            self._net_atoms[atom_str] = self._display.intern_atom(atom_str)
        for atom_str in SUPPORTED_WM_ATOMS:
            self._wm_atoms[atom_str] = self._display.intern_atom(atom_str)

        self._check.change_property(
            self._net_atoms[NetAtom.WmCheck], Xatom.WINDOW, 32, [self._check.id]
        )
        self._check.change_property(
            self._net_atoms[NetAtom.WmName],
            Xatom.STRING,
            8,
            WM_NAME.encode("utf-8"),
        )
        self._root.change_property(
            self._net_atoms[NetAtom.WmCheck], Xatom.WINDOW, 32, [self._check.id]
        )
        # Set supported EWMH atoms
        self._root.change_property(
            self._net_atoms[NetAtom.Supported],
            Xatom.ATOM,
            32,
            list(self._net_atoms.values()),
        )

        logger.debug("Setup finished")

    def _raise_placard(self):
        if not self._placard:
            return

        logger.debug("Raising placard...")
        self._placard.window.raise_window()
        self._display.sync()

    def _handle_map_request(self, ev: event.MapRequest):
        # Background on mapping and unmapping (last paragraph):
        # https://magcius.github.io/xplain/article/x-basics.html#lets-go
        logger.debug(f"Handling MapRequest event for window with ID {ev.window.id}")

        try:
            attrs = ev.window.get_attributes()
        except error.XError:
            logger.exception(
                f"Error attempting to get attributes for window {hex(ev.window.id)}"
            )
            return

        if not attrs or attrs.override_redirect:
            return

        self._manage_new_window(ev.window)

    def _handle_unmap_notify(self, ev: event.UnmapNotify):
        window_id = ev.window.id
        logger.debug(f"Handling UnmapNotify event for window {hex(window_id)}")

        try:
            client = self._clients[window_id]
        except KeyError:
            logger.debug(
                f"No client found for UnmapNotify event with ID {hex(ev.window.id)}"
            )
            return

        if client.type == ClientType.Placard:
            logger.info("Placard window is closing")
            self._placard = None

        del self._clients[window_id]
        self._set_ewmh_clients_list()

    def _handle_configure_notify(self, ev: event.ConfigureNotify):
        logger.debug(f"Handling ConfigureNotify event for window {hex(ev.window.id)}")

        if ev.window.id == self._root.id:
            old_dimensions = (self._width, self._height)
            self._width = ev.width
            self._height = ev.height
            # TODO: If we're going to handle this for real, we should really update all
            #  of our managed non-floating windows.
            #  @vinhowe: But I can't imagine that we we'll get a lot of screen dimension
            #  updates we need to handle "hot" in production.

            if self._debug_logging:
                logger.debug("Resizing root window:")
                debug_value_change(
                    logger.debug,
                    "width, height",
                    old_dimensions,
                    (self._width, self._height),
                )

        # TODO: If we ever need more involved multi-display handling, this would be
        #  the place to do it

    def _handle_configure_request(self, ev: event.ConfigureRequest):
        logger.debug(f"Handling ConfigureRequest event for window {hex(ev.window.id)}")

        try:
            client = self._clients[ev.window.id]
        except KeyError:
            logger.debug(
                f"No client found for ConfigureRequest event with ID {hex(ev.window.id)}"
            )
            return

        # We don't let the window types that we manage set their own geometry
        # directly, but we do want to allow offscreen source windows to set their own
        # dimensions, if not position.
        desired_geometry = WindowGeometry(ev.x, ev.y, ev.width, ev.height)
        client.desired_geometry = desired_geometry
        client.geometry = self._client_geometry(
            desired_geometry, client.type, client.floating
        )

        self.scale_client(client, client.geometry)
        self._raise_placard()

    def _handle_property_notify(self, ev: event.PropertyNotify):
        logger.debug(f"Handling PropertyNotify event for window {hex(ev.window.id)}")

        try:
            client = self._clients[ev.window.id]
        except KeyError:
            logger.debug(
                f"No client found for PropertyNotify event with ID {hex(ev.window.id)}"
            )
            return

        if ev.state == X.PropertyDelete:
            # Not sure why berry doesn't handle PropertyDelete
            return

        if ev.atom in [self._net_atoms[NetAtom.WmName], self._wm_atoms[WmAtom.Name]]:
            logger.debug(f"Handling title update on window {hex(client.window.id)}:")

            old_title = client.title
            client.title = self._window_title(client.window)
            if self._debug_logging:
                debug_value_change(logger.debug, "title", old_title, client.title)

            old_type = client.type
            client.type = self._client_type_from_title(client.title)
            if self._debug_logging:
                debug_value_change(logger.debug, "type", old_type, client.type)

            if client.type == ClientType.Placard:
                logger.info("Matched existing window as placard")
                self._placard = client

            client.geometry = self._client_geometry(
                client.desired_geometry, client.type, client.floating
            )
            self.scale_client(client, client.geometry)
            return

        if ev.atom == self._wm_atoms[WmAtom.NormalHints]:
            wm_normal_hints = self._extended_wm_normal_hints(client.window)
            if not wm_normal_hints:
                return

            if self._debug_logging:
                logger.debug(
                    f"Received size hints update on window {hex(client.window.id)}:"
                )
                debug_log_size_hints(logger.debug, wm_normal_hints)
                client.desired_geometry = WindowGeometry(
                    wm_normal_hints.x,
                    wm_normal_hints.y,
                    wm_normal_hints.max_width,
                    wm_normal_hints.max_height,
                )
            return

    def _handle_enter_notify(self, ev: event.EnterNotify):
        logger.debug(f"Handling EnterNotify event for window {hex(ev.window.id)}")
        # Handling EnterNotify events seems to let us use popup menus in Chrome,
        # etc., which is useful at least for debugging
        self._root.change_property(
            self._net_atoms[NetAtom.ActiveWindow], Xatom.WINDOW, 32, [ev.window.id]
        )
        ev.window.set_input_focus(X.RevertToPointerRoot, X.CurrentTime)

    def _manage_new_window(self, window: Window):
        if window.id in self._clients:
            # TODO: Add log statement here
            # Just ignore any map requests for existing clients
            return

        window_type = window.get_property(
            self._net_atoms[NetAtom.WmWindowType], Xatom.ATOM, 0, 32
        )
        # TODO: We could probably refactor these two similar
        #  `if (condition) then floating = True` blocks into one cleaner block
        floating = False
        if window_type:
            atom_id = window_type.value[0]
            if atom_id in [self._net_atoms[atom] for atom in FLOATING_WINDOW_TYPES]:
                # TODO: Add log statement here
                logger.debug(
                    f"_NET_WM_WINDOW_TYPE on new window {hex(window.id)} matches a floating window type"
                )
                floating = True
                # An alternative is just not managing these window types with the
                # following code. The problem is that they'll all just show up in the
                # upper left hand corner of the screen, which is hardly desirable.

                # window.map()
                # return

        ewmh_state = window.get_property(
            self._net_atoms[NetAtom.WmState], Xatom.ATOM, 0, 32
        )
        if ewmh_state and any(
            self._net_atoms[atom] in ewmh_state.value
            # TODO: Add other states that would qualify here
            for atom in FLOATING_WINDOW_STATES
        ):
            logger.debug(
                f"_NET_WM_STATE on new window {hex(window.id)} matches a floating window state"
            )
            floating = True

        wm_normal_hints = self._extended_wm_normal_hints(window)
        normal_hints_geometry = None
        # Some special window types will try to position themselves by using these
        # sizing hints, which is fine as long as the window identifies itself as a
        # floating window somehow
        if wm_normal_hints:
            if self._debug_logging:
                logger.debug(f"Found size hints on new window {hex(window.id)}:")
                debug_log_size_hints(logger.debug, wm_normal_hints)
                normal_hints_geometry = WindowGeometry(
                    wm_normal_hints.x,
                    wm_normal_hints.y,
                    wm_normal_hints.max_width,
                    wm_normal_hints.max_height,
                )

        # @vinhowe: At this point, berry gets class hint information from the window,
        # but it's unclear to me that it ever does anything with that information

        try:
            x_geometry = window.get_geometry()
        except error.BadDrawable:
            logger.exception(
                f"Error while attempting to get window geometry for new window {hex(window.id)}"
            )
            return

        # TODO: Should we be doing error handling here (map has onerror arg, set to
        #  None by default)?
        window.map()
        self._raise_placard()
        window.change_attributes(
            event_mask=X.EnterWindowMask
            | X.FocusChangeMask
            | X.PropertyChangeMask
            | X.StructureNotifyMask
        )

        # This sets WM_STATE on our managed client to NormalState, which is required
        # by Chromium for a window to be registered as a capture source on Linux:
        # https://source.chromium.org/chromium/_/webrtc/src.git/+/a1aa9d732cf08644a898dd9d93fc50c849cd83d4:modules/desktop_capture/linux/window_list_utils.cc;l=43-45,48-50
        # berry doesn't set this property for some reason, but dwm (which berry looks
        # to be based on) does.
        window.change_property(
            self._wm_atoms[WmAtom.State],
            self._wm_atoms[WmAtom.State],
            32,
            [Xutil.NormalState],
        )

        title = self._window_title(window)
        if title:
            logger.debug(f"Title for new window {hex(window.id)} is {title}")
        else:
            logger.debug(f"No title for new window {hex(window.id)}")

        client_type = FootronWindowManager._client_type_from_title(title)
        logger.debug(f"Client type for new window {hex(window.id)} is {client_type}")

        # If a window is matched with a specific type, we don't want to let it set
        # its own dimensions
        floating = floating and client_type is None
        logger.debug(
            f"Floating state for new window {hex(window.id)}: {str(floating).lower()}"
        )

        desired_geometry = (
            normal_hints_geometry
            if normal_hints_geometry
            else WindowGeometry(
                x_geometry.x, x_geometry.y, x_geometry.width, x_geometry.height
            )
        )
        if self._debug_logging:
            logger.debug(f"Desired geometry for new window {hex(window.id)}:")
            debug_log_window_geometry(logger.debug, desired_geometry)

        geometry = self._client_geometry(desired_geometry, client_type, floating)
        if self._debug_logging:
            logger.debug(f"Actual geometry for new window {hex(window.id)}:")
            debug_log_window_geometry(logger.debug, geometry)

        client = Client(
            window,
            geometry,
            desired_geometry,
            title,
            client_type,
            floating,
        )

        if client_type == ClientType.Placard:
            logger.info("Matched new placard window")
            self._placard = client

        self.scale_client(client, client.geometry)
        self._clients[client.window.id] = client
        self._set_ewmh_clients_list()

    @staticmethod
    def _client_type_from_title(title: str) -> Optional[ClientType]:
        if not title:
            return None

        if any(re.match(pattern, title) for pattern in OFFSCREEN_HACK_WINDOW_PATTERNS):
            return ClientType.OffscreenHack

        if OFFSCREEN_SOURCE_WINDOW_NAME in title:
            return ClientType.OffscreenSource

        if PLACARD_WINDOW_NAME in title:
            return ClientType.Placard

        return None

    @staticmethod
    def _extended_wm_normal_hints(window: Window):
        # noinspection PyProtectedMember
        # Again, as stated in the comment above ExtendedWMNormalHints, we build our
        # own struct because we want access to x and y
        return window._get_struct_prop(
            Xatom.WM_NORMAL_HINTS, Xatom.WM_SIZE_HINTS, ExtendedWMNormalHints
        )

    def _client_geometry(
        self,
        desired_geometry: WindowGeometry,
        client_type: Optional[ClientType],
        floating=False,
    ):
        if client_type is None and floating:
            return desired_geometry

        geometry_factory = LAYOUT_GEOMETRY[client_type]

        if isinstance(geometry_factory, dict):
            geometry_factory = geometry_factory[self._display_layout]

        if isinstance(geometry_factory, tuple):
            return WindowGeometry(*geometry_factory)

        return WindowGeometry(
            *geometry_factory(
                width=self._width, height=self._height, geometry=desired_geometry
            )
        )

    def _set_ewmh_clients_list(self):
        new_clients = self._clients.keys()
        logger.debug(
            f"Updating _NET_CLIENT_LIST on root window: {list(map(hex, new_clients))}"
        )

        self._root.change_property(
            self._net_atoms[NetAtom.ClientList], Xatom.WINDOW, 32, list(new_clients)
        )

    def scale_client(self, client, geometry: WindowGeometry):
        if self._debug_logging:
            logger.debug(
                f"Attempting to scale window {hex(client.window.id)} to new geometry:"
            )
            debug_log_window_geometry(logger.debug, geometry)

        try:
            client.window.configure(
                x=geometry.x,
                y=geometry.y,
                width=geometry.width,
                height=geometry.height,
            )
            self._display.sync()
        except error.XError:
            logger.exception(f"Error while scaling client {hex(client.window.id)}")

    def _window_title(self, window: Window):
        """Simplify dealing with _NET_WM_NAME (UTF-8) vs. WM_NAME (legacy)"""
        try:
            for atom in (self._net_atoms[NetAtom.WmName], self._wm_atoms[WmAtom.Name]):
                try:
                    title = window.get_full_property(atom, 0)
                except UnicodeDecodeError:
                    title = None
                else:
                    if title:
                        title = title.value
                        if isinstance(title, bytes):
                            title = title.decode("latin1", "replace")
                        return title

            return title
        except error.XError:
            logger.exception("Error while getting window title")
            return None

    def _loop(self):
        self._display.sync()
        while True:
            ev = self._display.next_event()
            if ev.type not in self._event_handlers:
                logger.debug(
                    f"Received {ev.__class__.__name__} event with no configured handler"
                )
                continue

            logger.debug(f"Handling event of type {ev.__class__.__name__}")
            self._event_handlers[ev.type](ev)
