# Partially based on code by Stephan Sokolow
# Source: https://gist.github.com/ssokolow/e7c9aae63fb7973e4d64cff969a78ae8
from contextlib import contextmanager
from typing import Optional

from Xlib.display import Display
from Xlib import X, error

from Xlib.error import XError
from Xlib.xobject.drawable import Window

import os

dpy = Display()
NET_ACTIVE_WINDOW = dpy.intern_atom("_NET_ACTIVE_WINDOW")
NET_WM_NAME = dpy.intern_atom("_NET_WM_NAME")  # UTF-8
WM_NAME = dpy.intern_atom("WM_NAME")  # Legacy encoding


def scale_window(display, window, x, y, width, height):
    try:
        window.configure(
            x=x,
            y=y,
            width=width,
            height=height,
        )
        display.sync()
    except error.BadWindow:
        pass


def fullscreen_window(display, window):
    screen = display.screen()
    scale_window(display, window, 0, 0, screen.width_in_pixels, screen.height_in_pixels)


def display_layout_env():
    if "CSTV_DISPLAY_LAYOUT" not in os.environ:
        return None

    return os.environ["CSTV_DISPLAY_LAYOUT"].lower()


def layout_placard_dimensions(screen, display_layout):
    default_value = (0, 0, int(screen.width_in_pixels * 0.20), screen.height_in_pixels)
    if display_layout is None or display_layout == "fullscreen":
        return default_value

    if display_layout == "fit4k":
        return 0, 0, 715, 1758

    if display_layout == "production":
        return 0, 0, 978, screen.height_in_pixels

    return default_value


def layout_viewport_dimensions(screen, display_layout):
    default_value = (0, 0, screen.width_in_pixels, screen.height_in_pixels)
    if display_layout is None or display_layout == "fullscreen":
        return default_value

    if display_layout == "fit4k":
        return 715, 0, 3125, 1758

    if display_layout == "fit4k":
        return 978, 0, 3125, screen.height_in_pixels

    return default_value


def main_loop():
    default_screen = dpy.screen()
    default_screen.root.change_attributes(event_mask=X.SubstructureNotifyMask)

    display_layout = display_layout_env()
    placard_dimensions = layout_placard_dimensions(default_screen, display_layout)
    viewport_dimensions = layout_viewport_dimensions(default_screen, display_layout)

    placard_window = None
    while True:
        ev = dpy.next_event()

        is_placard = False
        if hasattr(ev, "window"):
            window_name = get_window_name(ev.window)
            is_placard = window_name and window_name == "cstv-placard"
            if is_placard:
                placard_window = ev.window
                print(f"Setting window with name {window_name} as placard window")

        if ev.type == X.ConfigureNotify:
            # TODO: Find a less hacky solution than this
            if placard_window:
                if not is_placard:
                    placard_window.raise_window()
                else:
                    scale_window(dpy, placard_window, *placard_dimensions)
                    continue

            try:
                window_geometry = ev.window.get_geometry()
            except error.BadDrawable:
                continue

            x = window_geometry.x
            y = window_geometry.y
            width = window_geometry.width
            height = window_geometry.height

            if (
                x == y == 0
                and width == default_screen.width_in_pixels
                and height == default_screen.height_in_pixels
            ):
                continue

            scale_window(dpy, ev.window, *viewport_dimensions)
        elif ev.type == X.CreateNotify:
            scale_window(dpy, ev.window, *viewport_dimensions)


@contextmanager
def window_obj(win_id: Optional[int]) -> Window:
    """Simplify dealing with BadWindow (make it either valid or None)"""
    window = None
    if win_id:
        try:
            window = dpy.create_resource_object("window", win_id)
        except XError:
            pass
    yield window


def _get_window_name_inner(win_obj):
    """Simplify dealing with _NET_WM_NAME (UTF-8) vs. WM_NAME (legacy)"""
    for atom in (NET_WM_NAME, WM_NAME):
        try:
            window_name = win_obj.get_full_property(atom, 0)
        except UnicodeDecodeError:  # Apparently a Debian distro package bug
            raise
        else:
            if window_name:
                win_name = window_name.value
                if isinstance(win_name, bytes):
                    # Apparently COMPOUND_TEXT is so arcane that this is how
                    # tools like xprop deal with receiving it these days
                    win_name = win_name.decode("latin1", "replace")
                return win_name
            else:
                return "<unnamed window>"


def get_window_name(window: Window):
    with window_obj(window.id) as wobj:
        if wobj:
            try:
                win_title = _get_window_name_inner(wobj)
            except error.BadWindow:
                return "<bad window>"

    return win_title


if __name__ == "__main__":
    # TODO: Run this as a systemd service and automatically restart when it goes down,
    #  logging exceptions to watchdog
    main_loop()
