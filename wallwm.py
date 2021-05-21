# Partially based on code by Stephan Sokolow
# Source: https://gist.github.com/ssokolow/e7c9aae63fb7973e4d64cff969a78ae8
from contextlib import contextmanager
from typing import Optional

from Xlib.display import Display
from Xlib import X, error

from Xlib.error import XError
from Xlib.xobject.drawable import Window

dpy = Display()
NET_ACTIVE_WINDOW = dpy.intern_atom("_NET_ACTIVE_WINDOW")
NET_WM_NAME = dpy.intern_atom("_NET_WM_NAME")  # UTF-8
WM_NAME = dpy.intern_atom("WM_NAME")  # Legacy encoding


def fullscreen_window(display, window):
    screen = display.screen()
    try:
        window.configure(
            x=0,
            y=0,
            width=screen.width_in_pixels,
            height=screen.height_in_pixels,
        )
        display.sync()
    except error.BadWindow:
        pass


def main_loop():
    default_screen = dpy.screen()
    default_screen.root.change_attributes(
        event_mask=X.SubstructureNotifyMask
    )

    placard_window = None
    while True:
        ev = dpy.next_event()

        if ev.type == X.ConfigureNotify:
            window_geometry = ev.window.get_geometry()

            x = window_geometry.x
            y = window_geometry.y
            width = window_geometry.width
            height = window_geometry.height

            window_name = get_window_name(ev.window)

            # TODO: Find a less hacky solution than this
            if window_name and window_name == "cstv-placard":
                placard_window = ev.window
                print(f"Setting window with name {window_name} as placard window")

            # print(get_window_name(ev.window))

            # TODO: Raise placard window here IF ev.window != placard window
            print(placard_window)
            if placard_window and ev.window.id != placard_window.id:
                print("raising placard window")
                placard_window.raise_window()

            if (
                x == y == 0
                and width == default_screen.width_in_pixels
                and height == default_screen.height_in_pixels
            ):
                continue

            fullscreen_window(dpy, ev.window)
        elif ev.type == X.CreateNotify:

            # print(get_window_name(ev.window))
            fullscreen_window(dpy, ev.window)


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
                title = "<unnamed window>"

    return window_name


def get_window_name(window: Window):
    with window_obj(window.id) as wobj:
        if wobj:
            win_title = _get_window_name_inner(wobj)

    return win_title


if __name__ == "__main__":
    # TODO: Run this as a systemd service and automatically restart when it goes down,
    #  logging exceptions to watchdog
    main_loop()
