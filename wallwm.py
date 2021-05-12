from Xlib.display import Display
from Xlib import X, error
import argparse


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


def main_loop(uuid):
    dpy = Display()
    default_screen = dpy.screen()
    default_screen.root.change_attributes(event_mask=X.SubstructureNotifyMask)

    while True:
        ev = dpy.next_event()
        if ev.type == X.ConfigureNotify:
            window_geometry = ev.window.get_geometry()

            x = window_geometry.x
            y = window_geometry.y
            width = window_geometry.width
            height = window_geometry.height

            # TODO: Unconditionally raise info window here
            #  raise_window()

            if (
                x == y == 0
                and width == default_screen.width_in_pixels
                and height == default_screen.height_in_pixels
            ):
                continue

            fullscreen_window(dpy, ev.window)
        elif ev.type == X.CreateNotify:
            fullscreen_window(dpy, ev.window)


if __name__ == "__main__":
    # TODO: Run this as a systemd service and automatically restart when it goes down,
    #  logging exceptions to watchdog
    parser = argparse.ArgumentParser()
    parser.add_argument('--uuid')
    args = parser.parse_args()
    main_loop(uuid=args.uuid)
