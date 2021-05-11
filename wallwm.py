from Xlib.display import Display
from Xlib import X, error


def main_loop():
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
            target_width = default_screen.width_in_pixels
            target_height = default_screen.height_in_pixels

            print(width)
            print(height)

            if x == y == 0 and width == target_width and height == target_height:
                continue

            try:
                ev.window.configure(
                    x=0,
                    y=0,
                    width=default_screen.width_in_pixels,
                    height=default_screen.height_in_pixels,
                )
                dpy.sync()
            except error.BadWindow:
                pass


if __name__ == "__main__":
    main_loop()
