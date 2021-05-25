# Controller Setup

- Build picom (X compositor) from source (https://www.linuxfordevices.com/tutorials/linux/picom)
- Install fonts Open Sans and Montserrat from Google fonts into /usr/share/fonts/truetype/{open-sans,montserrat}
  - Clear cache with `fc-cache -f -v`
- ??


## Asides

If you want to change the background color, you'll need to use not xsetroot, but https://github.com/himdel/hsetroot AFAIK. Starting picom makes the background gray, which might not be an issue if crossfading between applications is handled well.
