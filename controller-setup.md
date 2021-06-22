# Controller Setup

- Build picom (X compositor) from source (https://www.linuxfordevices.com/tutorials/linux/picom)
- Install fonts Open Sans and Montserrat from Google fonts into /usr/share/fonts/truetype/{open-sans,montserrat}
  - Clear cache with `fc-cache -f -v`
- Create a `logs` folder in home
- Edit .xinitrc to run all of the things
- Build hsetroot (command to run will be `DISPLAY=:0 hsetroot -solid black`)

## Asides

// LIBREALSENSE BUILD LINE: `cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP=true -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=true -DOpenGL_GL_PREFERENCE=GLVND`
LIBREALSENSE BUILD LINE: `cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=true -DOpenGL_GL_PREFERENCE=GLVND`

If you want to change the background color, you'll need to use not xsetroot, but https://github.com/himdel/hsetroot AFAIK. Starting picom makes the background gray, which might not be an issue if crossfading between applications is handled well.
