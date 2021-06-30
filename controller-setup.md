# Setup


Base system is Ubuntu 20.04

## Usability(? where to group this stuff? what to call it?)

- `sudo update-alternatives --set editor /usr/bin/vim`

## Drivers

(Possibly https://itectec.com/ubuntu/ubuntu-how-to-enable-early-kms-on-ubuntu/ if this worked?)

## Applications

- Install Google Chrome

## Users

- Create cstv user IN GROUP docker and cstv

## Systemd

- Make sure to symlink
- TODO blah blah blah

## Docker

- Install Docker
- Follow steps to make it work for all users
- Install Docker Nvidia runtime
  - nvidia-docker2 is a must
- Add cstv user to docker group

## Node

Setup based on https://github.com/nodejs/docker-node/blob/main/16/buster/Dockerfile (Is this the best way for Ubuntu specifically?)

- Maybe also regularly update npm automatically?

## Controller

TODO: Update/break out this section

- Build picom (X compositor) from source (https://www.linuxfordevices.com/tutorials/linux/picom)
- Install fonts Open Sans and Montserrat from Google fonts into /usr/share/fonts/truetype/{open-sans,montserrat}
  - Clear cache with `fc-cache -f -v`
- Create a `logs` folder in home
- Edit .xinitrc to run all of the things
- Build hsetroot (command to run will be `DISPLAY=:0 hsetroot -solid black`)

- Install Node via NVM


## ROS2

TODO: Organize this into a set of steps rather than a bunch of unorganized notes.

Ports bound should be as described in the
[ROS2 documentation](https://docs.ros.org/en/ros2_documentation/galactic/Concepts/About-Different-Middleware-Vendors.html). We use the default `ROS_DOMAIN_ID` because we're behind a firewall.
Use `lsof -aPi -p <pid>` to see what ports are bound by a specific process.

Force ROS to only broadcast on localhost by setting `ROS_LOCALHOST_ONLY=1` (TODO: VERIFY THAT THIS WORKS).


## Firewall

TODO: What does this look like? Do we need to set up rules internally? Will we be able to set up a firewall externally somehow?


## NGINX

### Ports and routes:

- 0.0.0.0:80: NGINX
  - /: single-page static React app
  - /api: proxy to Public API
  - /messaging: proxy to Router
    - /messaging/out is blocked for all but internal traffic (see https://stackoverflow.com/a/54666335/1979008)
- :8000: Controller
- :8001: Public API
- :808x: Internal static servers for web apps


controller


- Install: `sudo apt install nginx`

## Asides

// LIBREALSENSE BUILD LINE: `cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP=true -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=true -DOpenGL_GL_PREFERENCE=GLVND`
LIBREALSENSE BUILD LINE: `cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=true -DOpenGL_GL_PREFERENCE=GLVND`

If you want to change the background color, you'll need to use not xsetroot, but https://github.com/himdel/hsetroot AFAIK. Starting picom makes the background gray, which might not be an issue if crossfading between applications is handled well.
