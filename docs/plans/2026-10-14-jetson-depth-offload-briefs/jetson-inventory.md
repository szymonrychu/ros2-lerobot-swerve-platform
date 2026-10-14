# Jetson Nano B01 inventory (192.168.1.32, hostname `nano`)

Collected 2026-10-14 ~10:33-10:40 CEST, read-only, uptime 8-15 min. Only writes: `/tmp/inv_cam0.jpg`, `/tmp/inv_cam1.jpg` on the Jetson. Raw dumps: `scratchpad/raw/*.txt`.

## 1. OS and boot

- Ubuntu 20.04.6 LTS (focal), L4T R32.6.1 (JetPack 4.6), kernel 4.9.253-tegra, Python 3.8.10 + 2.7.18.
- It is an **in-place 18.04 -> 20.04 upgrade of JetPack 4.6**, matching the **Qengineering community 20.04 image** fingerprint exactly (OpenCV 4.8.0 CUDA, PyTorch 1.13.0a0, torchvision 0.14.0a0, TF 2.4.1, TRT 8.0.1.6). Evidence: NVIDIA apt sources are commented `# disabled on upgrade to focal`; 52 `rc` leftovers from 18.04 (chromium 91 bionic, thunderbird 78 bionic, libreoffice 6.0 bionic). So the memory note is right.
- Boot: B01 QSPI bootloader reads `/boot/extlinux/extlinux.conf` **from the SD card** (mmcblk0p1, 29 GB). DEFAULT=`JetsonIO` (custom HDR40 DTB dated 2025-07-16, from jetson-io). Kernel, initrd and DTB are loaded from the SD card; `root=PARTUUID=02750bed...` = SSD `/dev/sda1` (119 GB USB UAS, Intrinsix 0578:0578 bridge behind a Realtek USB 3 hub). SD and SSD copies of extlinux.conf and the custom DTB are identical. **SD card is still required for boot** (and still holds a full old rootfs). The SD rootfs is not mounted.
- **Storage warnings:** dmesg shows repeated `uas_eh_abort_handler` / `uas_eh_bus_reset_handler` on sda (every few seconds at boot, again at 683 s) and `tegra-xusb ... Transfer event for disabled endpoint`. `dumpe2fs`: sda1 `clean with errors`, last fsck 2024-10-20, mount count 123, max -1. Kernel: "mounting fs with errors, running e2fsck is recommended". The USB-SSD path is unhealthy.
- Disk: `/` 117G, 28G used, 84G free. `du`: /usr 9.3G (/usr/lib 6.1G, /usr/local 4.8G: cuda-10.2 2.9G, /usr/local/lib 2.0G = python3.8 dist-packages TF 1.1G, torch 570M), /var 4.0G (docker 3.1G), /var/lib/snapd 1.7G, /home 2.5G, /opt 262M (ros 132M, nvidia 120M, ota_package 11M), /root 113M.
- RAM 3.9 GiB, 625 MiB used at idle-ish. Swap = nvzramconfig 4x 495 MiB zram (1.9 GiB), no swapfile. fstab only `/dev/root`.
- nvpmodel: MAXN (mode 0), "fan mode is not set" warning. jetson_clocks not applied (CPU schedutil 1.2-1.48 GHz, GPU at 76.8 MHz min, EMC 1600). Fan PWM 0 (target 0). Thermals: AO 36.5-38.5C, CPU 28.5-32.5C, GPU 27-29.5C, PLL 27.5-30C, PMIC 50C. thermal_zone6 read fails (-5) at boot (cosmetic).

## 2. GPU health

- **Working this boot.** No nvgpu/FECS/0xbadfbadf errors in dmesg; only the normal GPCPLL init line.
- `torch.cuda.is_available()` = True, 1024x1024 matmul on "NVIDIA Tegra X1" completed (15 s including CUDA context init, normal for Nano cold start).
- `cv2.cuda.getCudaEnabledDeviceCount()`=1, `cv2.cuda.resize` on GpuMat OK, `cv2.cuda.createStereoBM` OK.
- No deviceQuery binary built (samples exist in /usr/local/cuda/samples, source only).
- The earlier FECS boot loop left **no trace** in /var/log (syslog.1, kern.log*, 0 matches); journald is volatile (no /var/log/journal). Those boots likely died before rsyslog flushed; only serial console would have them.
- Historical: syslog.5-7 (Jul 2025) contain the old imx219 -121 probe failures.

## 3. Accelerated stack

- CUDA 10.2.300 (`/usr/local/cuda-10.2`, toolkit + samples + docs), cuDNN 8.2.1.32 (+dev, samples), TensorRT 8.0.1.6 (libnvinfer8, dev, samples, doc, python `tensorrt` 8.0.1.6 importable), VPI 1.1.15 (libnvvpi1, vpi1-dev, samples, demos, python-vpi1 - **`import vpi` fails on py3.8**; the binding is built for py3.6), VisionWorks 1.6 (10 libvisionworks* packages, deprecated), Multimedia API.
- nvidia-l4t-* 32.6.1: 3d-core, apt-source, bootloader, camera, configs, core, cuda, firmware, gputools, graphics-demos, gstreamer, init, initrd, jetson-io, jetson-multimedia-api, kernel, kernel-dtbs, kernel-headers, libvulkan, multimedia, multimedia-utils, oem-config, tools, wayland, weston, x11, xusb-firmware.
- OpenCV: **active cv2 = 4.8.0 custom build** at `/usr/lib/python3/dist-packages/cv2` (install prefix /usr, CUDA 10.2 YES arch 5.3, cuDNN 8.2.1 YES, GStreamer 1.16.3 YES, FFMPEG YES, GTK3). pkg-config opencv4 = 4.8.0. py2 cv2 also 4.8.0. Also present: Ubuntu libopencv*-4.2 packages (+ python3-opencv 4.2), NVIDIA libopencv 4.1.1 debs, and stale libopencv_core .so for 4.1.1, 4.2, 4.5.3, 4.6.0, 4.8.0 in /usr/lib/aarch64-linux-gnu; stale cv2 for py3.6 in /usr/lib/python3.6.
- GStreamer 1.16.3; nvarguscamerasrc, nvv4l2camerasrc, nvvidconv, nvjpegenc/dec, nvv4l2 h264/h265 enc, nvv4l2decoder, nveglglessink etc. present.
- Python libs (pip /usr/local): torch 1.13.0a0 (CUDA OK), torchvision 0.14.0a0, tensorflow 2.4.1 (+tensorboard, keras bits), numpy 1.18.5, scipy 1.3.3, platformio 6.1.16, jetson-stats 4.3.2, Jetson.GPIO, pyserial, rospkg/catkin/rosdep etc, fastapi-ish stack (starlette, uvicorn, pydantic 2). Not installed: onnxruntime, pycuda, jetson-inference. Python2 has no pip.
- Docker: docker.io 26.1.3, nvidia-docker2 2.2, nvidia-container-runtime default runtime.

## 4. Cameras

- `/dev/video0` = imx219 7-0010 (vi:0), `/dev/video1` = imx219 8-0010 (vi:4). Both bound this boot (`subdev imx219 7-0010 bound`, `8-0010 bound`); no -121 errors. New ribbons fixed it.
- Modes: 3264x2464@21, 3264x1848@28, 1920x1080@30, 1640x1232@30, 1280x720@60 (RG10).
- `nvarguscamerasrc sensor-id=0/1 num-buffers=1` 1280x720 -> nvjpegenc: both `Done Success`, rc 0, 71 KB / 68 KB JPEGs with a real scene (same bench, horizontally offset - plausible stereo pair). Strong pink/magenta cast: no ISP tuning override ("No override file found", "LSC surface is not based on full res") - expected without `camera_overrides.isp`; matters for colour, less for depth.
- nvargus-daemon enabled and active.

## 5. Services and processes

- Default target multi-user (GUI not started), but `display-manager.service -> gdm3` and lightdm/lxde/unity/ubuntu-desktop are installed.
- **CPU hog:** `ros_robot_platform.service` (custom, enabled, `/etc/systemd/system`) runs `roslaunch robot_platform robot_platform.launch` as jetson with `ROS_MASTER_URI=http://robot.lan:11311`. Its `platform_controller` node crash-loops every ~3 s (`/dev/serial/by-id/usb-Teensyduino_USB_Serial_13671670-if00` missing), each respawn burns 200-340% CPU during Python/rospy import; load avg ~3.2 at idle. Also running from it: rosmaster, rosout, move_base, hector_mapping, hector_trajectory_server, hector_geotiff, ekf_localization, rosbridge_websocket (:9090), rosapi.
- Running services: anacron avahi-daemon bluetooth colord containerd cron cups cups-browsed dbus docker fwupd getty@tty1 gpsd haveged irqbalance jtop kerneloops ModemManager netplan-wpa-wlan0 networkd-dispatcher NetworkManager nvargus-daemon nvmemwarning nvphs nvs-service polkit ros_robot_platform rpcbind rsyslog rtkit-daemon serial-getty@ttyGS0 serial-getty@ttyS0 ssh systemd-{journald,logind,networkd,resolved,timesyncd,udevd} udisks2 upower user@1000 uuidd whoopsie wpa_supplicant.
- Enabled extras not listed above: accounts-daemon, blueman-mechanism, ModemManager, nv-l4t-usb-device-mode, nvweston, nvfb, nvfb-early, nv-late-init, nvpmodel, nvzramconfig, pppd-dns, portmap, rsync, spice-vdagent(d), snapd.* (5 units), ubuntu-advantage, ua-reboot-cmds, ubuntu-fan, ufw, gpsd, lm-sensors, switcheroo-control, e2scrub_reap. Failed: systemd-networkd-wait-online. unattended-upgrades not installed.
- Custom unit files: `ros_robot_platform.service`, `jtop.service` (jetson-stats), `vncserver@.service` (TightVNC, template, not running).
- Timers: tmpfiles-clean, motd-news, fwupd-refresh, logrotate, man-db, e2scrub_all, fstrim, anacron, snapd.snap-repair, ua-timer.
- Cron: no user/root crontab; cron.d anacron, e2scrub_all, popularity-contest; cron.daily includes apport, popularity-contest, update-notifier-common. No rc.local.
- User units: pulseaudio, tracker-miner-fs, ubuntu-report.path, gpg sockets. No ~/.config/autostart; /etc/xdg/autostart is the full GNOME/LXDE/Unity set (only matters if GUI starts).
- Docker: containers `gifted_lalande` (image 51dbe19fb728, npm, exited 15 mo) and `great_greider` (cruise/webviz, exited); images node:12.22 864MB, untagged 867MB, ghcr.io/bitmeal/rosproxy 245MB, cruise/webviz 141MB. 1.1 GB images + 1.18 GB containers reclaimable.
- Snaps: bare, core18, core20, core22, gnome-3-34-1804, gnome-3-38-2004, gtk-common-themes, snap-store, snapd (two revisions each, 16 loop mounts, 1.7 GB).
- Listening: sshd :22; rosmaster :11311; rosbridge :9090; ~20 ephemeral ROS node TCP/UDP ports on 0.0.0.0; rpcbind :111 (tcp/udp, all interfaces); avahi :5353; cupsd 127.0.0.1:631; gpsd 127.0.0.1:2947; containerd 127.0.0.1; resolved 127.0.0.53.
- Memory top: hector_mapping 110 MB, docker 78 MB, rosbridge 62 MB, containerd 45 MB, fwupd 41 MB, roslaunch/rosmaster ~40 MB each.

## 6. Old ROS

- ROS 1 **Noetic**, `ros-noetic-ros-base` + 174 ros-noetic-* debs (navigation: move_base, costmap, dwa/dwb, teb deps, global_planner; hector_slam; robot_localization; rosbridge/rosapi; cv_bridge, depth_image_proc; gpsd_client; geographic_msgs ...), /opt/ros/noetic 132 MB, apt source `ros-latest.list`. Python ROS tooling: python3-catkin*, rosdep, rosdistro, rosinstall(-generator), rospkg, wstool, vcstools (deb + pip copies).
- Workspace: `/home/jetson/catkin_ws` (82 MB, catkin_make layout, src/build/devel), one repo: `src/ArduinoPlatformController` (68 MB, `git@github.com:szymonrychu/ArduinoPlatformController.git`, branch main == origin/main, clean, HEAD 0b87487 2025-07-15 "fix: Update in-place rotation moves"), one package `ros/robot_platform`.
- `~/.ros` 583 MB (410 log dirs). `~/.zshrc` sources `/opt/ros/noetic/setup.zsh` and exports ROS_MASTER_URI / ROS_HOSTNAME=robot.lan; `.bashrc` only adds cuda to PATH. User shell is zsh (oh-my-zsh + powerlevel10k).
- Other robot/legacy home content: `webviz` (cruise webviz clone, 43 MB), `Documents/rvizweb` (osrf, local Dockerfile mod), `lc29h_gps_rtk_hat_code` (+ .zip), `main.py` (17 KB, Jul 2025), `gpsdata.txt`, `jetson-io.patch`, `openvpn.sh`, `bootFromUSB` (JetsonHacks, `copyRootToUSB.sh` modified), `.platformio` 518 MB (Teensy toolchains), `.vscode-server` 801 MB, `.cache` 402 MB, `.dotnet`, `.mozilla`, `.vnc`, `.rviz`, `.keras`.
- udev: `teensy_udev.rules`, `rplidar.rules` (cp210x -> /dev/rplidar), `51-kinect.rules`. Plus gpsd service.
- /etc/hosts: 192.168.1.32 robot.lan, .35 steamdeck.lan, .36 controller.lan, 10.8.0.x master/steamdeck/overlord.ros (old OpenVPN ROS net).

## 7. Network

- Renderer: **systemd-networkd via netplan** (`/etc/netplan/00-wifi.yaml`); NetworkManager runs but every device is `unmanaged` (only a WiFi profile "main" and docker0 exist in NM).
- wlan0 (Intel 8265, iwlwifi): static 192.168.1.32/24, gw 192.168.1.1, DNS 192.168.1.1, SSID "main" - **the only internet path** (ping 1.1.1.1 OK, 60 ms). iwlwifi logged a microcode SW error + restart at boot but recovered.
- eth0: netplan `dhcp4: yes`, **carrier up at 1000 Mb/s** (something is plugged in) but no IPv4 (no DHCP server on the link), only link-local IPv6. This is why `systemd-networkd-wait-online` fails.
- Others: docker0 172.17.0.1/16 (down), l4tbr0/usb0/rndis0 (USB device-mode gadget, down), dummy0.
- Hostname `nano`; time via systemd-timesyncd (synced, Europe/Amsterdam); chrony/ntp not installed.

## 8. Proposed removal list

Risk scale: L = safe, M = check first, H = keep. "Purge" = apt purge / rm; "disable" = systemctl disable --now.

### Must keep (H)
nvidia-l4t-* (core, cuda, camera, gstreamer, multimedia, firmware, kernel*, bootloader, initrd, init, configs, tools, xusb-firmware, jetson-io), nvargus-daemon, nvpmodel, nvzramconfig, nvphs, nv.service, nv-late-init, CUDA 10.2 runtime libs, libcudnn8, libnvinfer8 + plugin + onnxparsers, libnvvpi1, custom OpenCV 4.8 (`/usr/lib/python3/dist-packages/cv2`, `libopencv_*.so.4.8.0`, opencv4.pc), gstreamer1.0-plugins-base/good, ssh, systemd-networkd/netplan, wpa_supplicant, timesyncd, haveged. Also keep the SD card (boot partition) and `/boot` custom DTB.

### Disable now, purge later
| Item | Cost | Risk | Action |
|---|---|---|---|
| `ros_robot_platform.service` | 2-3.4 cores in crash loop, ~400 MB RAM, ~25 open ports | L (ROS moves to Pi) | disable, then delete unit |
| docker + containerd (+nvidia-docker2, nvidia-container-*) | 125 MB RAM, 3.1 GB disk | M: keep if you want L4T containers for the vision service; else purge | disable; purge if native deploy |
| gdm3/lightdm/lxde/unity/ubuntu-desktop, xserver, weston (nvweston), nvfb | ~1-2 GB disk; nothing running now (multi-user target) | M: `nvidia-l4t-x11/weston/3d-core` are deps of nvidia-l4t-* - do not purge those; purge only the DE meta/apps and set multi-user (already) | purge DE meta packages, keep l4t graphics debs |
| snapd + all snaps | 1.7 GB, 16 loop mounts, snapd RAM | L | purge |
| cups, cups-browsed, hplip, printer drivers | RAM + :631 | L | purge |
| bluetooth, bluez, blueman | RAM | L (unless BT gamepad needed) | disable/purge |
| ModemManager | RAM, probes serial ports | L | purge |
| avahi-daemon | :5353 | L-M (lose .local mDNS; Pi link uses static IP) | disable |
| whoopsie, apport, kerneloops, popularity-contest, ubuntu-report, update-notifier, motd-news, ubuntu-advantage/ua-timer | small, phone-home | L | purge |
| fwupd (+ refresh timer) | 41 MB RAM | L | purge |
| gpsd | old GPS hat | L | purge |
| rpcbind/portmap, nfs-common | :111 on all interfaces | L | purge |
| spice-vdagent, vino, tightvncserver, remmina, vncserver@ unit | small | L | purge |
| NetworkManager (all devices unmanaged; networkd does the work) | ~20 MB RAM | M: confirm netplan owns wlan0 (it does) before purging | disable, purge later |
| tracker-miner-fs (user), pulseaudio (user), colord, rtkit, udisks2, upower, switcheroo-control, accounts-daemon, onboard, orca, brltty | RAM | L | purge with DE |
| jtop.service (jetson-stats) | 18 MB RAM | L (useful for monitoring) | keep or disable |

### Purge (disk only)
| Item | Size | Risk | Action |
|---|---|---|---|
| ROS Noetic: 174 ros-noetic-* + python3-ros*/catkin/rosdep, ros-latest.list | 132 MB + deps | L. Note `ros-noetic-cv-bridge` pulls Ubuntu libopencv 4.2 - purging it lets you drop 4.2 too | purge |
| ~/catkin_ws | 82 MB | L: repo is clean and == origin/main on GitHub | rm after confirming |
| ~/.ros logs | 583 MB | L | rm |
| ~/webviz, ~/Documents/rvizweb (local Dockerfile change), ~/.platformio (518 MB, Teensy fw tooling), ~/lc29h_gps_rtk_hat_code(+zip), main.py, gpsdata.txt, openvpn.sh, ~/bootFromUSB (local change) | ~600 MB | L-M: rvizweb and bootFromUSB have uncommitted edits; main.py / openvpn.sh unversioned - copy off first | archive, then rm |
| ~/.vscode-server, ~/.cache, ~/.dotnet, ~/.mozilla | ~1.2 GB | L | rm |
| Docker images/containers (node:12.22, untagged, rosproxy, webviz) | 2.3 GB | L | `docker system prune -a` (or with docker purge) |
| TensorFlow 2.4.1 + tensorboard + keras pip pkgs | ~1.2 GB | L unless a TF model is planned | pip uninstall |
| platformio pip, youtube-dl, other pip cruft | small | L | pip uninstall |
| VisionWorks 1.6 (10 libvisionworks* + repo lists, nvidia-container-csv-visionworks) | ~200 MB | L: deprecated, superseded by VPI | purge |
| CUDA docs, cuda-samples, libnvinfer-samples/doc, libcudnn8-samples, vpi1-samples/demos, nvidia-l4t-graphics-demos | ~1.3 GB | L (runtime untouched; keep cuda-nvcc and *-dev if you build on-device) | purge |
| cuda-repo-l4t-10-2-local (local apt repo copy) | 1.0 GB | L: CUDA debs already installed; removes ability to reinstall offline | purge |
| Stale OpenCV: Ubuntu libopencv*-4.2 + python3-opencv 4.2, NVIDIA libopencv 4.1.1 debs, orphan libopencv_* .so.4.5.3/.4.6.0, py3.6 cv2 dir | ~150-300 MB | M: make sure nothing links 4.2 (cv_bridge goes with ROS); python3-opencv 4.2 is shadowed by 4.8 but removing it must not touch `/usr/lib/python3/dist-packages/cv2` (verify with `dpkg -S` first) | purge carefully |
| Python 2.7 + py2 cv2 | ~100 MB | M: check reverse deps | purge late |
| firefox, 52 `rc` leftovers (chromium, thunderbird, libreoffice from 18.04) | ~250 MB + configs | L | purge / `dpkg --purge` rc |
| linux-firmware (non-Tegra blobs) | 700 MB | M-H: iwlwifi firmware comes from here - keep | keep |

### Other findings to act on (not removals)
- SSD on USB UAS shows repeated resets and an ext4 "clean with errors" state - run an offline fsck and consider `usb-storage.quirks=0578:0578:u` (disable UAS) or a different enclosure before relying on it.
- No log persistence: enable persistent journald (or a serial console) to catch the next FECS event.
- eth0 already has link at 1 Gb/s; it is set to DHCP with no server - set a static address in netplan for the Pi link.
- python VPI binding not importable on 3.8 (VPI 1.1 py bindings target 3.6): plan depth on cv2.cuda (StereoBM/StereoSGM work) or the VPI C API.
- Camera colour cast: install an ISP override for IMX219 if colour matters.
