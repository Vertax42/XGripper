## Install libsurvive

```bash
cd ~ && git clone https://github.com/cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt update && sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
sudo make install -j4
```

Plug in a headset / tracker / controller / etc and run:

```bash
./bin/survive-cli
```

This should calibrate and display your setup.

For visualization, you can either download a binary of websocketd or enable the experimental apt source and use sudo apt install websocketd. After which you can run:

```bash
./bin/survive-websocketd & xdg-open ./tools/viz/index.html
```
