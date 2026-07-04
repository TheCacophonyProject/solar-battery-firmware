# Programmer/debugger setup

These are the instructions for setting up a raspberry pi as a device for programming and debugging the battery.

## Hardware

- Raspberry Pi
- [Programmer Hat](https://github.com/TheCacophonyProject/battery-programmer) If you don't have the hat you can follow the schematics for wiring up a cable for programming.

## Software

- Latest RPi image
- Install `pigpio`

```bash
sudo apt update
sudo apt install -y unzip build-essential
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

For the `sudo make install` step you might get an error `No module named 'distutils'`. This is harmless

- `sudo ldconfig`
- `pip3 install --break-system-packages pigpio`
- `sudo systemctl enable pigpiod`
- `sudo systemctl start pigpiod`

Copy over `decode_log.py` to the RPi

Now run using `python3 decode_log.py --gpio 25`
