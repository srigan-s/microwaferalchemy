# Deployment

Everything here targets a Raspberry Pi 5 running **Raspberry Pi OS 64-bit** with
the Yahboom Raspbot V2 controller on I2C bus 1 (address `0x2B`).

| Script | Runs on | Purpose |
| --- | --- | --- |
| `install_pi.sh` | Raspberry Pi | apt prerequisites, I2C enable, virtualenv, install from bundle/wheel/source, log + config directories |
| `offline_install.sh` | Raspberry Pi (or any machine) | verify bundle hashes, create venv, `pip install --no-index` from bundled wheels |
| `verify_hardware.sh` | Raspberry Pi | read-only check: `i2cdetect`, controller at `0x2B`, sensor frames. No motion |
| `deploy.sh` | development machine | build/copy the bundle over SSH (`waferbot.local` / `pi` by default) and run the installer |

## What these scripts do not do

* They never erase, format, or mount a storage device. Flashing the new SD card
  is a manual step with Raspberry Pi Imager (see `../README.md`).
* They never install an autostart service. Nothing moves at boot; every motion
  command needs `--physical` plus a typed confirmation.
* They never run a motor test. `verify_hardware.sh` is read-only; the motor test
  is a separate, prompted command with the wheels off the ground.

## Pi apt prerequisites

Raspberry Pi OS ships a working Python 3.11+. The installer adds:

```bash
sudo apt-get install -y python3-venv python3-dev i2c-tools libi2c-dev
```

`smbus2` is pure Python, so no compiler is required for the runtime itself; the
development packages are installed because some Raspberry Pi OS images do not
ship the venv module. If you build the offline bundle on a machine without
network access to PyPI, prepare the wheels on a connected machine with a
compatible Python and copy `dist/waferbot-offline-*/` across.

## I2C

Enabling I2C turns on the interface **and** loads the kernel module at boot.
Follow the Raspberry Pi documentation (Configuration → Hardware communication →
I2C) either through the desktop Control Centre or:

```bash
sudo raspi-config
# 3 Interface Options -> I5 I2C -> Yes
```

The installer can do that non-interactively with
`sudo raspi-config nonint do_i2c 0`. A reboot is normally required before
`/dev/i2c-1` exists.

