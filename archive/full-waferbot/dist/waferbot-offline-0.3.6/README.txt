waferbot offline bundle 0.3.6

Layout:
  app/        application source (allowlisted files only)
  deployment/ install and verification scripts
  wheels/     waferbot + pure-Python dependency wheels

Install on a Raspberry Pi 5 running Raspberry Pi OS 64-bit:

  bash deployment/offline_install.sh --bundle . --venv ~/.venvs/waferbot

Prerequisites that must come from apt (not bundled here):
  sudo apt-get install -y python3-venv python3-dev i2c-tools libi2c-dev

Nothing is installed to start on boot. Verify before driving:
  ~/.venvs/waferbot/bin/waferbot sensors --physical --samples 5

I2C must be enabled (Control Centre > Interfaces > I2C, or
sudo raspi-config > 3 Interface Options > I5 I2C).
