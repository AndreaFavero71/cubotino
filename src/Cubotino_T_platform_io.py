#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 25 October 2026
#
# This is the ONLY module in Cubotino that is allowed to know which Raspberry Pi OS
# (or which Raspberry Pi model) the robot is running on. Everything OS-specific lives here.
#
# Reserved GPIOs" (4 = display backlight, 18 = top-cover LED, 12 and 13 = servos)
#
# Design rules:
#   - This module imports NOTHING from Cubotino. It is a leaf module (no cycles).
#   - Callers (Cubotino_T.py, servos, pigpiod, setup helpers) ask this module
#     questions; they do not branch on os_version themselves.
#   - No silent degradation: if a required capability is missing (e.g. pigpio),
#     raise a RuntimeError with an actionable message instead of falling back.
#   - Supported targets: OS 10 (Buster, legacy), OS 11 (Bullseye), OS 12 (Bookworm),
#     OS 13 (Trixie). Pi 5 servo path is explicitly unsupported.
#
#############################################################################################################
"""

import os
import re
import time


# ---------------------------------------------------------------------------
# OS detection
# ---------------------------------------------------------------------------

_OS_VERSION_CACHE = None


def get_os_version() -> int:
    """Return the Raspberry Pi OS major version as an int (10, 11, 12, 13...).
    Returns 0 if it cannot be determined (e.g. non-RPi Linux, or parse failure).
    Result is cached after the first call."""
    global _OS_VERSION_CACHE
    if _OS_VERSION_CACHE is not None:
        return _OS_VERSION_CACHE

    version = 0
    try:
        with open('/etc/os-release', 'r') as f:
            for row in f:
                if row.startswith('VERSION_ID'):
                    digits = ''.join(c for c in row if c.isdigit())
                    if digits:
                        version = int(digits)
                    break
    except OSError:
        version = 0

    _OS_VERSION_CACHE = version
    return version


def is_legacy_camera() -> bool:
    """True when the legacy picamera stack applies (OS <= 10)."""
    v = get_os_version()
    return 0 < v <= 10


# ---------------------------------------------------------------------------
# Camera backend selection
# ---------------------------------------------------------------------------

def get_camera_backend():
    """Return the camera backend *module* appropriate to this OS.

    The returned module exposes the same public surface used by Cubotino_T.py:
        module.camera           -> singleton Camera instance
        module.camera.get_width() / get_height() / get_frame()
        module.camera.set_auto(...) / set_gains(...) / get_metadata() / ...
    plus module-level helpers used by the caller.

    Two backends are supported and intentionally kept separate:
        OS <= 10 : Cubotino_T_camera_os10   (picamera, Buster-only)
        OS >= 11 : Cubotino_T_camera        (picamera2, modern default)
    """
    if is_legacy_camera():
        import Cubotino_T_camera_os10 as backend
    else:
        import Cubotino_T_camera as backend
    return backend


# ---------------------------------------------------------------------------
# GPIO pin factory (pigpio is required, no fallback)
# ---------------------------------------------------------------------------

_GPIO_FACTORY_CACHE = None


def get_gpio_pin_factory():
    """return a gpiozero PiGPIOFactory instance.

    pigpio is required for Cubotino: it provides hardware-timed pwm that keeps
    the servos stable. there is intentionally no fallback to lgpio, because
    software-timed pwm causes servo jitter.

    raises RuntimeError with an actionable message if:
      - the python 'pigpio' module is missing (pip3 install pigpio)
      - the pigpiod daemon is not reachable
      - the board is a pi 5 (pigpio does not support the rp1 gpio controller)

    note: no gpio pin is touched here. an earlier version probed pin 4 to
    verify the factory, but gpio 4 is the display backlight pin on the st7735
    module, and releasing it broke the backlight pwm. reachability is now
    verified through _pigpiod_reachable(), which opens a socket connection to
    the daemon and does not touch any pin.
    """
    global _GPIO_FACTORY_CACHE
    if _GPIO_FACTORY_CACHE is not None:
        return _GPIO_FACTORY_CACHE

    if _is_pi5():
        raise RuntimeError(
            "Cubotino servo layer does not support Raspberry Pi 5: pigpio "
            "cannot drive the RP1 GPIO controller. Use a Pi Zero 2 W, Pi 3 or Pi 4."
        )

    try:
        import pigpio  # noqa: F401
    except ImportError as e:
        raise RuntimeError(
            "python module 'pigpio' is not installed in the active environment.\n"
            "fix: with the venv activated, run  pip3 install pigpio"
        ) from e

    if not _pigpiod_reachable():
        raise RuntimeError(
            "pigpio python module is present but the pigpiod daemon is not "
            "reachable.\n"
            "fix: start the daemon with  sudo systemctl start pigpiod  "
            "(or  sudo pigpiod  if the unit is not installed)."
        )

    try:
        from gpiozero.pins.pigpio import PiGPIOFactory
        factory = PiGPIOFactory()
    except Exception as e:
        raise RuntimeError(
            "could not create PiGPIOFactory even though pigpiod is reachable."
        ) from e

    _GPIO_FACTORY_CACHE = factory
    return factory



def _is_pi5() -> bool:
    """Best-effort Pi 5 detection via /proc/device-tree/model."""
    try:
        with open('/proc/device-tree/model', 'rb') as f:
            model = f.read().decode('utf-8', errors='ignore')
        return 'Raspberry Pi 5' in model
    except OSError:
        return False


# ---------------------------------------------------------------------------
# pigpiod lifecycle
# ---------------------------------------------------------------------------

def ensure_pigpiod(verbose: bool = False) -> bool:
    """ensure the pigpiod daemon is running and has claimed its GPIO resources.

    on os <= 11 the daemon is restarted unconditionally with kill + start,
    solving a Bullseye/pigpiod/systemd race on GPIO resource claim.

    on os >= 12 the daemon is started only if it is not reachable (there
    is no known boot-time race on these OS, so no forced restarted).

    returns True if the daemon is reachable after this call, false
    otherwise. never raises.
    """
    import subprocess

    if get_os_version() <= 11:
        # force a clean daemon state
        try:
            subprocess.run(
                ['sudo', 'killall', 'pigpiod'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5
            )
        except Exception:
            pass
        time.sleep(0.2)

        started = False
        for cmd in (['sudo', 'systemctl', 'start', 'pigpiod'],
                    ['sudo', 'pigpiod']):
            try:
                subprocess.run(
                    cmd,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=5
                )
            except Exception:
                continue
            time.sleep(0.5)   # let the daemon claim dma/pwm resources
            if _pigpiod_reachable():
                started = True
                break

        if started:
            if verbose:
                print("pigpiod restarted")
            return True

        if verbose:
            print(
                "could not restart pigpiod.\n"
                "fix: make sure the daemon is installed and can be started:\n"
                "  sudo systemctl status pigpiod\n"
                "  sudo systemctl restart pigpiod"
            )
        return False

    # os >= 12
    if _pigpiod_reachable():
        if verbose:
            print("pigpiod already running")
        return True

    # try to start it (works if the user installed a unit manually)
    for cmd in (['sudo', 'systemctl', 'start', 'pigpiod'],
                ['sudo', 'pigpiod']):
        try:
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5
            )
        except Exception:
            continue
        time.sleep(0.5)
        if _pigpiod_reachable():
            if verbose:
                print("pigpiod started")
            return True

    if verbose:
        print(
            "pigpiod is not reachable.\n"
            "fix: start the daemon with  sudo systemctl start pigpiod  "
            "(or  sudo pigpiod  if the unit is not installed),\n"
            "then run Cubotino again."
        )
    return False


def _pigpiod_reachable() -> bool:
    """True if a pigpio connection can be opened and closed cleanly."""
    try:
        import pigpio
        pi = pigpio.pi()
        ok = pi.connected
        pi.stop()
        return ok
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Misc helpers that will be useful to callers
# ---------------------------------------------------------------------------

def get_boot_config_path() -> str:
    """Return the path to config.txt for this OS."""
    if get_os_version() >= 12:
        return '/boot/firmware/config.txt'
    return '/boot/config.txt'


def is_armv6() -> bool:
    """True on the original Pi Zero / Pi 1 (armv6l)."""
    return 'armv6' in os.uname().machine