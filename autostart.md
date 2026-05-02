# FPPE Server Autostart Diagnosis

**Date:** 2026-04-25
**Host:** fppe
**Status:** Server crashed at boot; not running

## How it starts

The fppe server is launched from `~/.config/labwc/autostart` (labwc compositor's GUI login autostart script). It:

1. Activates conda env `lerobot_alohamini`
2. Runs `python examples/debug/viser_control.py`
3. Logs to `/tmp/viser_control.log`
4. Also launches Chromium in `--kiosk` mode pointing at `http://localhost:8091/`

No systemd user unit. No entries in `~/.bash_profile`, `~/.bashrc`, or `~/.profile`.

## Root cause

The server crashed immediately at boot (~14:24 yesterday) with:

```
ConnectionError: Failed to sync read 'Present_Position' on ids=[21,22,23,24,25,26,1,2,3,4,5,6]
after 4 tries. [TxRxResult] There is no status packet!
```

`/dev/ttyACM0` is present (USB-serial bridge is enumerated), but the **Feetech servos are not replying**.

This is a **hardware/wiring problem**, not software:
- Motor power supply is off, or
- E-stop is engaged, or
- Daisy-chain bus cable from USB board to first servo is loose/unplugged

The autostart script uses `exec python ...` with **no retry logic**, so once it crashes it stays dead until the next login/boot.

## Fix

1. Check robot main power switch / brick is on
2. Check E-stop is released
3. Verify bus cable seated on both ends (USB board → first servo)
4. Restart:
   - **Option A:** Reboot (labwc autostart will re-run)
   - **Option B:** Run manually:
     ```bash
     bash -l -c 'source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot_alohamini && cd ~/lerobot_alohamini && python examples/debug/viser_control.py >> /tmp/viser_control.log 2>&1 &'
     ```

## Hardening opportunity

The current autostart has no resilience. If motors aren't ready at boot time, the server stays down. Consider:

- Wrapping the launch in an `until` loop that retries every few seconds
- Or moving to a **systemd user unit** with `Restart=on-failure` so it self-heals once motor power comes up
