# Board setup guide: flash the Octopus and run the arm

How to get from a fresh copy of the repo to the arm moving from the path editor.

Two computers can be involved:
- **Flashing computer:** any Windows/Mac/Linux machine with Arduino IDE. Only needed to put the
  firmware on the board.
- **Run computer:** Windows with WSL (Ubuntu 24.04, ROS 2 Jazzy). Runs ROS and talks to the
  board over USB.

They can be the same Windows machine. Background and design decisions:
`docs/control_system_decisions.md`. More firmware detail: `firmware/README.md`.

> **Safety, read first**
> - **Yaw, Pitch and Focus (TMC2209 drivers, Driver1–3) must be on 24 V, never 48 V.** A TMC2209 on 48 V is destroyed.
> - A driver plugged in backwards is destroyed. Check orientation before power-on.
> - **Z drops when the motors are off** unless the carriage is supported.
> - Homing and the e-stop aren't written yet. Keep a hand on the motor power switch.

---

## Part A: Flash the firmware (flashing computer)

### A1. One-time setup
1. Install **Arduino IDE 2** and **STM32CubeProgrammer** (free from st.com; it needs an ST account).
2. In Arduino IDE, open **File > Preferences > Additional boards manager URLs** and add:
   ```
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```
3. **Tools > Board > Boards Manager**: install **STM32 MCU based boards** (by STMicroelectronics).
4. **Tools > Manage Libraries**: install **TMCStepper** (by teemuatlut), version **0.7.3**.

### A2. Open the sketch
Open `firmware/arm_firmware/arm_firmware.ino` from the repo. The whole firmware is that one
file; `build_opt.h` next to it must stay in the same folder.

### A3. Tools menu settings

| Setting | Value |
|---|---|
| Board | STM32 MCU based boards > **Generic STM32F4 series** |
| Board part number | **Generic F446ZETx** |
| USB support | **CDC (generic 'Serial' supersede U(S)ART)** |
| Upload method | **STM32CubeProgrammer (DFU)** |

If the board or USB setting is wrong, the build stops with an error that says what to change.

### A4. Flash over USB
1. Motor power **off**.
2. Fit the **BOOT0** jumper on the Octopus, connect USB-C to the computer, and press the board's reset button.
3. **Sketch > Upload**. The upload ends with a "download verified" / "start operation achieved" message.
4. **Remove the BOOT0 jumper** and press reset. The firmware is now running.

This replaces BTT's SD-card bootloader, which is expected.

---

## Part B: Set up the run computer (once per computer)

### B1. Windows: install usbipd-win
This passes the USB board through to WSL. In an **admin PowerShell**:
```powershell
winget install usbipd
```
Close and reopen PowerShell afterwards.

### B2. WSL: install dependencies and build
Assumes ROS 2 Jazzy is already installed. Replace `~/capstone_ws` with wherever the repo is
cloned: the folder that contains `src/` and `firmware/`.
```bash
sudo apt update
sudo apt install python3-serial python3-numpy python3-yaml ros-jazzy-joint-state-publisher-gui
cd ~/capstone_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y     # anything else that's missing
colcon build --symlink-install
```

### B3. WSL: serial port permission
Serial ports belong to the `dialout` group, so your user must be in it:
```bash
sudo usermod -aG dialout $USER
```
Then in PowerShell: `wsl --shutdown`, and open WSL again. Check with `groups`: `dialout`
should be listed.

---

## Part C: Every session

### C1. Attach the board to WSL (PowerShell)
Plug the board in (BOOT0 jumper **off**). The first time on this computer, run the bind in
an **admin** PowerShell:
```powershell
usbipd bind --hardware-id 0483:5740
```
Every session (a normal PowerShell is fine; leave the window open):
```powershell
usbipd attach --wsl --hardware-id 0483:5740 --auto-attach
```
`--auto-attach` re-attaches the board after a reset or replug. While it's attached, Windows
can't use the board, so Arduino IDE won't see it.

In WSL, check that it's there:
```bash
ls /dev/ttyACM*       # should print /dev/ttyACM0
```

### C2. Bench test by hand (first time, or after changing anything)
```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 --eol LF -e
```
Type each command and press Enter. Exit with **Ctrl+]**.

| Step | Type | Expect |
|---|---|---|
| 1 | `PING` | `- OK q=64` |
| 2 | `INFO` | `... sysclk=180000000 ...` |
| 3 | turn **motor power on**, then `DRIVERS` | one `DRIVER ... ok` line per driver (6) |
| 4 | `ENABLE` | `- OK ...`; all motors hold and resist turning |
| 5 | `SPIN YAW 3200 400` | Yaw makes one turn (about 8 s) |
| 6 | `DISABLE` | motors go limp (support Z first) |

- Run `DRIVERS` again whenever motor power has been off: the drivers forget their settings.
- `ENABLE` / `DISABLE` switch every motor at once.
- To test another joint, use `SPIN <THETA|Z|R|YAW|PITCH> <steps> [steps/s]`. Negative steps reverse.
- **Close miniterm before step C3.** Only one program can have the port open at a time.

### C3. Run the arm from ROS
```bash
cd ~/capstone_ws
source install/setup.bash
ros2 launch camera_path_ik preview.launch.py arm:=serial port:=/dev/ttyACM0
```
RViz and the path editor open. In the editor's **Arm** row:
1. The status should no longer say **Not connected**. If it does, see Troubleshooting.
2. **Put the arm in its home position by hand:** Theta 0, **Z at the bottom**, **R fully in**,
   Yaw 0, Pitch 0.
3. **Motors on**, then **Set home here**.
4. **Manual jogging**: move each joint a small step. Check the direction, and that the
   distance roughly matches what you asked for.
5. Load or make a slow path, then **Run on arm**. **Stop** (or Esc) stops it at any time.

The solid arm in RViz is where the board is. The see-through ghost is the preview.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| No `/dev/ttyACM0` | Run `usbipd attach` again (C1). Check the BOOT0 jumper is off and the board was reset. |
| `Permission denied` on the port | B3 not done, or WSL not restarted since. |
| Arm row says **Not connected** | miniterm still open, board not attached, or wrong `port:=`. |
| `INFO` clock isn't 180000000 | Tell the firmware team (clock setup issue). |
| `DRIVERS` says `NO ANSWER` | Motor power off, wrong mode jumpers under the driver (SPI for Driver4–7, UART for Driver1–3), or the driver isn't seated. |
| A joint moves the wrong way | Note which one. Firmware fix: `invert_dir` in the Settings section of `firmware/arm_firmware/arm_firmware.ino`, then re-flash. |
| A joint moves the wrong distance | `steps_per_unit` in `src/arm_bridge/config/arm.yaml` (placeholders for now). Change it and relaunch; no re-flash needed. |
| The arm stops mid-move | The watchdog stops the arm if the PC goes quiet for 250 ms (a USB hiccup). Clear fault, then Set home here again. |
| Arduino IDE can't find the board to upload | BOOT0 fitted and reset pressed? Board not attached to WSL (`usbipd detach --hardware-id 0483:5740`)? |

---

## Test record (fill in and send back)

```
Date:                 Tester:
Board / revision:     Firmware commit (git log -1 --oneline):

Flash
[ ] Upload succeeded                      notes:

Bench (miniterm)
[ ] PING replied
[ ] INFO sysclk = ____________
[ ] DRIVERS: THETA __  Z1 __  Z2 __  R __  YAW __  PITCH __   (ok / NO ANSWER)
[ ] SPIN works:  THETA __  Z __  R __  YAW __  PITCH __        (y / n / not tried)
[ ] Z1 and Z2 turn the same way           (y / n)

ROS
[ ] Arm row connected
[ ] Set home here OK
Jog checks (asked vs. measured, direction right?):
  THETA  asked ______  moved ______  dir ok? __
  Z      asked ______  moved ______  dir ok? __
  R      asked ______  moved ______  dir ok? __
  YAW    asked ______  moved ______  dir ok? __
  PITCH  asked ______  moved ______  dir ok? __
[ ] Ran a path                            speed setting: ______
INFO tick_max after a fast run = ______  (budget 3600)

Problems / error messages (copy exact text):
```
