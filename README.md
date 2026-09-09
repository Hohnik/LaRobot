# LaRobot

LaRobot is an environment for the [YAM robot arms](https://i2rt.com/collections/yam-arm). It records motions, simulates the arms, moves them in the real world and trains a policy to mimic the motions. The setup of the arms and the cameras follows the [ABC project](https://github.com/amazon-far/abc).

This is a student project in which we want to learn how to build a robotics and AI system ourselves. The goal is to understand how the parts work together. If the robot arm does the job at the end, that is a good bonus but holds no priority over understanding how it works.

## How we work

- Keep one commit at max. 500 lines of code.
- Write every line by hand, even if you take the code from somewhere else. Then you get a chance to understand what happens, and why it happens.
- Another team member reviews your code.
- Every building block of the system is a GitHub issue. Take one, and solve it.
- Write in the issue what you learned, and why you built it the way it is.

## Installation

<details>
 <summary><b>Linux</b></summary>

### Running the application without root access

Libraries such as `pyspacemouse` communicate with the SpaceMouse through `/dev/hidraw*`. These device files are not always accessible to regular users, resulting in `Permission denied` or `Failed to open device` errors.

A udev rule provides permanent access without running the application as root.

### 1. Find the device ID

Connect the SpaceMouse and run:

```bash
lsusb
```

Example:

```text
ID 256f:c635 3Dconnexion SpaceMouse Compact
```

Here, `256f` is the vendor ID and `c635` is the product ID.

### 2. Create a udev rule

Open a new rule file:

```bash
sudoedit /etc/udev/rules.d/50-spacemouse.rules
```

Add the following, replacing the IDs if necessary:

```udev
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c635", MODE="0660", TAG+="uaccess"
```

The `50-` prefix ensures that the rule runs before systemd applies user-access permissions.

### 3. Apply the rule

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=hidraw
```

Unplug and reconnect the SpaceMouse.

### 4. Test access

```bash
pyspacemouse --list-connected
pyspacemouse --test
```

The device should now work without `sudo`.

### Headless systems

`TAG+="uaccess"` is intended for users logged into a local desktop. For SSH-only or headless systems, create a dedicated group:

```bash
sudo groupadd -f spacemouse
sudo usermod -aG spacemouse "$USER"
```

Replace `TAG+="uaccess"` in the rule with:

```udev
GROUP="spacemouse"
```

Then reload the rules, reconnect the device, and log out and back in.

Avoid tutorials that rely on the `plugdev` group: it does not exist by default on some distributions, including Arch Linux. Also avoid `MODE="0666"`, which unnecessarily gives every local user access to the device.

</details>
