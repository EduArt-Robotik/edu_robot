# Setting up your PS5 Gamepad

A joystick can be used to operate Eduard. For this purpose, the robot must be extended by a Debian-compatible Bluetooth stick. PlayStation&reg; 4 and PlayStation&reg; 5 controllers were used, which are interpreted identically in their operating interface. The initial start-up of a Bluetooth controller follows.

Start the Bluetooth control tool of the operating system:

```console
$ bluetoothctl
```

Set up the controller and prepare for scanning:

```console
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# power on
[bluetooth]# discoverable on
[bluetooth]# pairable on
```

Put the PlayStation&reg; Controller into connection mode by pressing the Share (PS5: Create) and PS buttons simultaneously. 
Rapid flashing indicates the status.

<!-- <img src="documentation/images/controller_pairing.jpg" width="500" /> <br> -->

Now start the scanning process:

```console
[bluetooth]# scan on
```

Your joystick is now recognized as a wireless controller, e.g. `[NEW] Device XX:XX:XX:XX:XX:XX DualSense Wireless Controller`. Copy the MAC address of the device for the rest of the procedure.

Connect the controller using the following commands and its MAC address. Wait a few seconds after `pair` and answer `yes` when asked. If needed, press the PlayStation button again when the light signals stop flashing.

```console
[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[bluetooth]# scan off
[bluetooth]# info XX:XX:XX:XX:XX:XX
[bluetooth]# exit
```

`info` should report `Paired: yes`, `Trusted: yes` and `Connected: yes`.

 > **Note**: These steps are only required once at the very beginning. From now on, when the PS button is pressed, the joystick should automatically connect to the Robot once it has successfully booted up. These operations are only then necessary again if the controller has been connected to another device in the meantime (see [Re-pairing a controller](#re-pairing-a-controller)).

## Re-pairing a controller

If the controller was paired with another device (e.g. a console or PC) in the meantime, it has replaced its pairing key. The robot still lists it as paired, but the connection fails. Remove the old pairing and then pair again as described above, starting with `agent on`:

```console
[bluetooth]# devices Paired
Device XX:XX:XX:XX:XX:XX DualSense Wireless Controller
[bluetooth]# remove XX:XX:XX:XX:XX:XX
```

## Useful `bluetoothctl` commands

| Command | Purpose |
|---|---|
| `show` | State of the robot's Bluetooth adapter (`Powered`, `Pairable`, …) |
| `devices` / `devices Paired` | List known / paired devices (older BlueZ versions: `paired-devices`) |
| `info <MAC>` | Device details: `Paired`, `Trusted`, `Connected` |
| `connect <MAC>` / `disconnect <MAC>` | Connect / disconnect a paired device, e.g. if it does not reconnect by itself |
| `remove <MAC>` | Delete a pairing completely |

Single commands can also be run directly from bash, e.g. `bluetoothctl info XX:XX:XX:XX:XX:XX`.

## Troubleshooting

- **Bluetooth service:** `systemctl status bluetooth` must show `active (running)`.
- **Adapter blocked:** check with `rfkill list`, unblock with `rfkill unblock bluetooth`.
- **Adapter off after every reboot** (`show` reports `Powered: no`): set `AutoEnable=true` in section `[Policy]` of `/etc/bluetooth/main.conf`. Otherwise the controller cannot connect automatically after boot.
