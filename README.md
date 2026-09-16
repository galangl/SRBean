# SR900 <-> HiBean Bridge

ESP32-S3 firmware that talks to your real SR900 roaster using its actual
(fully cracked) protocol, and exposes a simple custom BLE service that
HiBean's "Custom TC4" device type can connect to.

## Build and flash

Same process as the earlier project — this reuses the same ESP-IDF
installation and toolchain fix.

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\idf_env.ps1 set-target esp32s3
.\idf_env.ps1 build
.\idf_env.ps1 -p COM3 flash monitor
```

If `set-target` fails with a Clang-related error, see the toolchain fix
notes from the original ble_mitm_bridge project — the same fix to
`C:\esp\v6.1\esp-idf\tools\cmake\toolchain.cmake` applies here too, since
it's the same shared ESP-IDF installation.

## What it does

1. **Connects to the real roaster** exactly like the official app does:
   scans for `SR900-FDB666`, connects, subscribes to telemetry, sends the
   real MAC address handshake, computes the session token, and sends the
   required settings frame.
2. **Advertises itself separately** as `ESP32-SR900-Bridge` with its own
   simple custom BLE service, for HiBean to connect to.
3. **Translates** simple text commands from HiBean into real, validly
   signed SR900 protocol frames, and real roaster telemetry into a simple
   comma-separated text line for HiBean.

## Configuring HiBean (Custom TC4)

In HiBean: **Devices → + → Custom TC4**

### Transport
- Type: **BLE**
- Select device: **ESP32-SR900-Bridge**
- Service UUID: `7a9e0001-1234-4a5e-8b3d-9f1e2c3a4b50`
- Write characteristic UUID: `7a9e0003-1234-4a5e-8b3d-9f1e2c3a4b50`
- Notify characteristic UUID: `7a9e0002-1234-4a5e-8b3d-9f1e2c3a4b50`

### Status frame
- Status command: leave blank / not needed (status is pushed via notify automatically every ~2s once the roaster connection is live)
- Delimiter: `,`
- Message terminator: `\n`
- Example frame: `375.0,382.0,5,4`
  - Field 0 (index 0) = **BT** (required)
  - Field 1 (index 1) = ET
  - Field 2 (index 2) = fan feedback
  - Field 3 (index 3) = heater feedback

### Controls (optional — enable only once you've confirmed status works)
- **Heat**: template `HEAT;{value}`, range 0-9, step 1, test value e.g. 3, safe value 0, feedback field = index 3 (heater)
- **Fan**: template `FAN;{value}`, range 0-9, step 1, test value e.g. 3, safe value 0, feedback field = index 2 (fan)

Note: `START`, `STOP`, and `COOL` commands are implemented in the firmware
(sent as plain text with no `{value}`) but aren't exposed as HiBean
"controls" in this default setup — HiBean's Custom TC4 control model is
built around single numeric-range commands. You can trigger those three
manually for testing by writing the literal text `START`, `STOP`, or
`COOL` to the command characteristic (e.g. via nRF Connect), or extend
the firmware/HiBean config further if you want them exposed as buttons.

## Important safety note on the START command

The firmware's default `START` handler uses fixed values (10 min roast,
4 min cool, heat=5, fan=5) — see `handle_hibean_command()` in the source
if you want to change these defaults, or wire up a way to set them from
HiBean before starting.

## Verifying it works

1. Flash and open the serial monitor.
2. Power on the roaster — watch for the full handshake sequence in the log:
   `Found real roaster` → `Connected` → `Found DF01/DF02` → `Subscribed`
   → `Sending MAC request` → `Got MAC ... token=...` → `Sending settings`
   → `SETTINGS ACKED` → periodic `STATUS fan=... heater=... BT=...F ET=...F`
3. Connect HiBean to `ESP32-SR900-Bridge` and confirm status reads populate.
4. Only after status is confirmed working, test a control (start with Heat
   or Fan, at a safe idle value) before trusting it during a real roast.
