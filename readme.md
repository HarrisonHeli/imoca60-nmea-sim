# IMOCA 60 NMEA Sailboat Simulator

A high-fidelity, physics-based NMEA 0183 sailboat simulator written in Python. Designed specifically to interface with OpenCPN for marine software development, plugin testing, and virtual sailing. 

## Features

* **Realistic Sailing Physics:** Utilizes a true IMOCA 60 polar diagram to calculate Speed Over Ground (SOG) based on True Wind Speed and True Wind Angle.
* **Dynamic Environment:** Fetches real-world weather (True Wind Speed, True Wind Direction, Temperature, Pressure) for the boat's current coordinates via the free Open-Meteo API.
* **Vessel Dynamics:** Simulates realistic vessel behavior including heel (roll), pitch, and wave states.
* **Weather Helm & Autopilot:** Features a dynamic autopilot that calculates required rudder angles to counteract wind-induced weather helm. 
* **Tactics Plugin Compatibility:** Outputs calculated Apparent Wind and faked Speed Through Water (`$IIVHW`) to perfectly sync with OpenCPN's Tactics and Dashboard plugins.
* **Virtual AIS:** Generates dynamic AIS targets (Message Type 1) sailing in the vicinity.
* **Network Broadcasting:** Supports simultaneous TCP (for bidirectional autopilot control via `$xxAPB`) and UDP Broadcasting for network-wide telemetry sharing.
* **State Persistence:** Automatically saves and loads vessel state and environment parameters via JSON.

## Installation

1. Ensure you have Python 3 installed.
2. Install the required external library for weather fetching:
```bash
pip install requests
```

## Usage

Run the script from your terminal:

```bash
python imoca_sim.py
```

**Using Custom Config Files:**
You can maintain multiple saved states (e.g., different race starting lines or network configurations) by specifying a config file at launch:

```bash
python imoca_sim.py --config transat_jacques_vabre.json
```

## Connecting to OpenCPN

**For Local Testing (Bidirectional Autopilot):**
1. Open OpenCPN > Options > Connections.
2. Add a **TCP** connection.
3. Address: `127.0.0.1`, Port: `10110` (or your configured port).
4. Check both **"Receive input on this port"** and **"Output on this port (as autopilot)"**.

**For Network/Tablet Broadcasting:**
1. In the simulator GUI, set the UDP IP to your network's broadcast address (e.g., `192.168.1.255` or `255.255.255.255`).
2. On the receiving OpenCPN device, add a **UDP** connection.
3. Address: `0.0.0.0`, Port: `10111` (or your configured UDP port).
4. Check **"Receive input on this port"**.

## Supported NMEA 0183 Sentences

**Transmitted:**
* `$GPRMC` (Position, SOG, COG)
* `$IIMWV` (Apparent Wind)
* `$IIVHW` (Water Speed and Heading)
* `$IIMDA` (Meteorological Composite - Pressure, Temp)
* `$IIMTW` (Water Temperature)
* `$IIDPT` (Depth)
* `$IIHDG` (Heading)
* `$IIXDR` (Transducer - Pitch, Roll)
* `$IIRSA` (Rudder Sensor Angle)
* `!AIVDM` (AIS Targets)

**Received:**
* `$xxAPB` (Autopilot Sentence B - Target Heading)

## Credits
* **Concept & Architecture:** Harry
* **Lead Developer:** Gemini AI