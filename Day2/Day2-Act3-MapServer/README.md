# Activity 3 — 2D ToF Map Server

Receives 360° polar scans from the ESP32 (`Day2-Act3/Day2-Act3.ino`)
over Wi-Fi and draws them on a canvas map.

Python standard library only — no `requirements.txt`, no pip step.

## The network

Robot and laptop both join the same existing Wi-Fi:

| | |
|---|---|
| SSID | `GA25LM` |
| Password | `A1B2C3` |
| Robot | DHCP, also announces itself as `tof-robot.local` |
| This laptop | DHCP — found by name, see below |

## Run it

```bash
docker compose up --build -d      # from this directory
```

1. Put this laptop on `GA25LM`.
2. Start the container.
3. Power up the robot (or press its button) and scan.

Open **http://localhost:8080/** here, or `http://hp-pavilion.local:8080/`
from a phone on the same network.

```bash
docker compose logs -f            # watch scans arrive
docker compose down               # stop
```

Without Docker: `python3 server.py` (Python 3.8+, nothing to install).

## How the robot finds this machine

It resolves **`hp-pavilion.local`** over mDNS, then confirms with
`GET /api/health` before trusting the address — a name that resolves
is not proof the container is running.

mDNS rather than a hardcoded IP because this laptop's address is a
DHCP lease that changes between sessions, while its hostname does
not. `avahi-daemon` answers those queries and is already running
here; verify with:

```bash
systemctl is-active avahi-daemon
avahi-resolve -n hp-pavilion.local
```

⚠️ **Right now that returns `172.18.0.1` — a Docker bridge address**,
not a LAN one. Avahi publishes every interface it can see, including
`docker0` and the compose network's `br-*`. It normally answers each
query with the address of the interface the query arrived on, so a
robot on Wi-Fi should still get the Wi-Fi address — but if the serial
log shows the robot resolving a `172.x` address, that is what
happened. Two fixes:

```bash
# preferred: stop avahi advertising the docker bridges
echo 'deny-interfaces=docker0' | sudo tee -a /etc/avahi/avahi-daemon.conf
sudo systemctl restart avahi-daemon
```

or set `SERVER_HOST_IP` in the sketch to the address from
`hostname -I` and skip mDNS. The sketch prints the resolved address
and warns when it is not on the robot's own subnet, so this failure
is visible rather than silent.

To pin an address instead, set `SERVER_HOST_IP` in `Day2-Act3.ino`
(mDNS is then skipped entirely). Send `d` on the Serial Monitor to
force re-discovery, `w` for Wi-Fi status.

If discovery fails while the container is clearly up, the usual cause
is a host firewall blocking port 8080 on the wireless interface:

```bash
sudo ufw allow 8080/tcp                        # ufw
sudo firewall-cmd --add-port=8080/tcp          # firewalld
```

## Test without the robot

```bash
curl -s localhost:8080/api/health

curl -s -X POST localhost:8080/api/scan \
  -H 'Content-Type: application/json' \
  -d '{"scan":1,"steps":8,"step_deg":45,"points":
       [[0,800],[45,1100],[90,600],[135,0],
        [180,950],[225,1200],[270,700],[315,0]]}'
```

Refresh the page — eight bearings, two of them open.

## API

| Method | Path | Body |
|---|---|---|
| POST | `/api/scan/start` | `{"scan":123,"steps":72,"step_deg":5.0,"max_range_mm":2000}` |
| POST | `/api/scan/points` | `{"scan":123,"points":[[deg,mm],…]}` |
| POST | `/api/scan/end` | `{"scan":123}` |
| POST | `/api/scan` | start + points + end in one request |
| POST | `/api/clear` | — |
| GET | `/api/map` | current scan as JSON |
| GET | `/api/health` | liveness |

`distance = 0` means **no return at that bearing** — open space. Those
bearings are kept in the payload so the map can tell "looked and saw
nothing" apart from "never looked", but they are not drawn as
obstacles.

Bearing `0°` is straight ahead of the robot and increases **clockwise**,
matching the direction the car spins.

## Notes

- The scan in progress is served as it arrives, so the map fills in
  live while the robot turns. When a scan finishes it is kept and
  redrawn until the next one starts — the map never blanks out
  mid-scan.
- State is in memory only. Restarting the container forgets the map;
  just run another scan.
- The server accepts points for a scan it never saw a `/start` for, so
  one dropped packet at the beginning does not cost the whole map.
