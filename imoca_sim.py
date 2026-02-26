import tkinter as tk
from tkinter import ttk
import socket
import threading
import time
import math
import random
import requests
import json
import os
import argparse
from datetime import datetime

# --- Internal Fallback Polar (High-Res IMOCA 60) ---
FALLBACK_POLAR = {
    5.0:  {0: 0.0, 35: 4.5, 45: 6.0, 60: 7.0, 75: 7.5, 90: 8.0, 105: 8.0, 120: 7.5, 135: 6.5, 150: 5.5, 165: 4.5, 180: 4.0},
    10.0: {0: 0.0, 35: 8.5, 45: 10.0, 60: 11.5, 75: 12.5, 90: 14.0, 105: 14.5, 120: 14.0, 135: 12.0, 150: 10.5, 165: 9.0, 180: 8.0},
    15.0: {0: 0.0, 35: 10.5, 45: 12.5, 60: 15.0, 75: 18.0, 90: 21.0, 105: 23.0, 120: 22.0, 135: 18.5, 150: 15.0, 165: 12.5, 180: 11.0},
    20.0: {0: 0.0, 35: 11.5, 45: 14.5, 60: 18.0, 75: 23.0, 90: 27.0, 105: 29.0, 120: 28.0, 135: 24.0, 150: 19.0, 165: 15.0, 180: 13.0},
    25.0: {0: 0.0, 35: 12.0, 45: 15.5, 60: 20.0, 75: 26.0, 90: 31.0, 105: 34.0, 120: 33.0, 135: 28.0, 150: 23.0, 165: 18.0, 180: 15.0},
    30.0: {0: 0.0, 35: 12.5, 45: 16.0, 60: 21.0, 75: 27.0, 90: 33.0, 105: 37.0, 120: 36.0, 135: 31.0, 150: 26.0, 165: 21.0, 180: 17.0},
    35.0: {0: 0.0, 35: 12.0, 45: 15.5, 60: 20.5, 75: 26.5, 90: 32.5, 105: 36.0, 120: 35.0, 135: 32.0, 150: 28.0, 165: 23.0, 180: 18.0},
    40.0: {0: 0.0, 35: 10.0, 45: 13.0, 60: 18.0, 75: 24.0, 90: 29.0, 105: 32.0, 120: 31.0, 135: 29.0, 150: 26.0, 165: 21.0, 180: 17.0},
    45.0: {0: 0.0, 35: 8.0, 45: 10.0, 60: 14.0, 75: 19.0, 90: 24.0, 105: 27.0, 120: 26.0, 135: 25.0, 150: 23.0, 165: 19.0, 180: 15.0},
    50.0: {0: 0.0, 35: 5.0, 45: 7.0, 60: 10.0, 75: 14.0, 90: 18.0, 105: 21.0, 120: 21.0, 135: 21.0, 150: 19.0, 165: 16.0, 180: 13.0},
    55.0: {0: 0.0, 35: 2.0, 45: 4.0, 60: 7.0, 75: 10.0, 90: 13.0, 105: 16.0, 120: 16.0, 135: 17.0, 150: 16.0, 165: 14.0, 180: 11.0},
    60.0: {0: 0.0, 35: 0.0, 45: 2.0, 60: 4.0, 75: 7.0, 90: 10.0, 105: 12.0, 120: 13.0, 135: 14.0, 150: 14.0, 165: 12.0, 180: 10.0}
}

def load_polar_file(filepath):
    polar = {}
    try:
        if not os.path.exists(filepath):
            return None
            
        with open(filepath, 'r') as f:
            lines = f.readlines()
            
        tws_headers = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
                
            parts = line.split()
            if 'TWA/TWS' in parts[0].upper() or 'TWA\\TWS' in parts[0].upper():
                tws_headers = [float(x) for x in parts[1:]]
                for tws in tws_headers:
                    polar[tws] = {}
                continue
                
            if tws_headers and len(parts) > 1:
                twa = float(parts[0])
                for i, speed_str in enumerate(parts[1:]):
                    if i < len(tws_headers):
                        polar[tws_headers[i]][twa] = float(speed_str)
                        
        return polar if polar else None
    except Exception as e:
        print(f"Failed to parse polar file: {e}")
        return None

def get_polar_speed(tws, twa, active_polar):
    twa = abs(twa) % 360
    if twa > 180: twa = 360 - twa
    
    tws_keys = sorted(active_polar.keys())
    if tws <= tws_keys[0]: tws_low, tws_high = tws_keys[0], tws_keys[0]
    elif tws >= tws_keys[-1]: tws_low, tws_high = tws_keys[-1], tws_keys[-1]
    else:
        tws_low = max([k for k in tws_keys if k <= tws])
        tws_high = min([k for k in tws_keys if k >= tws])

    twa_keys = sorted(active_polar[tws_keys[0]].keys())
    if twa <= twa_keys[0]: twa_low, twa_high = twa_keys[0], twa_keys[0]
    elif twa >= twa_keys[-1]: twa_low, twa_high = twa_keys[-1], twa_keys[-1]
    else:
        twa_low = max([k for k in twa_keys if k <= twa])
        twa_high = min([k for k in twa_keys if k >= twa])

    def interp1d(x, x1, x2, y1, y2):
        if x1 == x2: return y1
        return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)

    speed_low_tws = interp1d(twa, twa_low, twa_high, active_polar[tws_low][twa_low], active_polar[tws_low][twa_high])
    speed_high_tws = interp1d(twa, twa_low, twa_high, active_polar[tws_high][twa_low], active_polar[tws_high][twa_high])
    return interp1d(tws, tws_low, tws_high, speed_low_tws, speed_high_tws)

def generate_checksum(sentence):
    calc = 0
    start = 1 if sentence[0] in ['$', '!'] else 0
    for char in sentence[start:]:
        if char == '*': break
        calc ^= ord(char)
    return f"{calc:02X}"

def format_lat_lon(dec_deg, is_lat):
    degrees = int(abs(dec_deg))
    minutes = (abs(dec_deg) - degrees) * 60
    if is_lat:
        direction = 'N' if dec_deg >= 0 else 'S'
        return f"{degrees:02d}{minutes:07.4f},{direction}"
    else:
        direction = 'E' if dec_deg >= 0 else 'W'
        return f"{degrees:03d}{minutes:07.4f},{direction}"

def int_to_bin(val, bits):
    val = int(val)
    if val < 0:
        val = (1 << bits) + val 
    return f"{val:0{bits}b}"

def to_6bit_ascii(bit_str):
    while len(bit_str) % 6 != 0:
        bit_str += '0'
    res = ""
    for i in range(0, len(bit_str), 6):
        val = int(bit_str[i:i+6], 2)
        if val < 40:
            res += chr(val + 48)
        else:
            res += chr(val + 56)
    return res

class VirtualShip:
    def __init__(self, mmsi, lat, lon, cog, sog):
        self.mmsi = mmsi
        self.lat = lat
        self.lon = lon
        self.cog = cog
        self.sog = sog

    def update(self, dt):
        dist_nm = self.sog * (dt / 3600.0)
        self.lat += (dist_nm * math.cos(math.radians(self.cog))) / 60.0
        self.lon += (dist_nm * math.sin(math.radians(self.cog))) / (60.0 * math.cos(math.radians(self.lat)))

    def generate_aivdm(self):
        payload = ""
        payload += int_to_bin(1, 6) 
        payload += int_to_bin(0, 2) 
        payload += int_to_bin(self.mmsi, 30)
        payload += int_to_bin(0, 4) 
        payload += int_to_bin(0, 8) 
        payload += int_to_bin(min(self.sog, 102.2) * 10, 10)
        payload += int_to_bin(1, 1) 
        payload += int_to_bin(self.lon * 600000, 28)
        payload += int_to_bin(self.lat * 600000, 27)
        payload += int_to_bin(self.cog * 10, 12)
        payload += int_to_bin(self.cog, 9) 
        payload += int_to_bin(60, 6) 
        payload += int_to_bin(0, 2) 
        payload += int_to_bin(0, 3) 
        payload += int_to_bin(0, 1) 
        payload += int_to_bin(0, 19) 
        
        ascii_payload = to_6bit_ascii(payload)
        sentence = f"!AIVDM,1,1,,A,{ascii_payload},0"
        return f"{sentence}*{generate_checksum(sentence)}\r\n"

class SailboatSim:
    def __init__(self, ui):
        self.ui = ui
        self.running = False
        self.server_socket = None
        self.udp_socket = None
        self.client_sockets = []
        
        external_polar = load_polar_file("imoca60.pol")
        if external_polar:
            self.active_polar = external_polar
            print("Successfully loaded external imoca60.pol")
        else:
            self.active_polar = FALLBACK_POLAR
            print("Using internal fallback polar")
        
        # Boat State
        self.lat = -32.1 
        self.lon = 115.4
        self.heading = 270.0
        self.target_heading = 270.0
        self.sog = 0.0
        self.rudder_angle = 0.0
        self.turn_rate = 0.0
        self.cog = 270.0 # Track actual direction of movement
        
        # Environment State
        self.base_tws = 15.0
        self.twd = 180.0
        self.temp_c = 20.0
        self.pressure_hpa = 1013.0
        self.sea_state = 2.0
        self.gust_multiplier = 1.2
        self.pitch = 0.0
        self.roll = 0.0
        self.water_temp = 22.0
        self.depth_base = 150.0
        self.depth_current = 150.0
        self.aws = 0.0
        self.awa = 0.0
        
        self.last_weather_update = 0
        self.ais_targets = []
        
        self.tcp_port_str = ""
        self.udp_ip_str = ""
        self.udp_port_str = ""

    def start(self):
        self.running = True
        self.tcp_port_str = self.ui.tcp_port_var.get()
        self.udp_ip_str = self.ui.udp_ip_var.get()
        self.udp_port_str = self.ui.udp_port_var.get()
        
        self.tcp_port = int(self.tcp_port_str)
        self.udp_port = int(self.udp_port_str)
        
        self.lat = float(self.ui.lat_var.get())
        self.lon = float(self.ui.lon_var.get())
        self.target_heading = float(self.ui.heading_var.get())
        self.heading = self.target_heading
        self.cog = self.heading
        
        self.ais_targets = [
            VirtualShip(211000000, self.lat + 0.05, self.lon + 0.05, 180, 12.5),
            VirtualShip(311000000, self.lat - 0.02, self.lon + 0.10, 290, 18.0),
            VirtualShip(411000000, self.lat + 0.08, self.lon - 0.04, 90, 8.0)
        ]

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.last_weather_update = 0 
        
        threading.Thread(target=self.tcp_server_thread, daemon=True).start()
        threading.Thread(target=self.physics_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.server_socket: self.server_socket.close()
        if self.udp_socket: self.udp_socket.close()
        for client in self.client_sockets: client.close()
        self.client_sockets.clear()

    def fetch_weather(self):
        def fetch():
            try:
                self.ui.log_msg("System", "Fetching weather data from Open-Meteo...")
                url = f"https://api.open-meteo.com/v1/forecast?latitude={self.lat}&longitude={self.lon}&current=temperature_2m,surface_pressure,wind_speed_10m,wind_direction_10m&wind_speed_unit=kn"
                response = requests.get(url, timeout=5).json()
                current = response['current']
                
                self.base_tws = current['wind_speed_10m']
                self.twd = current['wind_direction_10m']
                self.temp_c = current['temperature_2m']
                self.pressure_hpa = current['surface_pressure']
                
                self.ui.log_msg("System", f"Weather updated: {self.base_tws}kn @ {self.twd}°, {self.temp_c}°C")
                self.ui.tws_var.set(f"{self.base_tws:.1f}")
                self.ui.twd_var.set(f"{self.twd:.1f}")
            except Exception as e:
                self.ui.log_msg("Error", f"Failed to fetch weather: {e}")
        threading.Thread(target=fetch, daemon=True).start()

    def save_state_from_thread(self):
        data = {
            "tcp_port": self.tcp_port_str,
            "udp_ip": self.udp_ip_str,
            "udp_port": self.udp_port_str,
            "lat": f"{self.lat:.4f}",
            "lon": f"{self.lon:.4f}",
            "heading": f"{self.heading:.1f}",
            "sea_state": self.sea_state,
            "gust": self.gust_multiplier,
            "water_temp": self.water_temp,
            "depth": self.depth_base,
            "send_tws": self.ui.send_tws_var.get(),
            "anchor_mode": self.ui.anchor_mode_var.get()
        }
        try:
            with open(self.ui.config_file, "w") as f:
                json.dump(data, f, indent=4)
        except:
            pass 

    def physics_loop(self):
        dt = 0.5 
        t = 0
        tick_count = 0
        
        while self.running:
            current_time = time.time()
            if current_time - self.last_weather_update > 600: 
                self.fetch_weather()
                self.last_weather_update = current_time

            self.sea_state = float(self.ui.sea_state_var.get())
            self.gust_multiplier = float(self.ui.gust_var.get())
            self.water_temp = float(self.ui.water_temp_var.get())
            self.depth_base = float(self.ui.depth_var.get())
            
            self.depth_current = self.depth_base + (math.sin(t * 0.2) * self.sea_state * 0.5)

            current_tws = self.base_tws
            if random.random() < 0.2: 
                gust = (random.random() * (self.gust_multiplier - 1.0)) * self.base_tws
                current_tws += gust

            is_anchored = self.ui.anchor_mode_var.get()

            if is_anchored:
                # --- ANCHOR MODE PHYSICS ---
                self.sog = current_tws * 0.02 # Drift speed scales with wind
                # Bow swings into the wind with some wandering
                self.heading = (self.twd - 180 + (math.sin(t * 0.4) * 20)) % 360
                self.cog = self.twd # Movement is directly downwind
                
                twa_rel = (self.twd - self.heading + 360) % 360
                twa_rad = math.radians(twa_rel)
                wind_x = current_tws * math.cos(twa_rad)
                wind_y = current_tws * math.sin(twa_rad)
                
                # Because we are drifting backwards, headwind decreases slightly
                app_x = wind_x - self.sog 
                app_y = wind_y
                
                self.rudder_angle = 0.0
                self.roll = math.sin(t * 1.5) * self.sea_state * 0.5
                self.pitch = math.cos(t * 2.0) * self.sea_state * 0.5

            else:
                # --- SAILING PHYSICS ---
                twa_rel = (self.twd - self.heading + 360) % 360
                self.sog = get_polar_speed(current_tws, twa_rel, self.active_polar)
                self.cog = self.heading # Movement is directly forward
                
                twa_rad = math.radians(twa_rel)
                wind_x = current_tws * math.cos(twa_rad)
                wind_y = current_tws * math.sin(twa_rad)
                
                # Sailing forward increases headwind
                app_x = wind_x + self.sog 
                app_y = wind_y

                twa_sym = twa_rel if twa_rel <= 180 else twa_rel - 360
                heel_factor = math.sin(math.radians(abs(twa_sym))) 
                base_roll = heel_factor * (current_tws * 0.8) 
                wave_roll = math.sin(t * 1.5) * self.sea_state * 2.0
                self.roll = (base_roll + wave_roll) * (1 if twa_sym > 0 else -1)
                self.pitch = math.cos(t * 2.0) * self.sea_state * (self.sog / 10.0)

                weather_helm_force = self.roll * (self.sog / 15.0) * 0.4
                natural_turn_rate = -weather_helm_force 
                hdg_error = (self.target_heading - self.heading + 180) % 360 - 180
                rudder_target = (hdg_error * 2.0) + (weather_helm_force * 1.5)
                
                rudder_diff = rudder_target - self.rudder_angle
                self.rudder_angle += math.copysign(min(15.0 * dt, abs(rudder_diff)), rudder_diff)
                self.rudder_angle = max(-35.0, min(35.0, self.rudder_angle))

                rudder_effectiveness = self.rudder_angle * 0.2 * (self.sog / 10.0 + 0.1)
                self.turn_rate = rudder_effectiveness + natural_turn_rate
                
                self.heading += self.turn_rate * dt
                self.heading %= 360

            # --- SHARED UPDATES ---
            self.aws = math.sqrt(app_x**2 + app_y**2)
            self.awa = math.degrees(math.atan2(app_y, app_x)) % 360

            dist_nm = self.sog * (dt / 3600.0)
            self.lat += (dist_nm * math.cos(math.radians(self.cog))) / 60.0
            self.lon += (dist_nm * math.sin(math.radians(self.cog))) / (60.0 * math.cos(math.radians(self.lat)))

            for ship in self.ais_targets:
                ship.update(dt)

            self.broadcast_nmea(current_tws, twa_rel, tick_count)
            self.ui.update_readouts(self.sog, self.heading, self.pitch, self.roll, self.lat, self.lon, self.aws, self.awa, self.rudder_angle)
            
            if tick_count % 10 == 0 and tick_count > 0:
                self.save_state_from_thread()
            
            t += dt
            tick_count += 1
            time.sleep(dt)

    def broadcast_nmea(self, tws, twa_rel, tick_count):
        now = datetime.utcnow()
        time_str = now.strftime("%H%M%S.00")
        date_str = now.strftime("%d%m%y")
        
        sentences = []
        
        # Notice we are now sending self.cog in field 8 instead of self.heading
        lat_str = format_lat_lon(self.lat, True)
        lon_str = format_lat_lon(self.lon, False)
        rmc_core = f"GPRMC,{time_str},A,{lat_str},{lon_str},{self.sog:.1f},{self.cog:.1f},{date_str},,,"
        sentences.append(f"${rmc_core}*{generate_checksum(rmc_core)}\r\n")

        gsa_core = "GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,,,1.5,1.0,1.0"
        sentences.append(f"${gsa_core}*{generate_checksum(gsa_core)}\r\n")

        gsv1_core = "GPGSV,3,1,10,01,45,090,45,02,75,180,50,03,30,270,40,04,60,045,48"
        sentences.append(f"${gsv1_core}*{generate_checksum(gsv1_core)}\r\n")
        gsv2_core = "GPGSV,3,2,10,05,80,315,52,06,20,135,38,07,55,225,44,08,40,300,42"
        sentences.append(f"${gsv2_core}*{generate_checksum(gsv2_core)}\r\n")
        gsv3_core = "GPGSV,3,3,10,09,65,015,49,10,25,100,39"
        sentences.append(f"${gsv3_core}*{generate_checksum(gsv3_core)}\r\n")

        # Optional True Wind broadcast based on UI toggle
        if self.ui.send_tws_var.get():
            mwv_t_core = f"IIMWV,{twa_rel:.1f},T,{tws:.1f},N,A"
            sentences.append(f"${mwv_t_core}*{generate_checksum(mwv_t_core)}\r\n")

        mwv_r_core = f"IIMWV,{self.awa:.1f},R,{self.aws:.1f},N,A"
        sentences.append(f"${mwv_r_core}*{generate_checksum(mwv_r_core)}\r\n")

        press_in = self.pressure_hpa * 0.02953
        press_bar = self.pressure_hpa / 1000.0
        mda_core = f"IIMDA,{press_in:.2f},I,{press_bar:.3f},B,{self.temp_c:.1f},C,,,,,,,,,,,,"
        sentences.append(f"${mda_core}*{generate_checksum(mda_core)}\r\n")

        mtw_core = f"IIMTW,{self.water_temp:.1f},C"
        sentences.append(f"${mtw_core}*{generate_checksum(mtw_core)}\r\n")

        dpt_core = f"IIDPT,{self.depth_current:.1f},0.0"
        sentences.append(f"${dpt_core}*{generate_checksum(dpt_core)}\r\n")

        hdg_core = f"IIHDG,{self.heading:.1f},,,,"
        sentences.append(f"${hdg_core}*{generate_checksum(hdg_core)}\r\n")

        vhw_core = f"IIVHW,{self.heading:.1f},T,,M,{self.sog:.1f},N,,K"
        sentences.append(f"${vhw_core}*{generate_checksum(vhw_core)}\r\n")

        xdr_core = f"IIXDR,A,{self.pitch:.1f},D,PITCH,A,{self.roll:.1f},D,ROLL"
        sentences.append(f"${xdr_core}*{generate_checksum(xdr_core)}\r\n")

        rsa_core = f"IIRSA,{self.rudder_angle:.1f},A,,,"
        sentences.append(f"${rsa_core}*{generate_checksum(rsa_core)}\r\n")

        if tick_count % 6 == 0:
            for ship in self.ais_targets:
                sentences.append(ship.generate_aivdm())

        data = "".join(sentences).encode('ascii')
        
        for client in self.client_sockets[:]:
            try:
                client.send(data)
            except:
                self.client_sockets.remove(client)
                
        if self.udp_ip_str:
            try:
                self.udp_socket.sendto(data, (self.udp_ip_str, self.udp_port))
            except Exception:
                pass 
        
        if random.random() < 0.2: 
            self.ui.log_nmea_out(sentences[0].strip()) 

    def tcp_server_thread(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', int(self.tcp_port_str)))
        self.server_socket.listen(5)
        self.ui.log_msg("System", f"TCP Server listening on port {self.tcp_port_str}")
        
        while self.running:
            try:
                client, addr = self.server_socket.accept()
                self.client_sockets.append(client)
                self.ui.log_msg("System", f"TCP Client connected: {addr}")
                threading.Thread(target=self.client_handler, args=(client,), daemon=True).start()
            except:
                break

    def client_handler(self, client):
        buffer = ""
        while self.running:
            try:
                data = client.recv(1024).decode('ascii')
                if not data: break
                buffer += data
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.ui.log_nmea_in(line)
                        self.parse_incoming_nmea(line)
            except:
                break

    def parse_incoming_nmea(self, sentence):
        if sentence.startswith('$') and '*' in sentence:
            payload = sentence[1:sentence.index('*')]
            parts = payload.split(',')
            if parts[0].endswith('APB'):
                try:
                    target_hdg = float(parts[13])
                    self.target_heading = target_hdg
                    self.ui.heading_var.set(f"{target_hdg:.1f}")
                except (IndexError, ValueError):
                    pass

class SimGUI:
    def __init__(self, root, config_file="sim_state.json"):
        self.root = root
        self.root.title("IMOCA 60 Simulator - Network Pro")
        self.config_file = config_file
        self.setup_ui()
        self.load_config() 
        self.sim = SailboatSim(self)

    def load_config(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)
                    self.tcp_port_var.set(data.get("tcp_port", "10110"))
                    self.udp_ip_var.set(data.get("udp_ip", "127.0.0.1"))
                    self.udp_port_var.set(data.get("udp_port", "10111"))
                    self.lat_var.set(data.get("lat", "-32.1000"))
                    self.lon_var.set(data.get("lon", "115.4000"))
                    self.heading_var.set(data.get("heading", "270"))
                    self.sea_state_var.set(float(data.get("sea_state", 1.5)))
                    self.gust_var.set(float(data.get("gust", 1.3)))
                    self.water_temp_var.set(float(data.get("water_temp", 22.0)))
                    self.depth_var.set(float(data.get("depth", 150.0)))
                    self.send_tws_var.set(data.get("send_tws", False))
                    self.anchor_mode_var.set(data.get("anchor_mode", False))
                    self.log_msg("System", f"Saved state loaded from {self.config_file}")
            except Exception as e:
                self.log_msg("Error", f"Could not load state: {e}")

    def save_config(self):
        data = {
            "tcp_port": self.tcp_port_var.get(),
            "udp_ip": self.udp_ip_var.get(),
            "udp_port": self.udp_port_var.get(),
            "lat": self.lat_var.get(),
            "lon": self.lon_var.get(),
            "heading": self.heading_var.get(),
            "sea_state": self.sea_state_var.get(),
            "gust": self.gust_var.get(),
            "water_temp": self.water_temp_var.get(),
            "depth": self.depth_var.get(),
            "send_tws": self.send_tws_var.get(),
            "anchor_mode": self.anchor_mode_var.get()
        }
        try:
            with open(self.config_file, "w") as f:
                json.dump(data, f, indent=4)
            self.log_msg("System", f"Settings saved to {self.config_file}")
        except Exception as e:
            self.log_msg("Error", f"Could not save state: {e}")

    def setup_ui(self):
        ctrl_frame = ttk.LabelFrame(self.root, text="Parameters & Network")
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.tcp_port_var = tk.StringVar(value="10110")
        self.udp_ip_var = tk.StringVar(value="127.0.0.1") 
        self.udp_port_var = tk.StringVar(value="10111")
        self.lat_var = tk.StringVar(value="-32.100") 
        self.lon_var = tk.StringVar(value="115.400")
        self.heading_var = tk.StringVar(value="270")
        self.sea_state_var = tk.DoubleVar(value=1.5)
        self.gust_var = tk.DoubleVar(value=1.3)
        self.water_temp_var = tk.DoubleVar(value=22.0)
        self.depth_var = tk.DoubleVar(value=150.0)
        self.tws_var = tk.StringVar(value="15.0")
        self.twd_var = tk.StringVar(value="180.0")
        self.send_tws_var = tk.BooleanVar(value=False)
        self.anchor_mode_var = tk.BooleanVar(value=False)

        # Row 0
        ttk.Label(ctrl_frame, text="TCP Port:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(ctrl_frame, textvariable=self.tcp_port_var, width=8).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(ctrl_frame, text="UDP IP:").grid(row=0, column=2, sticky=tk.E)
        ttk.Entry(ctrl_frame, textvariable=self.udp_ip_var, width=15).grid(row=0, column=3, sticky=tk.W)
        ttk.Label(ctrl_frame, text="UDP Port:").grid(row=0, column=4, sticky=tk.E)
        ttk.Entry(ctrl_frame, textvariable=self.udp_port_var, width=8).grid(row=0, column=5, sticky=tk.W)
        
        # Row 1
        ttk.Label(ctrl_frame, text="Lat:").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(ctrl_frame, textvariable=self.lat_var, width=10).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(ctrl_frame, text="Lon:").grid(row=1, column=2, sticky=tk.E)
        ttk.Entry(ctrl_frame, textvariable=self.lon_var, width=10).grid(row=1, column=3, sticky=tk.W)
        ttk.Label(ctrl_frame, text="Course (T):").grid(row=1, column=4, sticky=tk.E)
        ttk.Entry(ctrl_frame, textvariable=self.heading_var, width=8).grid(row=1, column=5, sticky=tk.W)
        
        # Row 2
        ttk.Label(ctrl_frame, text="Sea State:").grid(row=2, column=0, sticky=tk.W)
        ttk.Scale(ctrl_frame, from_=0, to=5, orient=tk.HORIZONTAL, variable=self.sea_state_var).grid(row=2, column=1)
        ttk.Label(ctrl_frame, text="Gust Mult:").grid(row=2, column=2, sticky=tk.E)
        ttk.Scale(ctrl_frame, from_=1.0, to=2.5, orient=tk.HORIZONTAL, variable=self.gust_var).grid(row=2, column=3)
        ttk.Label(ctrl_frame, text="Wind (Kn):").grid(row=2, column=4, sticky=tk.E)
        ttk.Label(ctrl_frame, textvariable=self.tws_var).grid(row=2, column=5, sticky=tk.W)

        # Row 3 (New Toggles)
        ttk.Checkbutton(ctrl_frame, text="Tx True Wind (MWV-T)", variable=self.send_tws_var).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=5)
        ttk.Checkbutton(ctrl_frame, text="Anchor Mode (Drift)", variable=self.anchor_mode_var).grid(row=3, column=2, columnspan=2, sticky=tk.W, pady=5)

        self.btn_start = ttk.Button(ctrl_frame, text="Start Sim", command=self.toggle_sim)
        self.btn_start.grid(row=0, column=6, rowspan=4, padx=10, ipady=15)

        readout_frame = ttk.LabelFrame(self.root, text="Live Telemetry")
        readout_frame.pack(fill=tk.X, padx=5, pady=0)
        self.lbl_sog = ttk.Label(readout_frame, text="SOG: 0.0 kn")
        self.lbl_sog.grid(row=0, column=0, padx=10)
        self.lbl_hdg = ttk.Label(readout_frame, text="HDG: 0.0°")
        self.lbl_hdg.grid(row=0, column=1, padx=10)
        self.lbl_rudder = ttk.Label(readout_frame, text="Rudder: 0.0°")
        self.lbl_rudder.grid(row=0, column=2, padx=10)
        self.lbl_aws = ttk.Label(readout_frame, text="AWS: 0.0 kn")
        self.lbl_aws.grid(row=0, column=3, padx=10)
        
        self.lbl_pitch = ttk.Label(readout_frame, text="Pitch: 0.0°")
        self.lbl_pitch.grid(row=1, column=0, padx=10)
        self.lbl_roll = ttk.Label(readout_frame, text="Roll: 0.0°")
        self.lbl_roll.grid(row=1, column=1, padx=10)

        log_frame = tk.Frame(self.root)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        out_frame = ttk.LabelFrame(log_frame, text="NMEA Out (Throttled)")
        out_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
        self.txt_out = tk.Text(out_frame, height=15, width=40, font=("Consolas", 8))
        self.txt_out.pack(fill=tk.BOTH, expand=True)

        in_frame = ttk.LabelFrame(log_frame, text="NMEA In (Autopilot)")
        in_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=2)
        self.txt_in = tk.Text(in_frame, height=15, width=40, font=("Consolas", 8))
        self.txt_in.pack(fill=tk.BOTH, expand=True)

    def toggle_sim(self):
        if not self.sim.running:
            self.save_config() 
            self.sim.start()
            self.btn_start.config(text="Stop Sim")
        else:
            self.sim.stop()
            self.btn_start.config(text="Start Sim")

    def update_readouts(self, sog, hdg, pitch, roll, lat, lon, aws, awa, rudder):
        self.root.after(0, lambda: self.lbl_sog.config(text=f"SOG: {sog:.1f} kn"))
        self.root.after(0, lambda: self.lbl_hdg.config(text=f"HDG: {hdg:.1f}°"))
        self.root.after(0, lambda: self.lbl_pitch.config(text=f"Pitch: {pitch:.1f}°"))
        self.root.after(0, lambda: self.lbl_roll.config(text=f"Roll: {roll:.1f}°"))
        self.root.after(0, lambda: self.lbl_aws.config(text=f"AWS: {aws:.1f} kn"))
        self.root.after(0, lambda: self.lbl_rudder.config(text=f"Rudder: {rudder:.1f}°"))
        self.root.after(0, lambda: self.lat_var.set(f"{lat:.4f}"))
        self.root.after(0, lambda: self.lon_var.set(f"{lon:.4f}"))

    def _append_text(self, widget, text):
        widget.insert(tk.END, text + "\n")
        lines = int(widget.index('end-1c').split('.')[0])
        if lines > 50:
            widget.delete('1.0', tk.END) 
        widget.see(tk.END)

    def log_nmea_out(self, msg):
        self.root.after(0, lambda: self._append_text(self.txt_out, msg))

    def log_nmea_in(self, msg):
        self.root.after(0, lambda: self._append_text(self.txt_in, msg))

    def log_msg(self, prefix, msg):
        formatted = f"[{prefix}] {msg}"
        self.log_nmea_in(formatted)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IMOCA 60 Sailboat Simulator")
    parser.add_argument("--config", type=str, default="sim_state.json", help="Path to the state/config JSON file")
    args = parser.parse_args()

    root = tk.Tk()
    app = SimGUI(root, config_file=args.config)
    root.mainloop()