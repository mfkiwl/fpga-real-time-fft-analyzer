import sys
import numpy as np
import time
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from scipy.signal import butter, cheby1, cheby2, ellip, bessel
from PyQt5 import QtCore, QtNetwork, QtWidgets
import scipy.signal as signal
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import base64
from io import BytesIO


# ===================== USER CONFIG =====================
DEFAULT_COMM_MODE = "UART"
DEFAULT_LOCAL_IP = "0.0.0.0"
DEFAULT_LOCAL_PORT = 6006
DEFAULT_EXPECT_SRC_IP = "169.254.252.255"
DEFAULT_EXPECT_SRC_PORT = 5005
DEFAULT_UART_PORT = "COM5"
DEFAULT_UART_BAUD = 230400


# FPGA Commands
UART_REQUEST_CMD = 0xA5
FPGA_RESET_CMD = 0xFF
ETHERNET_MODE_CMD = 0xEF
UART_MODE_CMD = 0xFE
START_COMMAND = 0x55
FILTER_UPDATE_CMD = 0xF1  # Filter coefficient update command
# Filter type selection commands
FILTER_DEFAULT_CMD = 0x00  # Use default filter
FILTER_CUSTOM_CMD = 0xA1   # Use custom filter
FILTER_NONE_CMD = 0xB1     # No filter

# Frame settings
ETHERNET_PAYLOAD_SIZE = 1025
UART_FRAME_SIZE = 65536
FRAME_SIZE_BYTES = 65536
SAMPLES_PER_FRAME = 16384
FFT_SIZE = 16384
FS_HZ = 1_000_000.0

# Multi-packet configuration
PACKETS_PER_FRAME = 64  # Adjust this to your desired packet count
PACKET_DATA_SIZE = FRAME_SIZE_BYTES // PACKETS_PER_FRAME  # 512 bytes for 4 packets
ETHERNET_PAYLOAD_SIZE = PACKET_DATA_SIZE + 1  # +1 for count byte (513 bytes total)

# Rate limiting
ETHERNET_FPS_LIMIT = 30.0
ETHERNET_DISPLAY_INTERVAL = 1.0 / ETHERNET_FPS_LIMIT

app = Flask(__name__)
app.config['SECRET_KEY'] = 'fft_analyzer_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global configuration
web_config = {
    'comm_mode': DEFAULT_COMM_MODE,
    'local_ip': DEFAULT_LOCAL_IP,
    'local_port': DEFAULT_LOCAL_PORT,
    'uart_port': DEFAULT_UART_PORT,
    'uart_baud': DEFAULT_UART_BAUD,
    'expect_src_ip': DEFAULT_EXPECT_SRC_IP,
    'expect_src_port': DEFAULT_EXPECT_SRC_PORT,
    'plot_types': ['magnitude'],
    'freq_range_start': 0.0,
    'freq_range_end': 1000.0,
    'freq_range_max': 1.0,
    # Filter generator config
    'filter_type': 'lowpass',
    'filter_order': 4,
    'cutoff_freq': 10.0,
    'cutoff_freq2': 20.0,  # For bandpass/bandstop
    'sample_rate': 100.0,
    # Filter selection config
    'active_filter': 'default'  # default, custom, none
}

# Statistics tracking
receiver_state = {
    'is_active': False,
    'frames_received': 0,
    'frames_displayed': 0,
    'frames_dropped': 0,
    'incoming_fps': 0.0,
    'display_fps': 0.0,
    'last_display_time': 0.0,
    'fps_counters': {'incoming': 0, 'display': 0, 'time': time.time()},
    'last_reset_time': 0.0
}

# Global Qt objects
qt_app = None
receiver_controller = None


def int8_to_byte(val: int) -> int:
    """Convert signed int8 (-128…127) to the raw byte 0…255."""
    val = int(val) & 0xFF          # keeps two’s-complement pattern
    return val


# Filter design functions
def design_iir_filter(
        filter_type: str,
        order: int,
        cutoff: float,
        cutoff2: float | None = None,
        fs: float = 100.0,
        *,                       # force keyword-only for the extras
        kind: str = "butter",    # butter | cheby1 | cheby2 | ellip | bessel
        ripple: float = 1.0,     # dB for Chebyshev I / Elliptic pass-band
        attenuation: float = 40  # dB for Chebyshev II / Elliptic stop-band
):
    """
    Return second-order‐section (SOS) coefficients for several classical
    IIR prototypes.  All coefficients remain **unnormalized** because the
    FPGA side expects raw Q7 values after the common ×64 scaling step.

    Parameters
    ----------
    filter_type   “lowpass”, “highpass”, “bandpass”, or “bandstop”.
    order         Even integer (2, 4, 6, …) — you still quantize two sections.
    cutoff        First cutoff in MHz.
    cutoff2       Second cutoff (only for band-types).  If omitted ⇒ 2×cutoff.
    fs            Sample rate in MHz.
    kind          Topology: butter | cheby1 | cheby2 | ellip | bessel.
    ripple        Max pass-band ripple (dB) for Cheb-I / Elliptic.
    attenuation   Min stop-band atten. (dB) for Cheb-II / Elliptic.
    """
    nyq = fs / 2.0
    if filter_type in ("bandpass", "bandstop") and cutoff2 is None:
        cutoff2 = cutoff * 2

    wn = (cutoff / nyq if filter_type in ("lowpass", "highpass")
          else [cutoff / nyq, cutoff2 / nyq])

    match kind.lower():
        case "butter":
            return butter(order, wn, btype=filter_type, output="sos")
        case "cheby1":
            return cheby1(order, ripple,   wn, btype=filter_type, output="sos")
        case "cheby2":
            return cheby2(order, attenuation, wn, btype=filter_type, output="sos")
        case "ellip":
            return ellip(order, ripple, attenuation, wn,
                         btype=filter_type, output="sos")
        case "bessel":
            # ‘norm=phase’ keeps very linear phase below pass-band edge
            return bessel(order, wn, btype=filter_type,
                          output="sos", norm="phase")
        case _:
            raise ValueError(f"Unsupported kind: {kind}")

def quantize_coefficients(sos):
    """Quantize SOS coefficients to 8-bit signed integers (unnormalized)"""
    quantized_sections = []
    
    for section in sos:
        b0, b1, b2, a0, a1, a2 = section
        
        # Keep coefficients unnormalized - don't divide by a0
        # Use a fixed scaling factor for all coefficients
        scale = 64.0  # This gives some headroom while maximizing precision
        
        b0_q = np.clip(np.round(b0 * scale), -128, 127).astype(np.int8)
        b1_q = np.clip(np.round(b1 * scale), -128, 127).astype(np.int8)
        b2_q = np.clip(np.round(b2 * scale), -128, 127).astype(np.int8)
        a0_q = np.clip(np.round(a0 * scale), -128, 127).astype(np.int8)
        a1_q = np.clip(np.round(a1 * scale), -128, 127).astype(np.int8)
        a2_q = np.clip(np.round(a2 * scale), -128, 127).astype(np.int8)
        
        quantized_sections.append([b0_q, b1_q, b2_q, a0_q, a1_q, a2_q])
    
    return quantized_sections

def print_quantized_coefficients(quantized_coeffs, label="Quantized coefficients"):
    """Print quantized coefficients consistently (exact order sent to FPGA)"""
    print(f"{label}:")
    for i, section in enumerate(quantized_coeffs):
        b0, b1, b2, a0, a1, a2 = section
        print(f"  Section {i}:")
        print(f"    B0={int(b0)}, B1={int(b1)}, B2={int(b2)}")
        print(f"    A0={int(a0)}, A1={int(a1)}, A2={int(a2)}")

def generate_filter_response_plot(sos, fs=100.0):
    """Generate filter frequency response plot and return as base64 image"""
    try:
        # Calculate frequency response
        w, h = signal.sosfreqz(sos, worN=2048, fs=fs)
        
        # Create plot
        plt.figure(figsize=(10, 8))
        
        # Magnitude response
        plt.subplot(2, 1, 1)
        plt.plot(w, 20 * np.log10(np.maximum(np.abs(h), 1e-10)))
        plt.title('Filter Frequency Response (Unnormalized Coefficients)', fontsize=14, fontweight='bold')
        plt.ylabel('Magnitude (dB)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.xlim(0, fs/2)
        
        # Phase response
        plt.subplot(2, 1, 2)
        plt.plot(w, np.angle(h, deg=True))
        plt.xlabel('Frequency (KHz)', fontsize=12)
        plt.ylabel('Phase (degrees)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.xlim(0, fs/2)
        
        plt.tight_layout()
        
        # Convert plot to base64 string
        buffer = BytesIO()
        plt.savefig(buffer, format='png', dpi=150, bbox_inches='tight')
        buffer.seek(0)
        plot_data = buffer.getvalue()
        buffer.close()
        plt.close()
        
        plot_url = base64.b64encode(plot_data).decode()
        return f"data:image/png;base64,{plot_url}"
        
    except Exception as e:
        print(f"Error generating filter plot: {e}")
        return None

def reset_plot_and_data():
    """Reset all plot data and statistics"""
    receiver_state.update({
        'frames_received': 0,
        'frames_displayed': 0,
        'frames_dropped': 0,
        'incoming_fps': 0.0,
        'display_fps': 0.0,
        'last_display_time': 0.0,
        'fps_counters': {'incoming': 0, 'display': 0, 'time': time.time()}
    })
    
    socketio.emit('plot_reset', {
        'message': 'Plot and data reset',
        'timestamp': time.time()
    })
    print("Plot data and statistics reset")

def decode_mag_16iq_le(frame_bytes: bytes) -> np.ndarray:
    """Convert 2048-byte frame to magnitude spectrum"""
    if len(frame_bytes) != FRAME_SIZE_BYTES:
        raise ValueError(f"Invalid frame size: {len(frame_bytes)} (expected {FRAME_SIZE_BYTES})")
    
    arr = np.frombuffer(frame_bytes, dtype=np.uint8)
    re = (arr[0::4].astype(np.uint16) | (arr[1::4].astype(np.uint16) << 8)).astype(np.int16)
    im = (arr[2::4].astype(np.uint16) | (arr[3::4].astype(np.uint16) << 8)).astype(np.int16)
    
    mag = np.sqrt(re.astype(np.float32)**2 + im.astype(np.float32)**2)
    return mag

def decode_iq_components(frame_bytes: bytes):
    """Extract I and Q components"""
    if len(frame_bytes) != FRAME_SIZE_BYTES:
        raise ValueError(f"Invalid frame size: {len(frame_bytes)}")
    
    arr = np.frombuffer(frame_bytes, dtype=np.uint8)
    re = (arr[0::4].astype(np.uint16) | (arr[1::4].astype(np.uint16) << 8)).astype(np.int16)
    im = (arr[2::4].astype(np.uint16) | (arr[3::4].astype(np.uint16) << 8)).astype(np.int16)
    return re.astype(np.float32), im.astype(np.float32)

def update_fps():
    """Update FPS calculations"""
    now = time.time()
    elapsed = now - receiver_state['fps_counters']['time']
    if elapsed >= 1.0:
        receiver_state['incoming_fps'] = receiver_state['fps_counters']['incoming'] / elapsed
        receiver_state['display_fps'] = receiver_state['fps_counters']['display'] / elapsed
        receiver_state['fps_counters'] = {'incoming': 0, 'display': 0, 'time': now}

def should_display_frame(mode):
    """Rate limiting for Ethernet mode"""
    if mode == "UART":
        return True
    
    now = time.time()
    if (now - receiver_state['last_display_time']) >= ETHERNET_DISPLAY_INTERVAL:
        receiver_state['last_display_time'] = now
        return True
    else:
        receiver_state['frames_dropped'] += 1
        return False

def get_frequency_range_data(magnitude_data):
    """Fixed frequency range data with correct axis calculation"""
    # Correct frequency axis calculation
    freq_axis = np.arange(len(magnitude_data), dtype=np.float32) * (FS_HZ / FFT_SIZE) / 1e3  # MHz
    
    # Apply frequency range filter based on percentage
    start_idx = int(web_config['freq_range_start'] * len(magnitude_data) / 1000.0)
    end_idx = int(web_config['freq_range_end'] * len(magnitude_data) / 1000.0)
    start_idx = max(0, min(start_idx, len(magnitude_data) - 1))
    end_idx = max(start_idx + 1, min(end_idx, len(magnitude_data)))
    
    return freq_axis[start_idx:end_idx], magnitude_data[start_idx:end_idx]


class MultiPacketAssembler:
    """Assembles complete FFT frames from multiple UDP packets using count byte"""
    def __init__(self, packet_count, packet_data_size, evict_ms=3000):
        self.packet_count = packet_count        # Total packets per frame
        self.packet_data_size = packet_data_size # Data bytes per packet (excluding count byte)
        self.evict_ms = evict_ms
        self.packets = [None] * packet_count    # Storage for received packets
        self.timestamps = [0] * packet_count    # Timestamps for eviction
        self.frame_id = 0                       # Optional: track frame sequence

    def add(self, payload: bytes, now_ms: int):
        """Add a packet to the assembler. Returns complete frame when all packets received."""
        if len(payload) != self.packet_data_size + 1:  # +1 for count byte
            return None
            
        count_byte = payload[0]
        if count_byte >= self.packet_count:
            return None  # Invalid packet index
            
        data = payload[1:]  # Extract data portion
        self.packets[count_byte] = data
        self.timestamps[count_byte] = now_ms

        # Check if all packets for current frame are received
        if all(p is not None for p in self.packets):
            frame = b''.join(self.packets)
            print(f"[DEBUG] Assembled Ethernet frame of {len(frame)} bytes")
            # Reset for next frame
            self.packets = [None] * self.packet_count
            self.timestamps = [0] * self.packet_count
            self.frame_id += 1
            return frame

        # Evict old/stale packets
        for i in range(self.packet_count):
            if self.packets[i] is not None and (now_ms - self.timestamps[i]) > self.evict_ms:
                self.packets[i] = None
                self.timestamps[i] = 0
                
        return None

    def get_completion_status(self):
        """Return how many packets received for current frame"""
        received = sum(1 for p in self.packets if p is not None)
        return received, self.packet_count


class UdpReceiver(QtCore.QObject):
    def __init__(self, ip: str, port: int):
        super().__init__()
        self.sock = QtNetwork.QUdpSocket(self)
        ok = self.sock.bind(QtNetwork.QHostAddress(ip), port)
        if not ok:
            raise Exception(f"Failed to bind UDP socket to {ip}:{port}")
        
        self.sock.readyRead.connect(self.on_data_ready)
        # Use MultiPacketAssembler instead of HalfAssembler
        self.assembler = MultiPacketAssembler(
            packet_count=PACKETS_PER_FRAME,
            packet_data_size=PACKET_DATA_SIZE,
            evict_ms=4000
        )
        self.active = True
        print(f"UDP listening on {ip}:{port} (expecting {PACKETS_PER_FRAME} packets per frame)")

    def on_data_ready(self):
        while self.sock.hasPendingDatagrams() and self.active:
            data, sender_addr, sender_port = self.sock.readDatagram(self.sock.pendingDatagramSize())
            
            sender_ip = sender_addr.toString()
            if web_config.get('expect_src_ip') and sender_ip != web_config['expect_src_ip']:
                continue
            if web_config.get('expect_src_port') and sender_port != web_config['expect_src_port']:
                continue
                
            self.process_payload(data)

    def process_payload(self, data):
        """Process incoming UDP payload with count-based assembly"""
        if len(data) != ETHERNET_PAYLOAD_SIZE:
            print(f"Invalid payload size: {len(data)} (expected {ETHERNET_PAYLOAD_SIZE})")
            return
        
        now_ms = QtCore.QTime.currentTime().msecsSinceStartOfDay()
        assembled_frame = self.assembler.add(data, now_ms)
        
        if assembled_frame is not None and len(assembled_frame) == FRAME_SIZE_BYTES:
            print(f"[DEBUG] Assembled frame received for plotting, size={len(assembled_frame)}")
            receiver_state['frames_received'] += 1
            receiver_state['fps_counters']['incoming'] += 1
            update_fps()
            
            if should_display_frame("ETHERNET"):
                receiver_state['frames_displayed'] += 1
                receiver_state['fps_counters']['display'] += 1
                
                try:
                    magnitude_data = decode_mag_16iq_le(assembled_frame)
                    self.emit_plot_data(magnitude_data, assembled_frame)
                except Exception as e:
                    print(f"FFT decode error: {e}")
        elif assembled_frame is None:
            # Log packet assembly status
            received, total = self.assembler.get_completion_status()
            if received > 0:  # Only log if we have partial assembly
                print(f"Frame assembly: {received}/{total} packets received")

    def emit_plot_data(self, magnitude_data, frame):
        """Emit real FFT data to web interface"""
        freq_axis, filtered_mag = get_frequency_range_data(magnitude_data)
        
        plot_data = {}
        if 'magnitude' in web_config['plot_types']:
            plot_data['magnitude'] = filtered_mag.tolist()
        
        if 'real' in web_config['plot_types'] or 'imaginary' in web_config['plot_types']:
            real_data, imag_data = decode_iq_components(frame)
            start_idx = int(web_config['freq_range_start'] * SAMPLES_PER_FRAME / 1000)
            end_idx = int(web_config['freq_range_end'] * SAMPLES_PER_FRAME / 1000)
            start_idx = max(0, min(start_idx, SAMPLES_PER_FRAME - 1))
            end_idx = max(start_idx + 1, min(end_idx, SAMPLES_PER_FRAME))
            
            if 'real' in web_config['plot_types']:
                plot_data['real'] = real_data[start_idx:end_idx].tolist()
            if 'imaginary' in web_config['plot_types']:
                plot_data['imaginary'] = imag_data[start_idx:end_idx].tolist()

        max_val = np.max(filtered_mag) if len(filtered_mag) > 0 else 0
        max_idx = np.argmax(filtered_mag) if len(filtered_mag) > 0 else 0
        peak_freq = freq_axis[max_idx] if len(freq_axis) > max_idx else 0
        
        socketio.emit('frame_data', {
            'frequency': freq_axis.tolist(),
            'data': plot_data,
            'incoming_fps': round(receiver_state['incoming_fps'], 1),
            'display_fps': round(receiver_state['display_fps'], 1),
            'frames_received': receiver_state['frames_received'],
            'frames_displayed': receiver_state['frames_displayed'],
            'frames_dropped': receiver_state['frames_dropped'],
            'packet_count': receiver_state['frames_received'],
            'peak_magnitude': float(max_val),
            'peak_frequency': float(peak_freq),
            'peak_bin': int(max_idx),
            'timestamp': time.time(),
            'receiver_active': receiver_state['is_active'],
            'freq_range_start': web_config['freq_range_start'],
            'freq_range_end': web_config['freq_range_end']
        })

    def stop(self):
        self.active = False
        if self.sock:
            self.sock.close()



class UartReceiver(QtCore.QObject):
    def __init__(self, port: str, baud: int):
        super().__init__()
        try:
            import serial
            self.ser = serial.Serial(
                port, 
                baud, 
                timeout=0.001,
                writeTimeout=0.5,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
                exclusive=True
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"UART opened: {port} @ {baud} baud (enhanced)")
        except Exception as e:
            print(f"UART open failed: {e}")
            raise

        self.active = True
        self.frame_buffer = bytearray()
        self.last_emit_time = 0
        self.frame_drop_count = 0
        self.read_buffer = bytearray()
        self.last_read_time = time.time()
        self.read_timeouts = 0
        self.consecutive_empty_reads = 0
        
        # QTimer now properly in main thread
        self.read_timer = QtCore.QTimer(self)
        self.read_timer.timeout.connect(self.read_data)
        self.read_timer.start(1)

    def force_mode_reset(self):
        """Force complete FPGA state reset during mode switching"""
        try:
            for _ in range(3):
                reset_cmd = bytes([FPGA_RESET_CMD])
                if hasattr(self, 'ser') and self.ser.is_open:
                    self.ser.write(reset_cmd)
                    self.ser.flush()
                    time.sleep(0.1)
            
            time.sleep(1.0)
            
            receiver_state.update({
                'frames_received': 0,
                'frames_displayed': 0, 
                'frames_dropped': 0,
                'incoming_fps': 0.0,
                'display_fps': 0.0,
                'last_display_time': 0.0,
                'fps_counters': {'incoming': 0, 'display': 0, 'time': time.time()}
            })
            
            self.frame_buffer.clear()
            print("Complete FPGA mode reset performed")
            return True
        except Exception as e:
            print(f"Mode reset error: {e}")
            return False

    def send_start_sequence(self):
        """Send 0x55 followed by 0xA5 after 100ms for UART mode"""
        try:
            cmd = bytes([START_COMMAND])
            self.ser.write(cmd)
            self.ser.flush()
            print(f"UART start command sent: 0x{START_COMMAND:02X}")
            
            QtCore.QTimer.singleShot(100, self.send_data_request)
            return True
        except Exception as e:
            print(f"UART start sequence error: {e}")
            return False

    def send_data_request(self):
        """Send 0xA5 command (called after 100ms delay)"""
        try:
            cmd = bytes([UART_REQUEST_CMD])
            self.ser.write(cmd)
            self.ser.flush()
            print(f"UART data request sent: 0x{UART_REQUEST_CMD:02X} (100ms after start)")
            return True
        except Exception as e:
            print(f"UART data request error: {e}")
            return False

    def send_ethernet_start(self):
        """Send only 0x55 for Ethernet mode"""
        try:
            cmd = bytes([START_COMMAND])
            self.ser.write(cmd)
            self.ser.flush()
            print(f"Ethernet start command sent: 0x{START_COMMAND:02X}")
            return True
        except Exception as e:
            print(f"Ethernet start command error: {e}")
            return False

    def send_command(self, command):
        """Send FPGA command over UART"""
        try:
            if self.ser and self.ser.is_open:
                if command == FPGA_RESET_CMD:
                    now = time.time()
                    if (now - receiver_state['last_reset_time']) < 2.0:
                        print("FPGA reset ignored - cooldown active")
                        return False
                    receiver_state['last_reset_time'] = now
                
                cmd_bytes = bytes([command])
                self.ser.write(cmd_bytes)
                self.ser.flush()
                print(f"UART command sent: 0x{command:02X}")
                return True
        except Exception as e:
            print(f"UART command error: {e}")
            return False

    @staticmethod
    def _byte(val):                    # local shorthand
        return int(val) & 0xFF

    def send_filter_coefficients(self, coefficients):
        """Upload *exactly* the same int8 list that the preview shows."""
        try:
            if not (self.ser and self.ser.is_open):
                return False

            # 1. command header
            self.ser.write(bytes([FILTER_UPDATE_CMD]))
            self.ser.flush()
            print(f"Filter update command sent: 0x{FILTER_UPDATE_CMD:02X}")

            # 2. coefficient payload (two sections × 6 = 12 bytes expected)
            payload = bytearray(self._byte(c) for sec in coefficients for c in sec)
            self.ser.write(payload)
            self.ser.flush()

            print(f"Sent {len(payload)} coefficient bytes")
            print_quantized_coefficients(coefficients, "Coefficients sent to FPGA")
            return True

        except Exception as e:
            print(f"Filter coefficient send error: {e}")
            return False


    def read_data(self):
        """Enhanced read with better buffer management"""
        if not self.active:
            return
        
        try:
            now = time.time()
            bytes_available = self.ser.in_waiting
            
            if bytes_available > 0:
                chunk_size = min(bytes_available, 4096)
                new_data = self.ser.read(chunk_size)
                
                if new_data:
                    self.frame_buffer.extend(new_data)
                    self.consecutive_empty_reads = 0
                    self.last_read_time = now
                else:
                    self.consecutive_empty_reads += 1
            else:
                self.consecutive_empty_reads += 1
                
            # Check for stalled communication
            if (now - self.last_read_time) > 1.0 and self.consecutive_empty_reads > 1000:
                print("UART: No data for 1 second, resetting buffers")
                self.ser.reset_input_buffer()
                self.frame_buffer.clear()
                self.consecutive_empty_reads = 0
                self.last_read_time = now
                
            self.process_buffer()
            
        except Exception as e:
            print(f"UART read error: {e}")
            self.ser.reset_input_buffer()
            self.consecutive_empty_reads = 0

    def process_buffer(self):
        """Enhanced buffer processing with better frame timing"""
        frames_processed = 0
        
        while len(self.frame_buffer) >= UART_FRAME_SIZE and self.active:
            frame = bytes(self.frame_buffer[:UART_FRAME_SIZE])
            self.frame_buffer = self.frame_buffer[UART_FRAME_SIZE:]
            
            receiver_state['frames_received'] += 1
            receiver_state['fps_counters']['incoming'] += 1
            update_fps()
            
            try:
                magnitude_data = decode_mag_16iq_le(frame)
                
                now = time.time()
                time_since_last = now - self.last_emit_time
                
                min_spacing = 0.008 if frames_processed == 0 else 0.012
                
                if time_since_last >= min_spacing:
                    receiver_state['frames_displayed'] += 1
                    receiver_state['fps_counters']['display'] += 1
                    self.last_emit_time = now
                    
                    self.emit_frame_data(magnitude_data, frame, time_since_last)
                    frames_processed += 1
                else:
                    self.frame_drop_count += 1
                    receiver_state['frames_dropped'] += 1
                    
            except Exception as e:
                print(f"Frame decode error: {e}")
                
        if len(self.frame_buffer) > 10 * UART_FRAME_SIZE:
            print("UART: Buffer overflow, clearing oldest data")
            self.frame_buffer = self.frame_buffer[-2 * UART_FRAME_SIZE:]

    def emit_frame_data(self, magnitude_data, frame, timing_info):
        """Enhanced frame emission with priority handling"""
        freq_axis, filtered_mag = get_frequency_range_data(magnitude_data)
        
        plot_data = {}
        if 'magnitude' in web_config['plot_types']:
            plot_data['magnitude'] = filtered_mag.tolist()
        
        if 'real' in web_config['plot_types'] or 'imaginary' in web_config['plot_types']:
            real_data, imag_data = decode_iq_components(frame)
            start_idx = int(web_config['freq_range_start'] * SAMPLES_PER_FRAME / 1000)
            end_idx = int(web_config['freq_range_end'] * SAMPLES_PER_FRAME / 1000)
            start_idx = max(0, min(start_idx, SAMPLES_PER_FRAME - 1))
            end_idx = max(start_idx + 1, min(end_idx, SAMPLES_PER_FRAME))
            
            if 'real' in web_config['plot_types']:
                plot_data['real'] = real_data[start_idx:end_idx].tolist()
            if 'imaginary' in web_config['plot_types']:
                plot_data['imaginary'] = imag_data[start_idx:end_idx].tolist()

        max_val = np.max(filtered_mag) if len(filtered_mag) > 0 else 0
        max_idx = np.argmax(filtered_mag) if len(filtered_mag) > 0 else 0
        peak_freq = freq_axis[max_idx] if len(freq_axis) > max_idx else 0
        
        frame_payload = {
            'frequency': freq_axis.tolist(),
            'data': plot_data,
            'incoming_fps': round(receiver_state['incoming_fps'], 1),
            'display_fps': round(receiver_state['display_fps'], 1),
            'frames_received': receiver_state['frames_received'],
            'frames_displayed': receiver_state['frames_displayed'],
            'frames_dropped': receiver_state['frames_dropped'],
            'packet_count': receiver_state['frames_received'],
            'peak_magnitude': float(max_val),
            'peak_frequency': float(peak_freq),
            'peak_bin': int(max_idx),
            'timestamp': time.time(),
            'receiver_active': receiver_state['is_active'],
            'freq_range_start': web_config['freq_range_start'],
            'freq_range_end': web_config['freq_range_end'],
            'frame_timing': timing_info,
            'buffer_health': len(self.frame_buffer),
            'read_performance': self.consecutive_empty_reads
        }
        
        socketio.emit('frame_data', frame_payload, callback=self.confirm_delivery)

    def confirm_delivery(self, response=None):
        """Callback to confirm frame delivery"""
        pass

    def stop(self):
        self.active = False
        if hasattr(self, 'read_timer'):
            self.read_timer.stop()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

class ReceiverController(QtCore.QObject):
    # Qt signal for filter coefficients
    send_filter_coeff_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.current_receiver = None
        # Connect signal to slot
        self.send_filter_coeff_signal.connect(self._send_filter_coefficients)

    # Private slot method to handle coefficient sending
    @QtCore.pyqtSlot(list)
    def _send_filter_coefficients(self, coefficients):
        """Internal method to send filter coefficients"""
        if isinstance(self.current_receiver, UartReceiver):
            success = self.current_receiver.send_filter_coefficients(coefficients)
            if success:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'Filter coefficients uploaded successfully ({len(coefficients)} sections, unnormalized)'
                })
            else:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': 'Filter coefficient upload failed - check UART connection'
                })
        else:
            try:
                import serial
                temp_uart = serial.Serial(web_config['uart_port'], web_config['uart_baud'], timeout=1.0)
                
                # Send filter update command
                cmd_bytes = bytes([FILTER_UPDATE_CMD])
                temp_uart.write(cmd_bytes)
                temp_uart.flush()
                
                # Send coefficient data
                coeff_data = bytearray()
                for section in coefficients:
                    for coeff in section:
                        if hasattr(coeff, 'tobytes'):
                            coeff_byte = coeff.tobytes()[0]
                        else:
                            coeff_byte = (int(coeff) & 0xFF)
                        coeff_data.append(coeff_byte)
                
                temp_uart.write(coeff_data)
                temp_uart.flush()
                temp_uart.close()
                
                # Print what was sent - FIXED
                print_quantized_coefficients(coefficients, "Coefficients sent via temporary UART")
                
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'Filter coefficients uploaded via temporary UART ({len(coefficients)} sections, unnormalized)'
                })
                
            except Exception as e:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'Filter coefficient upload failed: {e}'
                })

    @QtCore.pyqtSlot(str)
    def start_receiver(self, mode):
        self.stop_receiver()
        
        reset_plot_and_data()
        receiver_state['is_active'] = True
        
        try:
            if mode == "ETHERNET":
                self.current_receiver = UdpReceiver(web_config['local_ip'], web_config['local_port'])
                print(f"Receiver started in {mode} mode")
                socketio.emit('receiver_status', {
                    'active': True,
                    'mode': mode,
                    'message': f'{mode} receiver started successfully'
                })
            elif mode == "UART":
                self.current_receiver = UartReceiver(web_config['uart_port'], web_config['uart_baud'])
                print(f"Receiver started in {mode} mode")
                socketio.emit('receiver_status', {
                    'active': True,
                    'mode': mode,
                    'message': f'{mode} receiver started successfully'
                })
            
        except Exception as e:
            print(f"Failed to start receiver: {e}")
            receiver_state['is_active'] = False
            socketio.emit('receiver_status', {
                'active': False,
                'mode': mode,
                'message': f'Failed to start receiver: {e}'
            })

    @QtCore.pyqtSlot(str)
    def send_start_commands(self, mode):
        """Send appropriate start commands based on mode"""
        if isinstance(self.current_receiver, UartReceiver):
            if mode == "ETHERNET":
                success = self.current_receiver.send_ethernet_start()
                if success:
                    socketio.emit('receiver_status', {
                        'active': receiver_state['is_active'],
                        'mode': mode,
                        'message': f'Ethernet start command sent (0x{START_COMMAND:02X})'
                    })
                else:
                    socketio.emit('receiver_status', {
                        'active': receiver_state['is_active'],
                        'mode': mode,
                        'message': 'Ethernet start command failed'
                    })
            elif mode == "UART":
                success = self.current_receiver.send_start_sequence()
                if success:
                    socketio.emit('receiver_status', {
                        'active': receiver_state['is_active'],
                        'mode': mode,
                        'message': f'UART start sequence initiated (0x{START_COMMAND:02X} → 0x{UART_REQUEST_CMD:02X})'
                    })
                else:
                    socketio.emit('receiver_status', {
                        'active': receiver_state['is_active'],
                        'mode': mode,
                        'message': 'UART start sequence failed'
                    })
        else:
            try:
                import serial
                temp_uart = serial.Serial(web_config['uart_port'], web_config['uart_baud'], timeout=1.0)
                cmd_bytes = bytes([START_COMMAND])
                temp_uart.write(cmd_bytes)
                temp_uart.flush()
                temp_uart.close()
                
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': mode,
                    'message': f'Start command sent via temporary UART (0x{START_COMMAND:02X})'
                })
                
            except Exception as e:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': mode,
                    'message': f'Start command failed: {e}'
                })

    @QtCore.pyqtSlot(int)
    def send_fpga_command(self, command):
        """Send FPGA command - works regardless of communication mode"""
        if isinstance(self.current_receiver, UartReceiver):
            success = self.current_receiver.send_command(command)
            cmd_names = {
                0xFF: "FPGA Reset", 
                0xEF: "Ethernet Mode", 
                0xFE: "UART Mode", 
                0xF1: "Filter Update",
                0x00: "Default Filter",
                0xA1: "Custom Filter", 
                0xB1: "No Filter"
            }
            cmd_name = cmd_names.get(command, f"Command 0x{command:02X}")
            
            if success:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'{cmd_name} command sent successfully (0x{command:02X})'
                })
                
                if command == FPGA_RESET_CMD:
                    reset_plot_and_data()
            else:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'{cmd_name} command failed - check UART connection'
                })
        else:
            try:
                import serial
                temp_uart = serial.Serial(web_config['uart_port'], web_config['uart_baud'], timeout=1.0)
                cmd_bytes = bytes([command])
                temp_uart.write(cmd_bytes)
                temp_uart.flush()
                temp_uart.close()
                
                cmd_names = {
                    0xFF: "FPGA Reset", 
                    0xEF: "Ethernet Mode", 
                    0xFE: "UART Mode", 
                    0xF1: "Filter Update",
                    0x00: "Default Filter",
                    0xA1: "Custom Filter", 
                    0xB1: "No Filter"
                }
                cmd_name = cmd_names.get(command, f"Command 0x{command:02X}")
                
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],  
                    'message': f'{cmd_name} command sent via temporary UART (0x{command:02X})'
                })
                
                if command == FPGA_RESET_CMD:
                    reset_plot_and_data()
                    
            except Exception as e:
                socketio.emit('receiver_status', {
                    'active': receiver_state['is_active'],
                    'mode': web_config['comm_mode'],
                    'message': f'FPGA command failed: {e}'
                })

    @QtCore.pyqtSlot()
    def stop_receiver(self):
        if self.current_receiver:
            self.current_receiver.stop()
            self.current_receiver.deleteLater()
            self.current_receiver = None
            receiver_state['is_active'] = False

# Flask thread function
def flask_thread():
    """Flask thread - now runs in secondary thread"""
    print("Starting Flask server in secondary thread...")
    try:
        socketio.run(app, debug=False, host='0.0.0.0', port=5000, use_reloader=False)
    except Exception as e:
        print(f"Flask server error: {e}")

# Flask routes and WebSocket handlers
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    emit('config_update', web_config)
    emit('receiver_status', {
        'active': receiver_state['is_active'],
        'mode': web_config['comm_mode'],
        'message': 'Connected to FFT Analyzer'
    })

@socketio.on('set_mode')
def handle_set_mode():
    """Enhanced mode switching with proper reset"""
    global web_config, receiver_controller
    
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "stop_receiver",
            QtCore.Qt.QueuedConnection
        )
        time.sleep(0.2)
    
    reset_plot_and_data()
    
    if receiver_controller and isinstance(receiver_controller.current_receiver, UartReceiver):
        receiver_controller.current_receiver.force_mode_reset()
    elif receiver_controller:
        try:
            import serial
            temp_uart = serial.Serial(web_config['uart_port'], web_config['uart_baud'], timeout=1.0)
            for _ in range(3):
                reset_cmd = bytes([FPGA_RESET_CMD])
                temp_uart.write(reset_cmd)
                temp_uart.flush()
                time.sleep(0.1)
            temp_uart.close()
            time.sleep(1.0)
            print("Temporary UART reset performed")
        except Exception as e:
            print(f"Temporary UART reset failed: {e}")
    
    mode = web_config['comm_mode']
    command = ETHERNET_MODE_CMD if mode == "ETHERNET" else UART_MODE_CMD
    
    print(f"Setting FPGA to {mode} mode with complete reset")
    
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "send_fpga_command",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(int, command)
        )
    
    time.sleep(0.5)
    
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "start_receiver",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, mode)
        )

@socketio.on('update_config')
def handle_update_config(data):
    """Handle basic configuration updates (no restart)"""
    global web_config
    web_config.update(data)
    emit('config_update', web_config)

@socketio.on('fpga_reset')
def handle_fpga_reset():
    """Handle FPGA reset command"""
    global receiver_controller
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "send_fpga_command",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(int, FPGA_RESET_CMD)
        )

@socketio.on('reset_plot')
def handle_reset_plot():
    """Handle manual plot reset"""
    reset_plot_and_data()
    emit('receiver_status', {
        'active': receiver_state['is_active'],
        'mode': web_config['comm_mode'],
        'message': 'Plot and data manually reset'
    })

@socketio.on('apply_frequency_range')
def handle_apply_frequency_range(data):
    """Handle frequency range updates"""
    web_config['freq_range_start'] = float(data.get('freq_start', 0))
    web_config['freq_range_end'] = float(data.get('freq_end', 1000))
    
    if 'plot_types' in data:
        web_config['plot_types'] = data.get('plot_types', ['magnitude'])
    
    emit('config_update', web_config)
    
    freq_msg = f"Frequency range: {web_config['freq_range_start']}-{web_config['freq_range_end']} KHz"
    plot_msg = f"Plot types: {', '.join(web_config['plot_types'])}"
    
    emit('receiver_status', {
        'active': receiver_state['is_active'],
        'mode': web_config['comm_mode'],
        'message': f"{freq_msg}, {plot_msg}"
    })

@socketio.on('start_receiver')
def handle_start_receiver():
    """Handle start receiver with appropriate commands based on mode"""
    global receiver_controller, web_config
    
    mode = web_config['comm_mode']
    
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "send_start_commands",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, mode)
        )

@socketio.on('stop_receiver')
def handle_stop_receiver():
    QtCore.QMetaObject.invokeMethod(
        receiver_controller, "stop_receiver",
        QtCore.Qt.QueuedConnection
    )

# Filter generator WebSocket handlers
@socketio.on("update_filter_config")
def handle_update_filter_config(data):
    web_config.update({
        "filter_type":   data.get("filter_type",  "lowpass"),
        "filter_order":  int(data.get("filter_order", 4)),
        "cutoff_freq":   float(data.get("cutoff_freq", 10.0)),
        "cutoff_freq2":  float(data.get("cutoff_freq2", 20.0)),
        "sample_rate":   float(data.get("sample_rate", 100.0)),
        "filter_kind":   data.get("filter_kind",  "butter"),
        "ripple":        float(data.get("ripple", 1.0)),
        "attenuation":   float(data.get("attenuation", 40.0)),
    })
    emit("config_update", web_config)


@socketio.on("generate_filter_preview")
def handle_generate_filter_preview():
    try:
        sos = design_iir_filter(
            web_config["filter_type"],
            web_config["filter_order"],
            web_config["cutoff_freq"],
            web_config["cutoff_freq2"],
            web_config["sample_rate"],
            kind=web_config["filter_kind"],
            ripple=web_config["ripple"],
            attenuation=web_config["attenuation"]
        )
        plot_url         = generate_filter_response_plot(sos, web_config["sample_rate"])
        quantized_coeffs = quantize_coefficients(sos)

        emit("filter_preview", {
            "success":      True,
            "plot_url":     plot_url,
            "coefficients": [[int(c) for c in sec] for sec in quantized_coeffs],
            "num_sections": len(quantized_coeffs),
            "note":         "Coefficients are unnormalized and scaled by 64"
        })
    except Exception as e:
        emit("filter_preview", {"success": False, "error": str(e)})


@socketio.on('apply_filter_to_fpga')
def handle_apply_filter_to_fpga():
    """Apply designed filter to FPGA - unnormalized coefficients - FIXED VERSION"""
    global receiver_controller
    
    try:
        # Design the filter
        sos = design_iir_filter(
            web_config['filter_type'],
            web_config['filter_order'],
            web_config['cutoff_freq'],
            web_config['cutoff_freq2'],
            web_config['sample_rate']
        )
        
        # Quantize coefficients (unnormalized) - FIXED
        quantized_coeffs = quantize_coefficients(sos)
        
        # Ensure we have exactly 2 sections (pad or truncate as needed)
        if len(quantized_coeffs) > 2:
            quantized_coeffs = quantized_coeffs[:2]
        elif len(quantized_coeffs) < 2:
            # Pad with identity sections
            identity_section = [64, 0, 0, 64, 0, 0]  # Pass-through section
            while len(quantized_coeffs) < 2:
                quantized_coeffs.append(identity_section)
        
        # Print what will be sent for verification - FIXED
        print_quantized_coefficients(quantized_coeffs, "Coefficients to be sent to FPGA")
        
        # Use signal emission to send coefficients
        if receiver_controller:
            receiver_controller.send_filter_coeff_signal.emit(quantized_coeffs)
        
    except Exception as e:
        print(f"Filter apply error: {e}")
        emit('receiver_status', {
            'active': receiver_state['is_active'],
            'mode': web_config['comm_mode'],
            'message': f'Filter apply failed: {e}'
        })

# Filter type selection handler
@socketio.on('set_filter_type')
def handle_set_filter_type(data):
    """Handle filter type selection command"""
    global receiver_controller, web_config
    
    filter_option = data.get('filter_option', 'default')
    web_config['active_filter'] = filter_option
    
    # Map filter options to UART commands
    command_map = {
        'default': FILTER_DEFAULT_CMD,
        'custom': FILTER_CUSTOM_CMD,
        'none': FILTER_NONE_CMD
    }
    
    command = command_map.get(filter_option, FILTER_DEFAULT_CMD)
    
    if receiver_controller:
        QtCore.QMetaObject.invokeMethod(
            receiver_controller, "send_fpga_command",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(int, command)
        )
    
    emit('config_update', web_config)

# Main function with corrected threading architecture
if __name__ == '__main__':
    print("Starting FFT Analyzer with Filter Generator and Filter Selection - FIXED COEFFICIENT DISPLAY/SEND CONSISTENCY")
    print("PyQt runs in MAIN thread, Flask runs in SECONDARY thread")
    
    # Create QApplication in main thread
    qt_app = QtWidgets.QApplication(sys.argv)
    receiver_controller = ReceiverController()
    
    # Send initial mode command
    mode = web_config['comm_mode']
    command = ETHERNET_MODE_CMD if mode == "ETHERNET" else UART_MODE_CMD
    print(f"Sending initial {mode} mode command: 0x{command:02X}")
    
    try:
        import serial
        temp_uart = serial.Serial(web_config['uart_port'], web_config['uart_baud'], timeout=1.0)
        cmd_bytes = bytes([command])
        temp_uart.write(cmd_bytes)
        temp_uart.flush()
        temp_uart.close()
        print(f"Initial {mode} mode command sent successfully")
    except Exception as e:
        print(f"Failed to send initial mode command: {e}")
    
    # Start receiver
    receiver_controller.start_receiver(web_config['comm_mode'])
    
    # Start Flask in secondary thread
    flask_thread_obj = threading.Thread(target=flask_thread, daemon=True)
    flask_thread_obj.start()
    
    # Brief delay to let Flask start
    time.sleep(1.0)
    
    print("System startup complete:")
    print(f"Frequency Resolution: {FS_HZ/FFT_SIZE/1000:.1f} kHz per bin")
    print(f"Total Spectrum: 0-{FS_HZ/1e3:.0f} KHz across {FFT_SIZE} bins")
    print("Web interface: http://localhost:5000")
    print("Filter Generator: UART command 0xF1 for coefficient updates - DISPLAY/SEND CONSISTENCY FIXED")
    print("Filter Selection: Commands 0x00 (Default), 0xA1 (Custom), 0xB1 (None)")
    print("Coefficients scaled by 64 and truncated to 8-bit signed range")
    print("What you see displayed = What gets sent to FPGA")
    print("PyQt event loop starting in main thread...")
    
    # Run Qt event loop in main thread
    try:
        qt_app.exec_()
    except KeyboardInterrupt:
        print("\nShutting down...")
        if receiver_controller:
            receiver_controller.stop_receiver()
        qt_app.quit()
