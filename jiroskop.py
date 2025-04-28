# -- coding: utf-8 --

import RPi.GPIO as GPIO
import time
import rtlsdr
import numpy as np
from smbus import SMBus
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from threading import Thread

# Mesajlarý terminalde ve spektrum analizörde gösterebilen güncellenen log fonksiyonu
def log_message(message, analyzer=None):
    print(message)
    if analyzer is not None:
        analyzer.add_log_message(message)

# ==============================
# MPU6050 CLASS
# ==============================
class MPU6050:
    def __init__(self, address=0x68, bus_num=1):
        self.address = address
        self.gyro_offset = 0
        self.bus = None

        print(f"Initializing MPU6050 on address {hex(address)}, bus {bus_num}")
        try:
            self.bus = SMBus(bus_num)
            print("SMBus initialized successfully")
        except Exception as e:
            print(f"Error initializing SMBus: {e}")
            try:
                import smbus2
                self.bus = smbus2.SMBus(bus_num)
                print("Using smbus2 as fallback")
            except Exception as e2:
                print(f"Failed to initialize fallback SMBus: {e2}")
                self.bus = DummySMBus()

        self.initialize_sensor()
        self.calibrate_gyro()

    def initialize_sensor(self):
        try:
            self.bus.write_byte_data(self.address, 0x6B, 0)
            time.sleep(0.1)
            print("Sensor initialized")
        except Exception as e:
            print(f"Error initializing sensor: {e}")

    def read_word(self, reg):
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            return val
        except Exception as e:
            print(f"Read error at reg {hex(reg)}: {e}")
            return 0

    def calibrate_gyro(self):
        print("Calibrating gyro, please keep the vehicle stationary...")
        offset_sum = 0
        samples = 100
        for i in range(samples):
            val = self.read_word(0x47)
            offset_sum += val
            time.sleep(0.01)
        self.gyro_offset = offset_sum / samples / 131.0
        print(f"Gyro offset: {self.gyro_offset:.2f} deg/s")

    def get_gyro_z(self):
        raw = self.read_word(0x47)
        return (raw / 131.0) - self.gyro_offset

class DummySMBus:
    def read_byte_data(self, *args):
        return 0
    def write_byte_data(self, *args):
        return

# ==============================
# MOTOR CONTROL PINS
# ==============================
IN1, IN2, IN3, IN4 = 17, 18, 22, 23
PWM1, PWM2 = 24, 25

def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([IN1, IN2, IN3, IN4, PWM1, PWM2], GPIO.OUT)

    global pwm1, pwm2
    pwm1 = GPIO.PWM(PWM1, 1000)
    pwm2 = GPIO.PWM(PWM2, 1000)
    pwm1.start(0)
    pwm2.start(0)

# ==============================
# PID PARAMETERS
# ==============================
Kp = 22.0
Ki = 0.2
Kd = 1.5
MAX_INTEGRAL = 50.0

# ==============================
# MOVEMENT FUNCTIONS (GYRO-BASED)
# ==============================
def move_forward(duration=1):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(duration)
    stop()

def rotate_left(duration=0.2):
    rotate_to_angle(-30, timeout=duration)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# ==============================
# ROTATION BASED ON PID
# ==============================
def rotate_based_on_pid(pid_output):
    min_power = 60
    max_power = 100
    power = min(max(abs(pid_output), min_power), max_power)

    if pid_output > 5:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    elif pid_output < -5:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        stop()
        return

    pwm1.ChangeDutyCycle(power)
    pwm2.ChangeDutyCycle(power)

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def rotate_to_angle(target_angle, timeout=10):
    error_integral = 0.0
    previous_error = 0.0
    current_angle = 0.0

    start_time = time.time()
    prev_time = start_time

    while time.time() - start_time < timeout:
        now = time.time()
        dt = now - prev_time
        prev_time = now

        if dt < 0.001:
            time.sleep(0.001)
            continue

        rate = gyro.get_gyro_z()
        current_angle += rate * dt
        current_angle = normalize_angle(current_angle)
        error = normalize_angle(target_angle - current_angle)

        error_integral += error * dt
        error_integral = max(min(error_integral, MAX_INTEGRAL), -MAX_INTEGRAL)
        error_derivative = (error - previous_error) / dt if dt > 0 else 0
        previous_error = error

        pid_output = Kp * error + Ki * error_integral + Kd * error_derivative
        rotate_based_on_pid(pid_output)

        if abs(error) < 3.0 and abs(rate) < 5.0:
            stop()
            return True

        time.sleep(0.01)

    stop()
    return False

# ==============================
# RTL-SDR CONFIGURATION AND SPECTRUM ANALYZER
# ==============================
class SpectrumAnalyzer:
    def __init__(self, center_freq=433000000, sample_rate=2.4e6, gain=0):
        self.center_freq = center_freq
        self.sample_rate = sample_rate
        self.gain = gain
        self.sdr = rtlsdr.RtlSdr()
        self.setup_sdr()
        
        # Initialize the spectrum data
        self.freq_range = None
        self.power_spectrum = None
        self.peak_power = -100
        self.avg_power = -100
        self.signal_history = []
        self.max_history_length = 20  # Store last 20 measurements for trend
        
        # Log messages
        self.log_messages = []
        self.max_log_messages = 10  # Keep last 10 messages
        
        # Grafik güncelleme sayacý
        self.update_counter = 0
        
        # System state eklendi - YENÝ KOD
        self.system_state = "IDLE"

        # Setup plot
        plt.ion()  # Turn on interactive mode
        # Figure baþlýðýný kaldýr
        self.fig = plt.figure(figsize=(10, 12))
        # Alt grafikler için daha fazla boþluk býrak
        gs = plt.GridSpec(3, 1, height_ratios=[3, 2, 2], hspace=0.4)
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1])
        self.ax_log = self.fig.add_subplot(gs[2])
        
        # Set window title instead of figure title
        self.fig.canvas.manager.set_window_title('433 MHz Live Spectrum Analyzer')
        
        # Spectrum plot
        self.spectrum_line, = self.ax1.plot([], [], 'r-', lw=1)
        self.peak_line, = self.ax1.plot([], [], 'g*', markersize=10)
        self.ax1.set_xlabel('Frequency (MHz)')
        self.ax1.set_ylabel('Power (dB)')
        self.ax1.set_title('Spectrum Analysis')
        self.ax1.grid(True)
        
        # Signal trend plot
        self.trend_line, = self.ax2.plot([], [], 'b-', lw=2)
        self.ax2.set_xlabel('Measurement')
        self.ax2.set_ylabel('Signal Power (dB)')
        self.ax2.set_title('Signal Power Trend')
        self.ax2.grid(True)
        
        # Log messages area
        self.ax_log.axis('off')
        self.ax_log.set_title('System Messages', fontsize=12)
        self.log_text = self.ax_log.text(0.05, 0.95, '', fontsize=10, 
                                         verticalalignment='top',
                                         family='monospace')

        # Alt kýsýmda yeterli boþluk býrak, pozisyonu yukarý taþý
        self.text_info = self.fig.text(0.02, 0.04, '', fontsize=11)
        
        # Sistem durumu metin alanýný daha yukarý taþý
        self.system_state_text = self.fig.text(0.5, 0.02, '', fontsize=16, 
                                              color='red', weight='bold',
                                              horizontalalignment='center')
        
    def setup_sdr(self):
        print("Initializing SDR...")
        self.sdr.sample_rate = self.sample_rate
        self.sdr.center_freq = self.center_freq
        self.sdr.gain = self.gain
        print(f"SDR Settings: Center Frequency: {self.center_freq/1e6} MHz, Sample Rate: {self.sample_rate/1e6} MHz, Gain: {self.gain} dB")

    def add_log_message(self, message):
        # Add timestamp to message
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # Add to log list and limit size
        self.log_messages.append(log_entry)
        if len(self.log_messages) > self.max_log_messages:
            self.log_messages.pop(0)
            
    # Sistem durumunu güncelleyen yeni fonksiyon - YENÝ KOD
    def update_system_state(self, state):
        self.system_state = state
        # Bir sonraki plot güncellemesinde görüntülenecek

    def get_spectrum(self):
        try:
            samples = self.sdr.read_samples(1024*128)
            
            # Window the data to reduce spectral leakage
            # Use numpy's window function instead of scipy's blackmanharris
            window = np.blackman(len(samples))
            windowed_samples = samples * window
            
            # Perform the FFT and get power
            fft_result = np.fft.fft(windowed_samples)
            fft_result = np.fft.fftshift(fft_result)
            power_spectrum = 10 * np.log10(np.abs(fft_result)**2 + 1e-10)
            
            # Calculate frequency range
            f_shift = np.fft.fftshift(np.fft.fftfreq(len(samples), 1/self.sample_rate))
            freq_mhz = (self.center_freq + f_shift) / 1e6
            
            # Find peak
            peak_idx = np.argmax(power_spectrum)
            peak_freq = freq_mhz[peak_idx]
            peak_power = power_spectrum[peak_idx]
            
            # Calculate average in the center region (to avoid edge effects)
            center_region = slice(len(power_spectrum)//4, 3*len(power_spectrum)//4)
            avg_power = np.mean(power_spectrum[center_region])
            
            # Store data for later use
            self.freq_range = freq_mhz
            self.power_spectrum = power_spectrum
            self.peak_power = peak_power
            self.avg_power = avg_power
            self.peak_freq = peak_freq
            
            # Store history for trend
            self.signal_history.append(peak_power)
            if len(self.signal_history) > self.max_history_length:
                self.signal_history.pop(0)
            
            return peak_power
            
        except Exception as e:
            print(f"Spectrum reading error: {e}")
            return -100

    def update_plot(self):
        # Sadece her 2. çaðrýda güncelleme yap - ancak force_update parametresi ekledik
        self.update_counter += 1
        if self.update_counter % 2 != 0:
            return
            
        if self.freq_range is None or self.power_spectrum is None:
            return
        
        # Update spectrum plot
        self.spectrum_line.set_data(self.freq_range, self.power_spectrum)
        peak_idx = np.argmax(self.power_spectrum)
        self.peak_line.set_data([self.freq_range[peak_idx]], [self.power_spectrum[peak_idx]])
        
        # Set dynamic limits
        min_power = np.min(self.power_spectrum)
        max_power = np.max(self.power_spectrum)
        power_range = max_power - min_power
        
        self.ax1.set_xlim(np.min(self.freq_range), np.max(self.freq_range))
        self.ax1.set_ylim(min_power - 0.1*power_range, max_power + 0.1*power_range)
        
        # Update trend plot
        x_trend = list(range(len(self.signal_history)))
        self.trend_line.set_data(x_trend, self.signal_history)
        
        if len(self.signal_history) > 0:
            self.ax2.set_xlim(0, len(self.signal_history))
            min_trend = min(self.signal_history)
            max_trend = max(self.signal_history)
            trend_range = max(max_trend - min_trend, 10)  # Ensure there's always some range
            self.ax2.set_ylim(min_trend - 0.1*trend_range, max_trend + 0.1*trend_range)
        
        # Update log messages
        if self.log_messages:
            log_text = '\n'.join(self.log_messages)
            self.log_text.set_text(log_text)
        
        # Alt bilgi metnini güncelle - ayrý satýrlara bölerek çakýþmayý önle
        # Peak bilgisini bir satýr, average bilgisini diðer satýr yap
        peak_info = f"Peak Frequency: {self.peak_freq:.3f} MHz | Peak Power: {self.peak_power:.2f} dB"
        avg_info = f"Average Power: {self.avg_power:.2f} dB"
        
        # Ýki satýrlý metin formatý
        info_str = f"{peak_info}\n{avg_info}"
        self.text_info.set_text(info_str)
        
        # Sistem durumu metnini güncelle - YENÝ KOD
        # Sistem durumunu daha büyük ve dikkat çekici yap
        self.system_state_text.set_text(f"System State: {self.system_state}")
        self.system_state_text.set_fontsize(18)  # Daha büyük font
        
        # Durum deðiþikliðini vurgulamak için MOVING DONE ve STOPPED durumlarýnda renk deðiþtir
        if self.system_state == "MOVING DONE":
            self.system_state_text.set_color("green")
        elif self.system_state == "STOPPED":
            self.system_state_text.set_color("blue")
        else:
            self.system_state_text.set_color("red")
        
        # Redraw the figure - Force güncelleme için flush_events'i her zaman çaðýr
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        if hasattr(self, 'sdr'):
            self.sdr.close()
        plt.close(self.fig)
        print("Spectrum analyzer closed.")
        
# ==============================
# LIVE SPECTRUM ANALYZER WITH VEHICLE OPERATION
# ==============================
def get_signal_strength(analyzer):
    return analyzer.get_spectrum()

def find_best_signal(analyzer):
    best_angle = 0
    max_signal = -100
    signal_map = {}

    for i in range(12):
        log_message(f"Step: {i+1}/12: Rotating 30 degrees", analyzer)
        rotate_to_angle(30)  # relative turn
        time.sleep(0.01)  # settle after rotation
        
        # Update spectrum multiple times for better reading
        for _ in range(2):
            signal_strength = get_signal_strength(analyzer)
            analyzer.update_plot()
            time.sleep(0.1)
        
        signal_map[i] = signal_strength
        log_message(f"Signal: {signal_strength:.2f} dB", analyzer)

        if signal_strength > max_signal:
            max_signal = signal_strength
            best_angle = i * 30

    log_message(f"Strongest Signal: {max_signal:.2f} dB at {best_angle} degrees", analyzer)
    return best_angle, max_signal

# ==============================
# NEW IMPROVED MOVEMENT FUNCTION WITH THRESHOLD CHECK - YENÝ KOD
# ==============================
def move_forward_in_steps(analyzer, reference_signal):
    """
    2.5 saniye yerine 3 adimda her birinde 0.8 saniye olmak uzere toplam hareket
    Her adimdan sonra threshold kontrolu yapilir
    Eger threshold degerine ulasilirsa, hareket durdurulur
    """
    log_message("Moving in 3 steps, 0.8 seconds each with threshold checking...", analyzer)
    
    target_found = False
    
    for step in range(3):
        # Her adýmý logla
        log_message(f"Movement step {step+1}/3", analyzer)
        
        # 0.8 saniye ileri git
        move_forward(0.8)
        
        # Her adýmdan sonra durum güncellemesi yap
        analyzer.update_system_state(f"MOVING {step+1}/3")
        
        # Her adýmdan sonra bir ölçüm al ve ekraný güncelle
        current_signal = get_signal_strength(analyzer)
        analyzer.update_plot()
        log_message(f"Signal after step {step+1}: {current_signal:.2f} dB", analyzer)
        
        # Threshold kontrolü - YENÝ KOD
        if current_signal >= reference_signal:
            log_message(f"Target signal threshold reached at step {step+1}! Stopping movement.", analyzer)
            analyzer.update_system_state("THRESHOLD MET")
            target_found = True
            analyzer.update_plot()  # Ekraný bir kez daha güncelle
            return True  # Threshold'a ulaþýldý
        
        # Kýsa bir bekleme
        time.sleep(0.1)
    
    log_message("Movement complete, threshold not reached", analyzer)
    return False  # Threshold'a ulaþýlamadý

# Global variable to control if the signal search should continue
running = False
analyzer = None

def start_signal_search():
    global running, analyzer, gyro
    running = True
    
    try:
        # Initialize gyro
        gyro = MPU6050()
        
        # Setup GPIO
        setup_gpio()
        
        # Initialize spectrum analyzer
        analyzer = SpectrumAnalyzer(center_freq=433000000, sample_rate=2.4e6, gain=0)
        log_message("Spectrum analyzer initialized.", analyzer)
        
        # System state güncellendi - YENÝ KOD
        analyzer.update_system_state("INITIALIZING")
        
        # First, get a reference signal level
        for _ in range(1):  # Take a few measurements to stabilize
            get_signal_strength(analyzer)
            analyzer.update_plot()
            time.sleep(0.1)
        
        # Instead of using the default -100 value, use the actual measured value
        if analyzer.peak_power > -90:
            reference_signal = analyzer.peak_power + 12
        else:
            # If no signal is detected, use a reasonable default
            reference_signal = -80
            
        log_message(f"Reference signal level: {reference_signal:.2f} dB", analyzer)
        
        # System state güncellendi - YENÝ KOD
        analyzer.update_system_state("SCANNING")

        while running:
            # Continuously update the spectrum while searching
            for _ in range(2):
                if not running:
                    break
                get_signal_strength(analyzer)
                analyzer.update_plot()
                time.sleep(0.1)
                
            if not running:
                break
                
            # Find the best signal direction
            best_angle, best_signal = find_best_signal(analyzer)
            
            if not running:
                break
                
            # Rotate to face the strongest signal
            log_message(f"Rotating to best angle: {best_angle} degrees", analyzer)
            rotate_to_angle(best_angle +30)
            time.sleep(0.1)
            
            if not running:
                break
                
            # Update display after rotation
            for _ in range(1):
                if not running:
                    break
                current_signal = get_signal_strength(analyzer)
                analyzer.update_plot()
                time.sleep(0.1)
                
            if not running:
                break
                
            log_message(f"Signal after rotation: {current_signal:.2f} dB", analyzer)
            
            # System state güncellendi - YENÝ KOD
            analyzer.update_system_state("MOVING")
            log_message("Moving toward target...", analyzer)
            
            # YENÝ KOD: Adým adým hareket et ve her adýmda threshold kontrolü yap
            target_found = move_forward_in_steps(analyzer, reference_signal)
            
            if not running:
                break
                
            # Eðer hedef bulunduðunda
            if target_found:
                # System state güncellendi - YENÝ KOD
                analyzer.update_system_state("MOVING DONE")
                log_message("Target signal threshold reached. Stopping...", analyzer)
                
                # Durduðunda ekraný bir kez daha güncelle
                get_signal_strength(analyzer)
                analyzer.update_plot()  # Zorla ekraný güncelle
                time.sleep(0.2)  # Görüntüleyebilmek için biraz bekle
                
                running = False
                break
            else:
                # Threshold kontrolü için son bir ölçüm al
                for _ in range(3):
                    if not running:
                        break
                    current_signal = get_signal_strength(analyzer)
                    analyzer.update_plot()
                    time.sleep(0.1)
                
                log_message(f"Final signal check after movement: {current_signal:.2f} dB", analyzer)
                
                # Son bir kontrol daha yap
                if current_signal >= reference_signal:
                    analyzer.update_system_state("MOVING DONE")
                    log_message("Target signal threshold reached after final check. Stopping...", analyzer)
                    get_signal_strength(analyzer)
                    analyzer.update_plot()
                    time.sleep(0.2)
                    running = False
                    break
                else:
                    log_message("Signal threshold not met, continuing search...", analyzer)
                    # System state güncellendi - YENÝ KOD
                    analyzer.update_system_state("SCANNING")

    except KeyboardInterrupt:
        log_message("Program stopped by user.", analyzer)
    except Exception as e:
        log_message(f"Error during signal search: {e}", analyzer)
    finally:
        # System state güncellendi - YENÝ KOD
        if analyzer is not None:
            analyzer.update_system_state("STOPPED")
        cleanup_resources()

def stop_signal_search():
    global running
    running = False
    log_message("Stopping signal search...", analyzer)
    # System state güncellendi - YENÝ KOD
    if analyzer is not None:
        analyzer.update_system_state("STOPPING")
    time.sleep(0.1)  # Give a moment for the running loop to exit
    cleanup_resources()
    
def cleanup_resources():
    global analyzer
    if analyzer is not None:
        log_message("System stopped.", analyzer)
        # System state güncellendi - YENÝ KOD
        analyzer.update_system_state("STOPPED")
        
        # Ekraný bir kez daha güncelle - YENÝ KOD
        # Böylece STOPPED mesajý ekranda görünecek
        analyzer.update_plot()  # Zorla ekraný güncelle
        time.sleep(0.1)  # Görüntüleyebilmek için biraz bekle
        
        analyzer.close()
        analyzer = None
    stop()
    GPIO.cleanup()
    print("System stopped.")

# ==============================
# SIMPLE GUI 
# ==============================
class SignalFinderApp:
    def __init__(self, master):
        self.master = master
        master.title("Sinyal Bulucu")
        master.geometry("300x200")  # Made taller to accommodate two buttons
        
        # Center the window
        master.eval('tk::PlaceWindow . center')
        
        # Create a frame with padding
        self.frame = tk.Frame(master, padx=20, pady=20)
        self.frame.pack(fill=tk.BOTH, expand=True)
        
        # Create start button
        self.start_button = tk.Button(
            self.frame, 
            text="START",
            command=self.start_program,
            font=("Arial", 16, "bold"),
            bg="#4CAF50",  # Green background
            fg="white",    # White text
            height=2,
            width=10,
            relief=tk.RAISED,
            bd=3
        )
        self.start_button.pack(pady=(0, 10))  # Add padding between buttons
        
        # Create stop button
        self.stop_button = tk.Button(
            self.frame, 
            text="DURDUR",
            command=self.stop_program,
            font=("Arial", 16, "bold"),
            bg="#F44336",  # Red background
            fg="white",    # White text
            height=2,
            width=10,
            relief=tk.RAISED,
            bd=3,
            state=tk.DISABLED  # Initially disabled
        )
        self.stop_button.pack()
        
    def start_program(self):
        # Disable the start button and enable the stop button
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        
        # Start the signal search in a separate thread to avoid freezing the UI
        thread = Thread(target=self.run_signal_search)
        thread.daemon = True  # Thread will exit when main program exits
        thread.start()
    
    def stop_program(self):
        # Disable the stop button
        self.stop_button.config(state=tk.DISABLED, text="DURDURULUYOR...")
        
        # Stop the signal search in a separate thread
        thread = Thread(target=self.run_stop_signal_search)
        thread.daemon = True
        thread.start()
    
    def run_signal_search(self):
        try:
            # Run the signal search function
            start_signal_search()
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Re-enable the start button and disable the stop button when finished
            self.master.after(0, lambda: self.update_buttons_after_stop())
    
    def run_stop_signal_search(self):
        try:
            # Stop the signal search
            stop_signal_search()
        except Exception as e:
            print(f"Error stopping: {e}")
        finally:
            # Re-enable the start button and disable the stop button
            self.master.after(0, lambda: self.update_buttons_after_stop())
    
    def update_buttons_after_stop(self):
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED, text="DURDUR")

# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = SignalFinderApp(root)
    root.mainloop()