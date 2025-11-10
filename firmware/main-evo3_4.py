# Firmware ottimizzato per macchina del gelato - 2025.09.19
# Con gestione della coda degli eventi
# Con moda su n=3,5,7,... valori consecutivi del selettore per filtrare false letture, necessario in standby
# Gestione protocollo BLE ... started wip
# A volte quando esce dallo stand by a causa di una rotazione della manopola, parte il ciclo --> sospendere gli interrupt
# durante la comunicazione BLE
# Rivista completamente la ISR della manopola, aggiunto debounce statistico e flag "count started"

import micropython
micropython.alloc_emergency_exception_buf(100)

from machine import Pin, PWM, UART, Timer, ADC, time_pulse_us, disable_irq, enable_irq
import time
import gc
import CRC16modbus as mb
import bluetooth
from BLEXpress_Peripheral import BLESimplePeripheral
import os
import json

# OTA Update modules
try:
    from ota import OTAUpdater
    # from WIFI_CONFIG import SSID, PASSWORD
    OTA_ENABLED = True
except ImportError:
    OTA_ENABLED = False
    print("OTA non disponibile")

# from NTC import T, RS # Temperatura, ReSistenza
# from info import B_SN, M_SN, BT_N, FW_V, PA_V # Board SN, Machine SN, BlueTooth Name, FirmWare Version, PArameter Version

machine_cfg = {}

# ================================================================================================
# CONFIGURAZIONI E COSTANTI
# ================================================================================================

class Config:
    # Timing
    ADC_SAMPLE_PERIOD = 100  # ms
    LED_BLINK_PERIOD = 500   # ms
    UI_DECODE_DELAY = 30     # ms
    MAIN_LOOP_DELAY = 290    # ms (es. 300ms)
    
    # Soglie e limiti
    MAX_ADC_SAMPLES = 30
    POWER_ON_DELAY = 10000      # ms (test 10s, portare a 30s per produzione)
    STANDBY_TIMEOUT = 60000     # ms (test 1min, portare a 5min per produzione) 
    PAUSE_TIMEOUT = 40000       # ms
    END_TIMEOUT = 120000        # ms
    
    # Calibrazioni motore
    KE = 71.4  # Costante del motore
    RA = 2.85  # Resistenza armatura
    
    # PWM frequencies
    PWM_FREQ_LED = 1000
    PWM_FREQ_BUZZER = 4000
    PWM_FREQ_MOTOR = 20000
    
    # Durate cicli standard (secondi)
    STD_CYCLE_DURATIONS = [60, 120, 180, 240, 300, 360, 420, 480]
    EXT_CYCLE_DURATIONS = [120, 180]
    
    # Profili velocità motore (rpm)
    SPEED_PROFILES = [
        [50.0, 100.0, 50.0],
        [60.0, 100.0, 60.0, 120.5, 55] 
    ]
    
    # Data matrix code generico
    DUMMY_DM_CODE = "yydddfffuuuuuucbpliiiikk"

# Stati della macchina
class MachineStates:
    IDLE = 0xFF
    BLOCKED = 0x00
    READY = 0x01
    RUN_STD = 0x02
    RUN_EXT = 0x18
    END_STD = 0x03
    END_EXT = 0x19
    PAUSE = 0x04
    ERROR = 0x05

# Codici eventi
class EventCodes:
    NO_EVENT = 0
    START_PRESSED = 1
    EXIT_STANDBY = 2
    MOTOR_UP = 3
    KNOB_ROTATION = 4
    CYCLE_END = 5
    MOTOR_DOWN = 6
    EXTERNAL_EVENT = 7
    WARNING_ERROR = 8
    START_LONG_PRESSED = 9

# NACK error codes
class NackCodes:
    PAYLOAD_SIZE = 0x01
    OUT_OF_RANGE = 0x02
    GENERIC_OP = 0x03
    CRC_ERROR = 0x04
    EEPROM_ERR = 0x05
    IFLASH_ERR = 0x06

# Messages control code + function code
class QueryCodes:
    SendCode = b'\x00\x00'
    CancelCode = b'\x00\x01'
    ResetAlarms = b'\x01\x00'
    GetVersion = b'\x01\x01'
    GetStatus = b'\x01\x02'
    SetParameter = b'\x01\x03'
    GetParameter = b'\x01\x04'
    ExtraTime = b'\x01\x05'
    SetInfo = b'\x01\x06'
    GetInfo = b'\x01\x07'
    SelectOption = b'\x01\x08'
    GetBlacklistRecord = b'\x01\x09'
    StartFWImageDownload = b'\x02\x00'
    DownloadFWImagePacket = b'\x02\x01'
    InstallFWImage = b'\x02\x02'
    
# ================================================================================================
# INIZIALIZZAZIONE HARDWARE
# ================================================================================================

class Hardware:
    def __init__(self):
        self._init_pins()
        self._init_pwm()
        self._init_adc()
        self._init_uart()
        
    def _init_pins(self):
        # Pin digitali
        self.builtin_led = Pin('LED', Pin.OUT)
        self.start_pin = Pin('GPIO15', Pin.IN)
        self.pos_sw = Pin('GPIO11', Pin.IN)
        self.c_tec_pow = Pin('GPIO21', Pin.OUT)
        
    def _init_pwm(self):
        # LED RGB
        self.led_r = PWM(Pin('GPIO14'))
        self.led_g = PWM(Pin('GPIO13'))
        self.led_b = PWM(Pin('GPIO12'))
        
        for led in [self.led_r, self.led_g, self.led_b]:
            led.freq(Config.PWM_FREQ_LED)
            
        # Buzzer e motori
        self.buzzer = PWM(Pin('GPIO7'))
        self.buzzer.freq(Config.PWM_FREQ_BUZZER)
        
        self.motor = PWM(Pin('GPIO2'))
        self.motor.freq(Config.PWM_FREQ_MOTOR)
        
        self.pump_fan = PWM(Pin('GPIO8'))
        self.pump_fan.freq(Config.PWM_FREQ_MOTOR)
        
    def _init_adc(self):
        # ADC
        self.i_mot = ADC(26)
        self.t_cav = ADC(27)
        self.v_mot = ADC(28)
        self.t_mcu = ADC(4)
        
    def _init_uart(self):
        # Debug UART
        self.debug = UART(0, baudrate=115200, tx=Pin('GPIO0'), rx=Pin('GPIO1'))
        self.debug.init(bits=8, parity=None, stop=1)

# ================================================================================================
# GESTIONE DATI E SENSORI
# ================================================================================================

class SensorData:
    def __init__(self, max_samples=Config.MAX_ADC_SAMPLES):
        self.max_samples = max_samples
        self.i_mot_buff = []
        self.t_cav_buff = []
        self.v_mot_buff = []
        self.t_mcu_buff = []
        self.et_cav = 0.0
        self.ei_mot = 0.0
        self.ei_mot_offset = 0.0
        
    def add_sample(self, hardware):
        """Aggiunge un campione dai sensori ADC"""
        # Corrente motore (mA)
        i_sample = hardware.i_mot.read_u16() * 0.0719342913 - 23.5
        self.i_mot_buff.append(i_sample)
        
        # Temperatura cavità (resistenza NTC)
        t_sample = (10000 * hardware.t_cav.read_u16()) / (65536 - hardware.t_cav.read_u16())
        self.t_cav_buff.append(t_sample)
        
        # Tensione motore
        v_sample = hardware.v_mot.read_u16() * (3.3/65536) * ((47+10)/10) - 0.1
        self.v_mot_buff.append(v_sample)
        
        # Temperatura MCU
        t_mcu_sample = 27 - (hardware.t_mcu.read_u16() * (3.3/65536) - 0.706) / 0.001721
        self.t_mcu_buff.append(t_mcu_sample)
        
        # Mantieni dimensione buffer costante
        if len(self.i_mot_buff) > self.max_samples:
            self.i_mot_buff.pop(0)
            self.t_cav_buff.pop(0)
            self.v_mot_buff.pop(0)
            self.t_mcu_buff.pop(0)
            
    def get_averages(self):
        """Restituisce le medie dei buffer"""
        if not self.i_mot_buff:
            return 0, 0, 0, 0
            
        return (
            sum(self.i_mot_buff) / len(self.i_mot_buff),
            sum(self.t_cav_buff) / len(self.t_cav_buff),
            sum(self.v_mot_buff) / len(self.v_mot_buff),
            sum(self.t_mcu_buff) / len(self.t_mcu_buff)
        )
        
    def calc_temp_ntc(self, rntc):
        """Calcola temperatura NTC con interpolazione"""
        T = machine_cfg["NTC"]["T"]
        RS = machine_cfg["NTC"]["RS"]

        if rntc < min(RS) or rntc > max(RS): # Controllo di range
            return -999
            
        k = 0
        while k < len(RS) and rntc < RS[k]: # Ricerca intervallo 
            k += 1
            
        if k > 0:
            return T[k] - 5 * (rntc - RS[k]) / (RS[k-1] - RS[k]) # Interpolazione lineare del valore
        else:
            return T[0]

# ================================================================================================
# GESTIONE CODA EVENTI
# ================================================================================================

class EventQueue:
    def __init__(self, max_size=10):
        self.events = []
        self.max_size = max_size
        
    def put(self, event):
        """Aggiunge evento alla coda (thread-safe per interrupt)"""
        if len(self.events) < self.max_size:
            self.events.append(event)
        else:
            # Rimuove il più vecchio se la coda è piena
            self.events.pop(0)
            self.events.append(event)
            
    def get(self):
        """Preleva evento dalla coda (FIFO)"""
        if self.events:
            return self.events.pop(0)
        return None
        
    def is_empty(self):
        """Controlla se la coda è vuota"""
        return len(self.events) == 0
        
    def clear(self):
        """Svuota la coda"""
        self.events.clear()

# ================================================================================================
# GESTIONE INTERFACCIA UTENTE
# ================================================================================================

class UIManager:
    def __init__(self, event_queue, state_machine, hardware):
        self.state_machine = state_machine
        self.event_queue = event_queue
        self.hardware = hardware
        self.start_pulse = 0 # Cattura l'inizio dell'impulso *IRQ
        self.pulse_duration = 0 # Cattura la durata dell'impulso *IRQ
        self.pulse_started = False # Flag di inizio cattura impulso *IRQ
        self.sel = [] # Valore letto del selettore
        self.selector = 0 # Valore corrente del selettore
        self.sw = 0 # Valore letto del tasto
        self.switch = 0 # Valore corrente del tasto
        self.sw_count = 0 # Numero di letture consecutive con tasto premuto
        # Mappatura durate impulsi -> valori
        self.pulse_map = {
             (700, 1100): (0, 'sw_release'),
             (1700, 2100): (1, 'sel_1'),
             (2700, 3000): (1, 'sw_press'),
             (3700, 4100): (2, 'sel_2'),
             (5700, 6100): (3, 'sel_3'),
             (7700, 8100): (4, 'sel_4'),
             (9700, 10100): (5, 'sel_5'),
             (11700, 12100): (6, 'sel_6'),
             (13700, 14100): (7, 'sel_7'),
             (15700, 16100): (8, 'sel_8'),
             }

    def handle_start_irq(self, pin):
        """Handler interrupt per il pin START"""
        state = disable_irq() # Disabilita interrupt
        d1 = time_pulse_us(pin, 0, 20000) # Timeout 20ms
        if 1700 <= d1 <= 2100 or 3700 <= d1 <= 16100: # Evita falsi trigger sotto 3.8ms
            d2 = time_pulse_us(pin, 0, 10000) # Timeout 10ms
            if 700 <= d2 <= 1100 or 2700 <= d2 <= 3100:
                self.pulse_duration = d1 # Selettore
                self._decode_pulse()
                self.decode_events()
                self.pulse_duration = d2 # Tasto
                self._decode_pulse()
                self.decode_events()                
            else:
                print("Pulse out of range")
                pass
        enable_irq(state) # Riabilita interrupt

    def _decode_pulse(self):
        """Decodifica la durata dell'impulso"""
        decoded = False
        for (min_dur, max_dur), (value, action) in self.pulse_map.items():
            if min_dur <= self.pulse_duration <= max_dur:
                decoded = True
                # print(f"Impulso: {self.pulse_duration} us -> {action}") # Debug
                if action == 'sw_release':
                    self.sw = 0
                elif action.startswith('sel_'):
                    self.sel.append(value)
                    if self.state_machine.standby:
                        if len(self.sel) > 3: # In standby filtro più severo
                            self.sel.pop(0)
                    elif len(self.sel) > 1: # In condizioni normali filtro più leggero / nessun filtro
                        self.sel.pop(0)
                elif action == 'sw_press':
                    self.sw = 1
                    self.sw_count += 1
                    self.last_sw_press_duration = self.pulse_duration # Per debug
                break
        if not decoded:
            # print(f"Pulse inconsistent: {self.pulse_duration}") # Debug
            pass

    def decode_events(self):
        """Decodifica gli eventi dell'interfaccia utente"""       
        if len(self.sel) > 0:
            sel = max(set(self.sel), key=self.sel.count)    # Accetto il valore più frequente degli ultimi tre letti
            if sel != self.selector:
                self.selector = sel
                self.event_queue.put(EventCodes.KNOB_ROTATION)
                if self.state_machine.standby: self.event_queue.put(EventCodes.EXIT_STANDBY)
            elif self.sw == 0 and self.sw_count > 0:
                self.switch = self.sw 
                if self.state_machine.standby:
                    self.event_queue.put(EventCodes.EXIT_STANDBY)
                    print("Tasto premuto A: ", self.sw_count, self.last_sw_press_duration) # Debug
                elif self.sw_count < 6:
                    self.event_queue.put(EventCodes.START_PRESSED)
                    print("Tasto premuto B: ", self.sw_count, self.last_sw_press_duration) # Debug
                else:
                    self.event_queue.put(EventCodes.START_LONG_PRESSED)
                self.sw_count = 0

# ================================================================================================
# MACCHINA A STATI
# ================================================================================================

class StateMachine:
    def __init__(self, hardware, sensor_data, event_queue):
        self.event_queue = event_queue
        self.hardware = hardware
        self.sensor_data = sensor_data
        self.current_status = MachineStates.IDLE
        self.old_status = None
        self.status_init_time = time.ticks_ms()
        self.status_elapsed_time = 0
        self.num_pause = 0
        
        # Ciclo
        self.cycle_duration = 0
        self.start_cycle = 0
        self.actual_time = 0
        self.total_time = 0
        self.extra_time = False
        self.option = 0
        self.duty_cycle = 0 # duty cycle del motore
        self.motor_speed = 0 # velocità motore impostata/stimata
        self.alarm_mask = 0x00000000
        self.pos_sw = hardware.pos_sw.value()
        self.flavour = "000"
        self.scanner = 0x0000
        self.current_dm_code = Config.DUMMY_DM_CODE
        
        # LED e audio
        self.r, self.g, self.b, self.blink = 0, 0, 0, False
        self.led_on = False
        self.beep_count = 0
        
        # Standby
        self.standby = False
        
        # Istante di accensione della macchina
        self.power_on_time = time.ticks_ms()
        
    def process_event(self, event):
        """Processa un evento e aggiorna lo stato"""
        if event == EventCodes.START_PRESSED:
            self._handle_start_pressed()
        elif event == EventCodes.EXIT_STANDBY:
            self._handle_exit_standby()
        elif event == EventCodes.MOTOR_UP:
            self._handle_motor_up()
        elif event == EventCodes.KNOB_ROTATION:
            self._handle_knob_rotation()
        elif event == EventCodes.CYCLE_END:
            self._handle_cycle_end()
        elif event == EventCodes.MOTOR_DOWN:
            self._handle_motor_down()
        elif event == EventCodes.EXTERNAL_EVENT:
            self._handle_external_event()
            
    def _handle_start_pressed(self):
        """Gestisce pressione del tasto START"""
        if self.current_status == MachineStates.READY:
            self.start_cycle = time.ticks_ms()
            self.current_status = MachineStates.RUN_STD
            self.num_pause = 0
            
        elif self.current_status in [MachineStates.RUN_STD, MachineStates.RUN_EXT]:
            self.num_pause += 1
            self.status_init_time = time.ticks_ms()
            
            if self.num_pause < 2:
                self.actual_time += time.ticks_diff(time.ticks_ms(), self.start_cycle)
                self.old_status = self.current_status
                self.current_status = MachineStates.PAUSE
            else:
                self.total_time = 0
                self.beep_count = 6
                if self.current_status == MachineStates.RUN_STD:
                    self.current_status = MachineStates.END_STD
                elif self.current_status == MachineStates.RUN_EXT:
                    self.current_status = MachineStates.END_EXT
                    
        elif self.current_status in [MachineStates.END_STD, MachineStates.END_EXT]:
            self.current_status = MachineStates.IDLE
            
            
        elif self.current_status == MachineStates.PAUSE:
            self.start_cycle = time.ticks_ms()
            self.current_status = self.old_status
            self.old_status = None
        else:
            self.beep_count = 2
            
    def _handle_exit_standby(self):
        """Gestisce uscita da standby"""
        self.standby = False
        self.current_status = MachineStates.IDLE
        
    def _handle_motor_up(self):
        """Gestisce sollevamento braccio motore"""
        if self.current_status == MachineStates.READY:
            self.status_init_time = time.ticks_ms()
        elif self.current_status in [MachineStates.RUN_STD, MachineStates.RUN_EXT]:
            self.num_pause += 1
            self.status_init_time = time.ticks_ms()
            if self.num_pause < 2:
                self.actual_time += time.ticks_diff(time.ticks_ms(), self.start_cycle)
                self.old_status = self.current_status
                self.current_status = MachineStates.PAUSE
            else:
                self.total_time = 0
                self.beep_count = 6
                if self.current_status == MachineStates.RUN_STD:
                    self.current_status = MachineStates.END_STD
                elif self.current_status == MachineStates.RUN_EXT:
                    self.current_status = MachineStates.END_EXT
        elif self.current_status in [MachineStates.END_STD, MachineStates.END_EXT]:
            self.current_status = MachineStates.IDLE
            
    def _handle_knob_rotation(self):
        """Gestisce rotazione manopola"""
        if self.current_status == MachineStates.READY:
            self.status_init_time = time.ticks_ms()
            
    def _handle_cycle_end(self):
        """Gestisce fine ciclo"""
        self.total_time = 0
        self.beep_count = 6
        self.status_init_time = time.ticks_ms()
        if self.current_status == MachineStates.RUN_STD:
            self.current_status = MachineStates.END_STD
        elif self.current_status == MachineStates.RUN_EXT:
            self.current_status = MachineStates.END_EXT
            
    def _handle_motor_down(self):
        """Gestisce abbassamento braccio motore"""
        self.status_init_time = time.ticks_ms()

    def _handle_external_event(self):
        """Gestisce la comunicazione in corso"""
        self.status_init_time = time.ticks_ms() # Evita che la macchina tenti di andare in standby durante la comunicazione

    def update_cycle(self, selector):
        """Aggiorna durata ciclo in base al selettore"""
        if 1 <= selector <= len(Config.STD_CYCLE_DURATIONS):
            self.cycle_duration = Config.STD_CYCLE_DURATIONS[selector-1] * 1000
            
    def run_cycle(self):
        """Esegue il ciclo di lavorazione"""
        self.total_time = self.actual_time + time.ticks_diff(time.ticks_ms(), self.start_cycle)
        
        if self.total_time >= self.cycle_duration:
            self.event_queue.put(EventCodes.CYCLE_END)
        else:
            # Calcola velocità motore dal profilo
            speed_profile = Config.SPEED_PROFILES[0]  # Profilo 0 = lento, Profilo 1 = movimentato
            time_progress = self.total_time / self.cycle_duration
            tab_len = len(speed_profile)
            
            i = 0
            while i < tab_len:
                tab_progress = (i + 1) / tab_len
                if time_progress <= tab_progress:
                    break
                i += 1
                
            self.motor_speed = speed_profile[min(i, len(speed_profile)-1)]
            
            # Controllo motore, modello lineare
            avg_i, _, _, _ = self.sensor_data.get_averages()
            self.duty_cycle = (Config.KE * self.motor_speed + Config.RA * (avg_i - self.sensor_data.ei_mot_offset)) / 12000
            self.duty_cycle = max(0, min(1, self.duty_cycle))  # Limita 0-1
            
            self.hardware.pump_fan.duty_u16(65535)  # Pompa e ventole al massimo
            self.hardware.motor.duty_u16(int(self.duty_cycle * 65535))
            self.hardware.c_tec_pow.value(1)
            
            # Se la corrente media del motore supera 1.3A allora interrompere il ciclo
            # Questo valore di soglia dovrebbe però variare in base a self.duty_cycle,
            # perchè il valore di corrente a rotore bloccato dipende dalla tensione applicata
            if (avg_i - self.sensor_data.ei_mot_offset) > 1300:
                self.event_queue.put(EventCodes.CYCLE_END)
        
    def turn_off_loads(self):
        """Spegne tutti i carichi"""
        self.r, self.g, self.b, self.blink = 0, 0, 0, False
        self.hardware.buzzer.duty_u16(0)
        self.hardware.pump_fan.duty_u16(0)
        self.hardware.motor.duty_u16(0)
        self.hardware.c_tec_pow.value(0)
        
    def update_leds(self):
        """Aggiorna stato LED"""
        if self.blink:
            sw = self.led_on
        else:
            sw = 1
            self.beep_count = 0
            
        self.hardware.builtin_led.value(sw)
        self.hardware.led_r.duty_u16(int(sw * self.r * 65535 / 255))
        self.hardware.led_g.duty_u16(int(sw * self.g * 65535 / 255))
        self.hardware.led_b.duty_u16(int(sw * self.b * 65535 / 255))
        
        if self.beep_count > 0:
            self.hardware.buzzer.duty_u16(int(sw * 128 * 65535 / 255))
            self.beep_count -= 1
        else:
            self.hardware.buzzer.duty_u16(0)

# ================================================================================================
# GESTIONE BLUETOOTH
# ================================================================================================

class BluetoothManager:
    global machine_cfg

    def __init__(self, hardware, ui_manager, state_machine, sensor_data, event_queue):
        self.event_queue = event_queue
        self.hardware = hardware
        self.state_machine = state_machine
        self.sensor_data = sensor_data
        self.ui_manager = ui_manager
        self.ble = bluetooth.BLE()
        ble_name = str(machine_cfg["Info"]["Bluetooth Name"][1])
        self.peripheral = BLESimplePeripheral(self.ble, ble_name) # Letto da config.json
        self.peripheral.on_write(self._on_rx)

        # Indirizzo MAC Bluetooth
        addr = self.ble.config('mac')[1]
        machine_cfg["Info"]["Bluetooth Address"] = (
            str(hex(addr[0])) + ":"+
            str(hex(addr[1])) + ":"+
            str(hex(addr[2])) + ":"+
            str(hex(addr[3])) + ":"+
            str(hex(addr[4])) + ":"+
            str(hex(addr[5]))
        )
        with open('config.json', 'w') as f:
            json.dump(machine_cfg, f)

    def _on_rx(self, data):
        """Callback ricezione dati BLE"""
        
        if self.ui_manager.pulse_started: # Ignora i dati se è in corso la lettura del selettore
            return []

        # print(data)
        print("BLE RX:", self._format_packet(data)) # Debug only    

        response = bytearray()
        # Verifica CRC
        if data[-2:] == mb.CRC16(data[:-2]):
            # Extract from data Control_code and Function_code
            cf_code = data[3:5] 
            if cf_code == QueryCodes.SendCode:
                response = self._query_SendCode(data)
            elif cf_code == QueryCodes.CancelCode:
                response = self._query_CancelCode()
            elif cf_code == QueryCodes.ResetAlarms:
                response = self._query_ResetAlarms()
            elif cf_code == QueryCodes.GetVersion:
                response = self._query_GetVersion()
            elif cf_code == QueryCodes.GetStatus:
                response = self._query_GetStatus()
            elif cf_code == QueryCodes.SetParameter:
                response = self._query_SetParameters()
            elif cf_code == QueryCodes.GetParameter:
                response = self._query_GetParameter()
            elif cf_code == QueryCodes.ExtraTime:
                response = self._query_ExtraTime(data)
            elif cf_code == QueryCodes.SetInfo:
                response = self._query_SetInfo(data)
            elif cf_code == QueryCodes.GetInfo:
                response = self._query_GetInfo(data[7]) # Information identifier, i dati iniziano in posizione 7
            elif cf_code == QueryCodes.SelectOption:
                response = self._query_SelectOption(data)
            elif cf_code == QueryCodes.GetBlacklistRecord:
                response = self._query_GetBlacklistRecord()
            elif cf_code == QueryCodes.StartFWImageDownload:
                response = self._query_StartFWImageDownload()
            elif cf_code == QueryCodes.DownloadFWImagePacket:
                response = self._query_DownloadFWImagePacket()
            elif cf_code == QueryCodes.InstallFWImage:
                response = self._query_InstallFWImage()
            else:
                # Rihiesta non gestita!
                return []
            
            # Invio risposta  
            if self.peripheral.is_connected():
                # print(cf_code, response)
                response = response + mb.CRC16(response)
                print("BLE TX:", self._format_packet(response)) # Debug only
                self.peripheral.send(response)
        else:
            # CRC error!
            print("CRC error in received packet")
            pass

        if self.state_machine.standby:
            # Uscita da standby se arrivano dati BLE
            self.event_queue.put(EventCodes.EXIT_STANDBY)
        else:
            # evento comunicazione in corso
            self.event_queue.put(EventCodes.EXTERNAL_EVENT)
            pass

        return []
        
    def _format_packet(self, data: bytes):
        """Formatta pacchetto per debug"""
        pkt = data.hex().upper()
        return '|' + '|'.join(pkt[i:i+2] for i in range(0, len(pkt), 2)) + '|'

    def _query_SendCode(self, data):
        dm_code = data[7:7+24] # il data matrix code contiene 24 caratteri
        # !!!!!! dm_code contiene i codici ascii dei numeri, per i calcoli e i confronti vanno convertiti in interi !!!!!!
        # Controllo checksum del datamatrix
        temp = [3,7,3,7,3,7,3,7,3,7,3,7,3,7,3,7,3,7,3,7,3,7]
        for i in range(len(temp)):
            temp[i] = temp[i] * int(chr(dm_code[i])) # converto da byte ASCII a intero !!!
        r = sum(temp)%100
        chk1 = int((99-r)/10)
        chk2 = (99-r)%10
        if (int(chr(dm_code[-2])) == chk1) and (int(chr(dm_code[-1])) == chk2):
            self.current_dm_code = dm_code
            self.state_machine.flavour = dm_code[5:8] # estraggo il codice ascii del flavour per il GetStatus

            record = {
                "dm_code": dm_code.decode('utf-8'),
                "start_time": time.localtime(),
                "status": 1 # 1 = selezionato, 2 = cancellato, 3 = avviato ma non completato, 4 = completato
            }
            with open('cycles.json', 'a') as f:
                f.write(json.dumps(record) + '\n')
            
            # print("Datamatrix valido", dm_code)
            return b'\xAA\x00\x01\x00\x80\x00\x00' # ACK
        else:
            print("Errore di checksum nel datamatrix: ", dm_code, r, chk1, chk2)
            return b'\xAA\x00\x01\x80\x80\x00\x01\x03' # NACK + codice errore "03 = Generic operation error"
    
    def _query_CancelCode(self):
        # controllare la validità del datamatrix
        if self.state_machine.flavour != b'\x30\x30\x30': # diverso da "000"
            # TO DO cancellare il codice dal file dei cicli o marcarlo come "cancellato"
            self.state_machine.flavour = "000".encode('utf-8')
            return b'\xAA\x00\x01\x00\x81\x00\x00' # ACK
        else:
            return b'\xAA\x00\x01\x80\x81\x00\x01\x03' # NACK + codice errore "03 = Generic operation error"
    
    def _query_ResetAlarms(self):
        return b'\xAA\x00\x01\x01\x80\x00\x00' # ACK
        pass
    
    def _query_GetVersion(self):
        res = bytearray(b'\xAA\x00\x01\x01\x81\x00\x10')
        res = res + bytearray(16)
        FW_V = machine_cfg["Info"]["Firmware Version"][1]
        L = min(15, machine_cfg["Info"]["Firmware Version"][0])
        res[7:7+L] = FW_V[0:L].encode('utf-8')
        return res

    def _query_GetStatus(self):
        
        avg_i, avg_t, avg_v, avg_t_mcu = self.sensor_data.get_averages()
        
        res = b'\xAA\x00\x01\x01\x82\x00\x25'
        res = res + self.state_machine.current_status.to_bytes(1)
        res = res + int(self.state_machine.cycle_duration/100).to_bytes(4) # Durata del ciclo (ds)
        res = res + int(self.state_machine.total_time/100).to_bytes(4) # Avanzamento ciclo (ds)
        res = res + int(10*self.sensor_data.et_cav).to_bytes(2) # Temperatura sistema/cavità (d°C)
        res = res + int(10*avg_t_mcu).to_bytes(2) # temperatura ambiente/mcu (d°C)
        res = res + int(avg_i - self.sensor_data.ei_mot_offset).to_bytes(2) # Corrente media assorbita dal motore (mA)
        res = res + b'\x00\x00' # Tensione TEC: non viene più misurata, si misura la temperatura della cavità
        res = res + int(1000*avg_v).to_bytes(2) # Tensione di alimentazione dei carichi (mV)
        res = res + int(avg_i - self.sensor_data.ei_mot_offset).to_bytes(2) # Corrente media assorbita dal motore (mA)
        res = res + int(100*self.state_machine.duty_cycle).to_bytes(1) # duty cycle del motore (%)
        res = res + int(self.state_machine.duty_cycle*1000*avg_v).to_bytes(2) # Tensione di alimentazione del motore (mV)
        res = res + int(self.state_machine.motor_speed).to_bytes(2) # Velocità stimata del motore (rpm)
        res = res + int(self.state_machine.alarm_mask).to_bytes(4)
        res = res + bytes([self.hardware.pos_sw.value()]) # Posizione braccio motore
        res = res + self.state_machine.flavour # Flavour 3 caratteri ASCII
        res = res + int(self.state_machine.option).to_bytes(1) # cream / solid
        res = res + int(self.state_machine.scanner).to_bytes(2) # scanner optico (obsoleto) o altra espansione   
        return res 
    
    def _query_SetParameters(self):
        return b'\xAA\x00\x01\x81\x83\x00\x01\x03' # NACK, non gestito
        pass
    
    def _query_GetParameter(self):
        return b'\xAA\x00\x01\x81\x84\x00\x01\x03' # NACK, non gestito
        pass
    
    def _query_ExtraTime(self, data):
        # da verificare ...... con extratime = 0 si esce dal ciclo, importante per l'App
        if data[7] not in [0, 1]:
            return b'\xAA\x00\x01\x80\x85\x00\x01\x03' # NACK + codice errore "03 = Generic operation error"
        self.state_machine.extra_time = (data[7] == 1)
        return b'\xAA\x00\x01\x00\x85\x00\x00' # ACK

    
    def _query_SetInfo(self, data):
        # Da verificare ....
        info_id = data[7]
        info_data = data[8:list(data).index(0)] # dati terminati da 0

        if info_id == 0x00: # Board Serial Number
            machine_cfg["Info"]["Board Serial Number"][1] = info_data[0:machine_cfg["Info"]["Board Serial Number"][0]].decode('utf-8')
        elif info_id == 0x01: # Machine Serial Number
            machine_cfg["Info"]["Machine Serial Number"][1] = info_data[0:machine_cfg["Info"]["Machine Serial Number"][0]].decode('utf-8')
        elif info_id == 0x02: # Bluetooth Name
            machine_cfg["Info"]["Bluetooth Name"][1] = info_data[0:machine_cfg["Info"]["Bluetooth Name"][0]].decode('utf-8')
        elif info_id == 0x03: # Parameter Version
            machine_cfg["Info"]["Parameter Version"][1] = info_data[0:machine_cfg["Info"]["Parameter Version"][0]].decode('utf-8')
        elif info_id == 0x04: # Bluetooth Address
            pass
        elif info_id == 0x05: # WiFi SSID
            machine_cfg["Network"]["SSID"][1] = info_data[0:machine_cfg["Network"]["SSID"][0]].decode('utf-8')
        elif info_id == 0x06: # WiFi Password
            machine_cfg["Network"]["PASSWORD"][1] = info_data[0:machine_cfg["Network"]["PASSWORD"][0]].decode('utf-8')
        else:
            # Infomation Identifier non gestito
            return b'\xAA\x00\x01\x81\x86\x00\x01\x03' # NACK, non gestito
        with open('config.json', 'w') as f:    
            json.dump(machine_cfg, f)

        return b'\xAA\x00\x01\x01\x86\x00\x00' # ACK
    
    def _query_GetInfo(self, info_id):
        res = bytearray(b'\xAA\x00\x01\x01\x87\x00\x21')
        res = res + bytearray(32)
    
        if info_id == 0x00: # Board Serial Number
            info_data = machine_cfg["Info"]["Board Serial Number"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Info"]["Board Serial Number"][0])
        elif info_id == 0x01: # Machine Serial Number
            info_data = machine_cfg["Info"]["Machine Serial Number"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Info"]["Machine Serial Number"][0])
        elif info_id == 0x02: # Bluetooth Name
            info_data = machine_cfg["Info"]["Bluetooth Name"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Info"]["Bluetooth Name"][0])
        elif info_id == 0x03: # Parameter Version
            info_data = machine_cfg["Info"]["Parameter Version"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Info"]["Parameter Version"][0])
        elif info_id == 0x04: # Bluetooth Address
            info_data = machine_cfg["Info"]["Bluetooth Address"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Info"]["Bluetooth Address"][0])
        elif info_id == 0x05: # WiFi SSID
            info_data = machine_cfg["Network"]["SSID"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Network"]["SSID"][0])
        elif info_id == 0x06: # WiFi Password
            info_data = machine_cfg["Network"]["PASSWORD"][1]
            L = min(machine_cfg["Max Data Length"], machine_cfg["Network"]["PASSWORD"][0])
        else:
            # Infomation Identifier non gestito
            return b'\xAA\x00\x01\x81\x87\x00\x01\x03' # NACK, non gestito

        # res[7] = info_id
        res[7:7+L] = info_data[0:L].encode('utf-8')     
        return res
    
    def _query_SelectOption(self, data):
        # da verificare ......
        if data[7] not in [0, 1]:
            return b'\xAA\x00\x01\x81\x88\x00\x01\x03' # NACK + codice errore "03 = Generic operation error"
        self.state_machine.option = data[7] # 0 = cream, 1 = solid
        return b'\xAA\x00\x01\x01\x88\x00\x00' # ACK
    
    def _query_GetBlacklistRecord(self):
        return b'\xAA\x00\x01\x81\x89\x00\x01\x03' # NACK, non gestito 
        pass
    
    def _query_StartFWImageDownload(self):
        return b'\xAA\x00\x01\x82\x80\x00\x01\x03' # NACK, non gestito 
        pass
    
    def _query_DownloadFWImagePacket(self):
        return b'\xAA\x00\x01\x82\x81\x00\x01\x03' # NACK, non gestito
        pass
    
    def _query_InstallFWImage(self):
        return b'\xAA\x00\x01\x82\x82\x00\x01\x03' # NACK, non gestito
        pass

# ================================================================================================
# APPLICAZIONE PRINCIPALE
# ================================================================================================

class GelatoMachine:   
    def __init__(self):
        global machine_cfg
        
        print("Inizializzazione macchina del gelato...")
        # Load machine configuration from file
        if 'config.json' in os.listdir():
            with open('config.json') as f:
                machine_cfg = json.load(f)
            print(f"Current machine configuration is \n'{json.dumps(machine_cfg)}") # Debug only
        else:
            with open('default.json') as f:
                machine_cfg = json.load(f)
        
        # Inizializza coda eventi condivisa
        self.event_queue = EventQueue()   
        # Inizializza componenti
        self.sensor_data = SensorData()
        self.hardware = Hardware()
        self.state_machine = StateMachine(self.hardware, self.sensor_data, self.event_queue)
        self.ui_manager = UIManager(self.event_queue, self.state_machine, self.hardware)
        self.bluetooth = BluetoothManager(self.hardware, self.ui_manager, self.state_machine, self.sensor_data, self.event_queue)

        self._setup_timers()

        # OTA Update (se disponibile)
        if OTA_ENABLED:
            self._check_ota_update()
            
        print("Inizializzazione completata!")
        heading_str = (
            f'{"STATUS":>6}\t'
            f'{"I_MOT":>7}\t'
            f'{"T_CAV":>7}\t'
            f'{"V_MOT":>7}\t'
            f'{"T_MCU":>7}\t'
            f'{"POS_SW":>6}\t'
            f'{"SELECT":>6}\t'
            f'{"S_TIME":>7}\t'
            f'{"STANDBY":>8}\t'
            f'{"T_TIME":>7}\t'
            f'{"C_TIME":>7}\t'
        )
        print(heading_str.rstrip(), end = '\n') # Debug only

        self._setup_interrupts()

    def _setup_interrupts(self):
        """Configura interrupt hardware"""
        self.hardware.start_pin.irq(
            trigger=Pin.IRQ_FALLING,
            handler=self.ui_manager.handle_start_irq
        )
        self.hardware.pos_sw.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=self._pos_sw_irq_handler
        ) 
        
    def _setup_timers(self):
        """Configura timer"""
        # Timer per campionamento ADC
        self.adc_timer = Timer()
        self.adc_timer.init(
            mode=Timer.PERIODIC,
            period=Config.ADC_SAMPLE_PERIOD,
            callback=lambda t: self.sensor_data.add_sample(self.hardware)
        )
        
        # Timer per LED/Buzzer
        self.led_timer = Timer()
        self.led_timer.init(
            mode=Timer.PERIODIC,
            period=Config.LED_BLINK_PERIOD,
            callback=self._led_timer_callback
        )
        
    def _led_timer_callback(self, timer):
        """Callback timer LED"""
        self.state_machine.led_on = not self.state_machine.led_on
        self.state_machine.update_leds()
        
    def _pos_sw_irq_handler(self, pin):
        """Handler interrupt switch posizione"""
        if self.state_machine.standby:
            self.event_queue.put(EventCodes.EXIT_STANDBY)
        else:
            if pin.value() == 1:  # Braccio alzato
                self.state_machine.r, self.state_machine.g, self.state_machine.b = 150, 245, 0
                self.state_machine.blink = False
                self.hardware.motor.duty_u16(0)
                self.event_queue.put(EventCodes.MOTOR_UP)
            else:  # Braccio abbassato
                self.event_queue.put(EventCodes.MOTOR_DOWN)
            
    def _check_ota_update(self):
        """Controlla e applica aggiornamenti OTA"""
        try:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 0, 128, 0
            self.state_machine.blink = True
            
            firmware_url = "https://github.com/Marce68/Pi-Pico-GEM/"
            ota_updater = OTAUpdater(machine_cfg["Network"]["SSID"],
                                     machine_cfg["Network"]["PASSWORD"],
                                     firmware_url,
                                     "test.py")
            ota_updater.download_and_install_update_if_available()
        except Exception as e:
            print(f"Errore OTA: {e}")
            
    def _dump_debug_data(self):
        """Output dati di debug"""
        avg_i, _ , avg_v, avg_t_mcu = self.sensor_data.get_averages()
        
        dbg_str = (
            f"{self.state_machine.current_status:>6d}\t"
            f"{avg_i:>7.2f}\t"
            f"{self.sensor_data.et_cav:>7.2f}\t"
            f"{avg_v:>7.2f}\t"
            f"{avg_t_mcu:>7.2f}\t"
            f"{self.hardware.pos_sw.value():>6d}\t"
            f"{self.ui_manager.selector:>6d}\t"
            f"{self.state_machine.status_elapsed_time:>7d}\t"
            f"{self.state_machine.standby:>8g}\t"
            f"{self.state_machine.total_time:>7d}\t"
            f"{self.state_machine.cycle_duration:>7d}\t"
            ) # debug only......
        
        print(dbg_str.rstrip(), end = "\n")

        try:
            # self.hardware.debug.write(dbg_str)
            pass
        except:
            # print("Errore scrittura su seriale di debug")
            pass
            
    def run(self):
        """Loop principale"""
        while True:
            if not self.ui_manager.pulse_started:
                try:
                    # Processa gli eventi in coda elinimato:
                    while not self.event_queue.is_empty():
                        self.state_machine.process_event(self.event_queue.get())

                    # Per catturare eventuali stati incoerenti e terminare l'esecuzione per ispezione delle variabili (Thonny)
                    # raise KeyboardInterrupt
                        
                    # Aggiorna durata ciclo
                    self.state_machine.update_cycle(self.ui_manager.selector)

                    # Debug output
                    self._dump_debug_data()
                    
                    # Calcola temperatura NTC
                    _, avg_t_cav, _, _ = self.sensor_data.get_averages()
                    self.sensor_data.et_cav = self.sensor_data.calc_temp_ntc(avg_t_cav)
                    
                    # Gestione stati macchina
                    self._handle_machine_states()
                
                    # Garbage collection se necessario
                    if gc.mem_free() < 10240: # Soglia 10KB
                        gc.collect()
                    
                    time.sleep_ms(Config.MAIN_LOOP_DELAY)

                except Exception as e:
                    print(f"Errore nel loop principale: {e}")
                    time.sleep_ms(1000)
            
                
    def _handle_machine_states(self):
        """Gestisce logica stati macchina"""
        power_on_elapsed = time.ticks_diff(time.ticks_ms(), self.state_machine.power_on_time)
        self.state_machine.status_elapsed_time = time.ticks_diff(time.ticks_ms(), self.state_machine.status_init_time)
        
        # Braccio motore alzato - priorità massima
        if self.hardware.pos_sw.value() == 1:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 150, 245, 0
            self.state_machine.blink = False
            self.hardware.motor.duty_u16(0)
            
            if self.state_machine.current_status == MachineStates.IDLE:
                # Attiva pompa/ventole e spegne tutto il resto dopo delay di accensione
                if power_on_elapsed > Config.POWER_ON_DELAY:
                    self.hardware.pump_fan.duty_u16(65535)
                else:
                    self.state_machine.turn_off_loads()
                    
        # Transizione Idle -> Ready
        elif self.state_machine.current_status == MachineStates.IDLE:
            self.state_machine.current_status = MachineStates.READY
            self.state_machine.status_init_time = time.ticks_ms()
            self.hardware.motor.duty_u16(0) # Necessario quando si torna in IDLE dopo un ciclo
            self.hardware.c_tec_pow.value(0) # Necessario quando si torna in IDLE dopo un ciclo
             # self.state_machine.turn_off_loads()
            
        # Stato Ready
        elif self.state_machine.current_status == MachineStates.READY:
            avg_i, _, _, _ = self.sensor_data.get_averages()
            self.sensor_data.ei_mot_offset = avg_i  # Offset corrente
            
            if self.state_machine.status_elapsed_time >= Config.STANDBY_TIMEOUT:
                self.state_machine.standby = True
                self.state_machine.turn_off_loads()
            else:
                self.state_machine.r, self.state_machine.g, self.state_machine.b = 0, 201, 204
                self.state_machine.blink = False
                self.hardware.pump_fan.duty_u16(65535)
                
        # Stati di esecuzione
        elif self.state_machine.current_status in [MachineStates.RUN_STD, MachineStates.RUN_EXT]:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 0, 201, 204
            self.state_machine.blink = True
            self.state_machine.run_cycle()
               
        # Stato Pausa
        elif self.state_machine.current_status == MachineStates.PAUSE:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 255, 255, 255
            self.state_machine.blink = True
            self.hardware.motor.duty_u16(0)
            
            if self.state_machine.status_elapsed_time >= Config.PAUSE_TIMEOUT:
                self.state_machine.beep_count = 2
                
        # Stati di fine
        elif self.state_machine.current_status in [MachineStates.END_STD, MachineStates.END_EXT]:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 0, 0, 255
            self.state_machine.blink = True
            self.hardware.buzzer.duty_u16(0)
            
            if self.state_machine.status_elapsed_time >= Config.END_TIMEOUT:
                self.hardware.motor.duty_u16(0)
                self.hardware.c_tec_pow.value(0)
                self.state_machine.beep_count = 2
                
        # Cattura stati non gestiti
        else:
            self.state_machine.r, self.state_machine.g, self.state_machine.b = 255, 0, 0
            self.state_machine.blink = False

# ================================================================================================
# PUNTO DI INGRESSO
# ================================================================================================
gem_machine = GelatoMachine() # Per ispezionare le variabili dopo stop con CTRL+C
def main():
    """Funzione principale"""
    global gem_machine

    try:
        # gem_machine = GelatoMachine()
        gem_machine.run()
    except KeyboardInterrupt:
        # CTRL+C in Windows
        print("\nArresto richiesto dall'utente")
    except Exception as e:
        print(f"Errore critico: {e}")
        # Reset del sistema in caso di errore grave
        import machine
        machine.reset()

if __name__ == "__main__":
    main()