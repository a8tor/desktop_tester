import serial
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.dates as mdates
import datetime
import csv
import os
import sys
from threading import Thread, Event

# Modbus register addresses (copied from prog.py)
REGISTERS = {
    # Control
    "TESTER_ADDR": 0x01,
    "BIZ_TYPE": 0x02,
    "TST_TYPE": 0x03,
    "CTRL": 0x04,

    # BIZ values
    "BIZ_IN_U": 0x0A,
    "BIZ_OUT_U": 0x0F,

    # Current values
    "I_IN": 0x09,
    "I_OUT": 0x08,
    "U_TP": 0x0B,
    "U_C": 0x0C,
    "U_IN": 0x0D,
    "U_OUT": 0x0E,
    "TST_SENT_COUNT": 0x05,
    "TST_ERROR_COUNT": 0x06,

    # Calibration
    "I_OUT_CAL_A": 0x10,
    "I_OUT_CAL_B": 0x11,
    "I_IN_CAL_A": 0x12,
    "I_IN_CAL_B": 0x13,
    "U_TP_CAL_A": 0x16,
    "U_TP_CAL_B": 0x17,
    "U_C_CAL_A": 0x18,
    "U_C_CAL_B": 0x19,
    "U_IN_CAL_A": 0x1A,
    "U_IN_CAL_B": 0x1B,
    "U_OUT_CAL_A": 0x1C,
    "U_OUT_CAL_B": 0x1D,

    # Допуски BIZ 4 (Tolerances BIZ4)
    "U_OUT_OFF_MIN_BIZ4": 0x20, # U_вых при выкл и при нагрузке показывать
    "U_OUT_OFF_MAX_BIZ4": 0x21, 
    "dU_OUT_MIN_BIZ4": 0x22, # разница U_вых без нагрузки и с нарузкой(используется только при автоматическом тесте)
    "dU_OUT_MAX_BIZ4": 0x23,
    "U_TP_MIN_BIZ4": 0x24, # U_кт при любой нагруке
    "U_TP_MAX_BIZ4": 0x25,
    "I_OUT_SHORT_MIN_BIZ4": 0x26, # I_вых при коротком
    "I_OUT_SHORT_MAX_BIZ4": 0x27,
    "I_OUT_FAIL_MIN_BIZ4": 0x28, # I_вых при отказе
    "I_OUT_FAIL_MAX_BIZ4": 0x29,
    "I_IN_OFF_MIN_BIZ4": 0x2A, # I_вход при выкл
    "I_IN_OFF_MAX_BIZ4": 0x2B,
    "I_IN_LOAD_MIN_BIZ4": 0x2C, # I_вход при нагрузке
    "I_IN_LOAD_MAX_BIZ4": 0x2D,
    "I_IN_SHORT_MIN_BIZ4": 0x2E, # I_вход при коротком
    "I_IN_SHORT_MAX_BIZ4": 0x2F,
    "I_IN_FAIL_MIN_BIZ4": 0x30, # I_вход при отказе
    "I_IN_FAIL_MAX_BIZ4": 0x31,

    # Допуски BIZ 5 (Tolerances BIZ5)
    "U_OUT_OFF_MIN_BIZ5": 0x32,
    "U_OUT_OFF_MAX_BIZ5": 0x33,
    "dU_OUT_MIN_BIZ5": 0x34,
    "dU_OUT_MAX_BIZ5": 0x35,
    "U_TP_MIN_BIZ5": 0x36,
    "U_TP_MAX_BIZ5": 0x37,
    "I_OUT_SHORT_MIN_BIZ5": 0x38,
    "I_OUT_SHORT_MAX_BIZ5": 0x39,
    "I_OUT_FAIL_MIN_BIZ5": 0x3A,
    "I_OUT_FAIL_MAX_BIZ5": 0x3B,
    "I_IN_OFF_MIN_BIZ5": 0x3C,
    "I_IN_OFF_MAX_BIZ5": 0x3D,
    "I_IN_LOAD_MIN_BIZ5": 0x3E,
    "I_IN_LOAD_MAX_BIZ5": 0x3F,
    "I_IN_SHORT_MIN_BIZ5": 0x40,
    "I_IN_SHORT_MAX_BIZ5": 0x41,
    "I_IN_FAIL_MIN_BIZ5": 0x42,
    "I_IN_FAIL_MAX_BIZ5": 0x43,

    # Допуски BIZ 6 (Tolerances BIZ6)
    "U_OUT_OFF_MIN_BIZ6": 0x44,
    "U_OUT_OFF_MAX_BIZ6": 0x45,
    "dU_OUT_MIN_BIZ6": 0x46,
    "dU_OUT_MAX_BIZ6": 0x47,
    "U_TP_MIN_BIZ6": 0x48,
    "U_TP_MAX_BIZ6": 0x49,
    "I_OUT_SHORT_MIN_BIZ6": 0x4A,
    "I_OUT_SHORT_MAX_BIZ6": 0x4B,
    "I_OUT_FAIL_MIN_BIZ6": 0x4C,
    "I_OUT_FAIL_MAX_BIZ6": 0x4D,
    "I_IN_OFF_MIN_BIZ6": 0x4E,
    "I_IN_OFF_MAX_BIZ6": 0x4F,
    "I_IN_LOAD_MIN_BIZ6": 0x50,
    "I_IN_LOAD_MAX_BIZ6": 0x51,
    "I_IN_SHORT_MIN_BIZ6": 0x52,
    "I_IN_SHORT_MAX_BIZ6": 0x53,
    "I_IN_FAIL_MIN_BIZ6": 0x54,
    "I_IN_FAIL_MAX_BIZ6": 0x55,

    "REAL_TIME": 0x80,
    "TEST_COUNT": 0x81,
}

# Форматирование значений регистров
def format_register_value(addr, value):
    """
    Форматирует значение регистра с учетом его типа и возвращает строку с нужной единицей измерения.
    addr: адрес регистра (int)
    value: числовое значение регистра (int)
    """
    # Обратная карта: addr -> имя регистра
    name = None
    for k, v in REGISTERS.items():
        if v == addr:
            name = k
            break
    if name is None:
        return str(value)
    # Логика по примеру из read_and_update
    if name.startswith("U_"):
        return f"{value / 100:.2f} V"
    elif name.startswith("I_"):
        return f"{value / 10:.1f} mA"
    else:
        return str(value)

# Device parameters
DEVICE_ADDRESS = 1  # Modbus slave address, adjust if needed

# --- Modbus и служебные функции (копия из prog.py) ---
def modbus_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def build_read_command(device_addr, start_reg, reg_count):
    cmd = struct.pack('>B B H H', device_addr, 3, start_reg, reg_count)
    crc = modbus_crc(cmd)
    cmd += struct.pack('<H', crc)
    return cmd

def parse_response(response, reg_count):
    if len(response) < 5:
        raise ValueError('Response too short')
    if response[1] & 0x80:
        raise ValueError('Modbus exception')
    byte_count = response[2]
    if byte_count != reg_count * 2:
        raise ValueError('Unexpected byte count')
    values = []
    for i in range(reg_count):
        val = struct.unpack('>H', response[3 + 2 * i: 5 + 2 * i])[0]
        values.append(val)
    return values

def get_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

# --- Главное окно приложения ---
class FullTestApp(tk.Tk):
    def set_widgets_state(self, state):
        # Меняет состояние всех элементов интерфейса, кроме соединения
        for w in self.widgets_to_lock:
            try:
                w.config(state=state)
            except Exception:
                # Для Frame рекурсивно блокируем все дочерние виджеты
                for child in w.winfo_children():
                    try:
                        child.config(state=state)
                    except Exception:
                        pass
                        
    def __init__(self):
        super().__init__()
        self.title("Full Test Modbus RS-485")
        self.geometry("1050x530+500+200")
        # self.resizable(False, False)
        # Привязка обработчика закрытия окна
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        # Статистика связи
        self.request_count = 0
        self.error_count = 0
        
        # --- Окно параметров соединения ---
        self.connection_frame = ttk.LabelFrame(self, text="Параметры соединения")
        self.connection_frame.grid(column=0, row=0, padx=10, pady=10, sticky="nw")
        ttk.Label(self.connection_frame, text="COM порт:").grid(column=0, row=0, padx=5, pady=5, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.connection_frame, textvariable=self.port_var, values=get_serial_ports(), state="readonly", width=10)
        self.port_combo.grid(column=1, row=0, padx=5, pady=5)
        if self.port_combo["values"]:
            self.port_combo.current(0)
            
        # Bind dropdown click event to refresh ports
        def on_dropdown_click(event):
            current_value = self.port_var.get()
            self.port_combo['values'] = get_serial_ports()
            if current_value in self.port_combo['values']:
                self.port_var.set(current_value)
            elif self.port_combo['values']:
                self.port_combo.current(0)
                
        self.port_combo.bind('<Button-1>', on_dropdown_click)
        self.baud_var = tk.StringVar(value="115200")  # Фиксированная скорость 115200
        
        # Создаем фрейм для кнопки и индикатора
        self.conn_control_frame = ttk.Frame(self.connection_frame)
        self.conn_control_frame.grid(column=0, row=1, columnspan=2, padx=5, pady=5, sticky="w")
        
        # Создаем индикатор состояния (красный по умолчанию)
        self.status_indicator = tk.Canvas(self.conn_control_frame, width=20, height=20, bd=0, highlightthickness=0)
        self.status_indicator.grid(column=0, row=0, padx=(0, 5), pady=5, sticky="w")
        self.indicator_id = self.status_indicator.create_rectangle(2, 2, 18, 18, fill="red", outline="black")
        
        # Кнопка подключения
        self.connect_btn = ttk.Button(self.conn_control_frame, text="Подключить", command=self.handle_connect)
        self.connect_btn.grid(column=1, row=0, padx=0, pady=5, sticky="w")
        # --- Окно текущих значений ---
        self.values_frame = ttk.LabelFrame(self, text="Текущие значения", relief="groove", borderwidth=2)
        self.values_frame.grid(column=0, row=1, columnspan=2, rowspan=2, padx=10, pady=10, sticky="nw")
        
        # Переменная для хранения состояния обновления
        self.realtime_update = tk.BooleanVar(value=False)
        self.param_labels = {}
        self.display_params = [
            ("U_вход", 0x0D),
            ("U_конд", 0x0C),
            ("U_вых", 0x0E),
            ("U_кт", 0x0B),
            ("I_вход", 0x09),
            ("I_выход", 0x08),
        ]
        # Создаем шрифты для различных элементов
        param_font = ('Arial', 12, 'bold')
        value_font = ('Courier New', 14)
        tolerance_font = ('Courier New', 12)  # Меньший шрифт для допусков
        header_font = ('Arial', 12, 'normal')  # Тоньше и не жирный шрифт для заголовков
        
        # Создаем заголовки столбцов
        ttk.Label(self.values_frame, text="Тестер", font=header_font).grid(column=1, row=0, padx=10, pady=5, sticky="n")
        ttk.Label(self.values_frame, text="БИЗ", font=header_font).grid(column=3, row=0, padx=10, pady=5, sticky="n")
        ttk.Label(self.values_frame, text="Допуски", font=header_font).grid(column=5, row=0, padx=10, pady=5, sticky="n")
        
        # Создаем столбец Тестер (все значения)
        self.tester_labels = {}
        for i, (name, addr) in enumerate(self.display_params):
            name_label = ttk.Label(self.values_frame, text=name, font=param_font)
            name_label.grid(column=0, row=i+1, padx=(10, 5), pady=5, sticky="w")
            
            # Создаем красивый frame для значения
            value_frame = ttk.Frame(self.values_frame, relief="ridge", borderwidth=2, width=100, height=40)
            value_frame.grid(column=1, row=i+1, padx=(0, 10), pady=5, sticky="nsew")
            value_frame.grid_propagate(False)
            
            lbl = ttk.Label(value_frame, text="N/A", font=value_font, width=8, anchor="center")
            lbl.pack(expand=True)
            
            self.tester_labels[addr] = lbl
            self.param_labels[addr] = lbl  # Для совместимости с существующим кодом
        
        # Создаем столбец БИЗ (пустые ячейки)
        self.biz_labels = {}
        # Сопоставление адресов регистров с именами для отображения
        biz_mapping = {
            0x0E: ("BIZ_OUT_U", 0x0F),  # U_вых отображает BIZ_OUT_U (0x0F)
            0x0B: ("BIZ_IN_U", 0x0A)   # U_кт отображает BIZ_IN_U (0x0A)
        }
        
        # Создаем ячейки БИЗ только для соответствующих строк
        for addr, (display_name, reg_addr) in biz_mapping.items():
            # Находим соответствующую строку в основной таблице
            row_index = 0
            for j, (param_name, param_addr) in enumerate(self.display_params):
                if param_addr == addr:
                    row_index = j + 1
                    break
                    
            # Создаем ячейки для БИЗ с текстом "N/A"
            value_frame = ttk.Frame(self.values_frame, relief="ridge", borderwidth=2, width=100, height=40)
            value_frame.grid(column=3, row=row_index, padx=(0, 10), pady=5, sticky="nsew")
            value_frame.grid_propagate(False)
            
            lbl = ttk.Label(value_frame, text="N/A", font=value_font, width=8, anchor="center")
            lbl.pack(expand=True)
            
            # Сохраняем метку с адресом регистра, из которого будем брать данные
            self.biz_labels[addr] = (lbl, reg_addr)
        
        # Создаем столбец Допуски (пустые ячейки)
        self.tolerance_labels = {}
        # Только определенные параметры для колонки Допуски
        tolerance_params = [
            ("U_вых", 0x0E),
            ("U_кт", 0x0B),
            ("I_вход", 0x09),
            ("I_выход", 0x08)
        ]
        
        for i, (name, addr) in enumerate(tolerance_params):
            # Находим соответствующую строку в основной таблице
            row_index = 0
            for j, (param_name, param_addr) in enumerate(self.display_params):
                if param_addr == addr:
                    row_index = j + 1
                    break
                    
            # Создаем ячейки для Допусков с текстом "N/A"
            value_frame = ttk.Frame(self.values_frame, relief="ridge", borderwidth=2, width=150, height=40)
            value_frame.grid(column=5, row=row_index, padx=(0, 10), pady=5, sticky="nsew")
            value_frame.grid_propagate(False)
            
            lbl = ttk.Label(value_frame, text="N/A", font=tolerance_font, width=14, anchor="center")
            lbl.pack(expand=True)
            
            self.tolerance_labels[addr] = lbl
            
        self.realtime_update.set(1)
        
        # Добавляем поле для ввода ID
        self.id_frame = ttk.Frame(self.values_frame)
        self.id_frame.grid(column=0, row=len(self.display_params)+2, columnspan=6, padx=10, pady=5, sticky="ew")
        
        ttk.Label(self.id_frame, text="ID устройства:").grid(column=0, row=0, padx=5, pady=5, sticky="w")
        
        # Создаем поле ввода для ID
        self.device_id_var = tk.StringVar()
        self.device_id_entry = ttk.Entry(self.id_frame, textvariable=self.device_id_var, width=15)
        self.device_id_entry.grid(column=1, row=0, padx=5, pady=5, sticky="w")
        
        
        # Кнопки Включить/Выключить БИЗ
        self.biz_on_button = ttk.Button(self, text="Включить БИЗ", command=lambda: self.send_ctrl_command(14))
        self.biz_on_button.place(x=10, y=460)
        self.biz_off_button = ttk.Button(self, text="Выключить БИЗ", command=lambda: self.send_ctrl_command(15))
        self.biz_off_button.place(x=10, y=490)


        # --- Окно связи ---
        self.comm_frame = ttk.LabelFrame(self, text="Связь")
        self.comm_frame.place(x=200, y=10)
        #self.comm_frame.grid(column=1, row=0, padx=10, pady=10, sticky="w")
        ttk.Label(self.comm_frame, text="Количество запросов:").grid(column=0, row=0, padx=5, pady=5, sticky="w")
        self.req_count_lbl = ttk.Label(self.comm_frame, text="0", width=8, relief="sunken")
        self.req_count_lbl.grid(column=1, row=0, padx=5, pady=5)
        ttk.Label(self.comm_frame, text="Количество ошибок:").grid(column=0, row=1, padx=5, pady=5, sticky="w")
        self.err_count_lbl = ttk.Label(self.comm_frame, text="0", width=8, relief="sunken")
        self.err_count_lbl.grid(column=1, row=1, padx=5, pady=5)
        # --- Блок выбора нагрузки ---
        self.load_frame = ttk.LabelFrame(self, text="Нагрузка")
        self.load_frame.place(x=500, y=10)
        #self.load_frame.grid(column=3, row=0, rowspan=2, padx=10, pady=10, sticky="nw")
        self.off_btn = ttk.Button(self.load_frame, text="Выкл", command=lambda: self.send_ctrl_command(4))
        self.off_btn.grid(column=0, row=0, padx=5, pady=5, sticky="ew")
        self.load_btn = ttk.Button(self.load_frame, text="Нагрузка", command=lambda: self.send_ctrl_command(5))
        self.load_btn.grid(column=0, row=1, padx=5, pady=5, sticky="ew")
        self.short_btn = ttk.Button(self.load_frame, text="Короткое", command=lambda: self.send_ctrl_command(6))
        self.short_btn.grid(column=0, row=2, padx=5, pady=5, sticky="ew")
        self.fail_btn = ttk.Button(self.load_frame, text="Отказ", command=lambda: self.send_ctrl_command(10))
        self.fail_btn.grid(column=0, row=3, padx=5, pady=5, sticky="ew")

        # Кнопка автотеста и длительного теста справа от блока нагрузки
        self.test_frame = ttk.Frame(self)
        self.test_frame.place(x=500, y=280)
        #self.test_frame.grid(column=3, row=2, padx=10, pady=10, sticky="nw")
        self.test_btn = ttk.Button(self.test_frame, text="Тест", width=10, command=self.run_auto_test)
        self.test_btn.grid(row=0, column=0, padx=5, pady=5)

        # Кнопка для калибровки БИЗ
        self.calibrate_btn = ttk.Button(self.test_frame, text="Калибровка\n      БИЗ", width=13, 
                                      command=self.calibrate_biz)
        self.calibrate_btn.grid(row=2, column=0, padx=5, pady=5)

        # --- Блок выбора типа БИЗ ---
        self.biz_frame = ttk.LabelFrame(self, text="Тип БИЗ")
        self.biz_frame.place(x=500, y=180)
        #self.biz_frame.grid(column=3, row=1, padx=10, pady=60, sticky="nw")

        # BIZ type selection combobox
        self.biz_type_var = tk.StringVar()
        self.biz_type_combo = ttk.Combobox(
            self.biz_frame,
            textvariable=self.biz_type_var,
            values=["БИЗ 4", "БИЗ 5", "БИЗ 6"],
            state="readonly",
            width=10,
        )
        self.biz_type_combo.grid(column=0, row=0, padx=5, pady=5, sticky="ew")
        self.biz_type_combo.current(0)
        # Bind the combobox selection event
        self.biz_type_combo.bind("<<ComboboxSelected>>", self.on_biz_type_changed)

        # Окно для вывода результатов автотеста
        self.results_frame = ttk.LabelFrame(self, text="Результаты теста")
        self.results_frame.place(x=600, y=10)
        #self.results_frame.grid(column=0, row=3, columnspan=4, padx=10, pady=10, sticky="nsew")

        # Добавляем текстовое поле с полосой прокрутки
        self.results_text = tk.Text(self.results_frame, height=30, width=50, wrap="word")
        self.results_text.grid(column=0, row=0, padx=5, pady=5, sticky="nsew")

        # Добавляем полосу прокрутки
        scrollbar = ttk.Scrollbar(self.results_frame, orient="vertical", command=self.results_text.yview)
        scrollbar.grid(column=1, row=0, sticky="ns")
        self.results_text.config(yscrollcommand=scrollbar.set)
        
        # --- Переменные для работы с портом ---
        self.ser = None
        self.device_addr = 1  # Можно сделать выбор адреса
        # --- Таймер обновления ---
        self.after_id = None
        
        # --- Переменные для хранения текущего режима и допусков ---
        self.current_load_mode = "off"  # Возможные значения: "off", "load", "short", "fail"
        # Словарь для хранения допусков для разных БИЗ
        self.tolerance_values = {
            4: {},  # Допуски для БИЗ 4
            5: {},  # Допуски для БИЗ 5
            6: {}   # Допуски для БИЗ 6
        }

        # Список виджетов, которые нужно блокировать до подключения
        self.widgets_to_lock = [
            self.values_frame,
            self.load_frame,
            self.biz_frame,
            self.test_frame,
            self.results_frame,
            self.off_btn, self.load_btn, self.short_btn, self.fail_btn,
            self.test_btn,
            self.biz_type_combo,
            self.biz_on_button, self.biz_off_button
        ]

        # Изначально блокируем все элементы
        self.set_widgets_state("disabled")

    def on_biz_type_changed(self, event=None):
        """Обработчик изменения типа БИЗ"""
        if not hasattr(self, 'ser') or not self.ser or not self.ser.is_open:
            return
            
        # Получаем выбранный тип БИЗ (4, 5 или 6)
        selected_biz = self.biz_type_var.get()
        try:
            biz_type = int(selected_biz.split()[-1])  # Извлекаем число из строки "БИЗ 4"
            
            # Отправляем значение в регистр BIZ_TYPE (адрес 0x03)
            self.write_register(self.device_addr, REGISTERS["BIZ_TYPE"], biz_type)
            
            # Обновляем отображение допусков для выбранного типа БИЗ
            self.update_tolerance_display()

        except Exception as e:
            print(f"Ошибка при изменении типа БИЗ: {e}")

    def build_write_command(self, device_addr, register_addr, value):
        """Build Modbus RTU command to write single register (function code 6)"""
        cmd = bytearray()
        cmd.append(device_addr)
        cmd.append(6)  # Function code 6: Write Single Register
        cmd.append((register_addr >> 8) & 0xFF)
        cmd.append(register_addr & 0xFF)
        cmd.append((value >> 8) & 0xFF)
        cmd.append(value & 0xFF)
        crc = modbus_crc(cmd)
        cmd.append(crc & 0xFF)
        cmd.append((crc >> 8) & 0xFF)
        return cmd

    def write_register(self, device_addr, register_addr, value):
        """Write single holding register (function code 6)."""
        if not self.ser or not self.ser.is_open:
            raise ValueError("Serial port not open")

        cmd = self.build_write_command(device_addr, register_addr, value)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.write(cmd)
        time.sleep(0.1)

        response = self.ser.read(8)
        if len(response) != 8:
            raise ValueError("Invalid response length for write register")
        if response[:6] != cmd[:6]:
            raise ValueError("Response does not match request")

    def read_register(self, device_addr, register_addr):
        """Чтение одного регистра (function code 3)."""
        if not self.ser or not self.ser.is_open:
            raise ValueError("Serial port not open")
            
        cmd = build_read_command(device_addr, register_addr, 1)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.write(cmd)
        time.sleep(0.1)

        response = self.ser.read(7)  # 5 байт заголовка + 2 байта данных
        if len(response) != 7:
            raise ValueError("Invalid response length for read register")

        values = parse_response(response, 1)
        if not values:
            raise ValueError("Failed to parse response")

        return values[0]

    def start_polling(self):
        """Start the polling loop to read values from the tester."""
        # Cancel any existing polling
        if hasattr(self, 'after_id') and self.after_id:
            self.after_cancel(self.after_id)
        # Start polling
        self.poll_device()
        # Schedule next poll in 500ms
        self.after_id = self.after(1000, self.start_polling)

    def calibrate_biz(self):
        """Отправляет команду калибровки БИЗ и приостанавливает опрос на 2 секунды"""
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Ошибка", "Нет соединения с устройством")
            return
        time.sleep(0.3)
        # Отменяем запланированный опрос
        if self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None

        # Отправляем команду калибровки (13)
        self.send_ctrl_command(13)

        # Запускаем новый таймер для возобновления опроса через 1.2 секунды
        # и сохраняем ID таймера
        self.after_id = self.after(1200, self.start_polling)

    def handle_connect(self):
        # Если порт уже открыт, то закрываем его (отключаемся)
        if self.ser and self.ser.is_open:
            # Останавливаем опрос
            if self.after_id:
                self.after_cancel(self.after_id)
                self.after_id = None

            # Записываем 1 в регистр TST_TYPE перед закрытием
            try:
                self.write_register(DEVICE_ADDRESS, REGISTERS["TST_TYPE"], 1)
            except Exception as e:
                # Error writing to TST_TYPE before close (silently handled)
                pass
            # Закрываем порт
            self.ser.close()
            self.ser = None

            # Блокируем интерфейс
            self.set_widgets_state("disabled")

            # Меняем текст кнопки и цвет индикатора
            self.connect_btn.config(text="Подключить")
            self.status_indicator.itemconfig(self.indicator_id, fill="red")
            return

        # Иначе подключаемся
        port = self.port_var.get()
        baud = 115200  # Фиксированная скорость 115200
        try:
            # Открываем новое соединение
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.3)

            # Пробуем получить ответ от тестера, чтобы убедиться в наличии связи
            try:
                # Запрашиваем регистр TST_TYPE и записываем в него двойку
                self.write_register(DEVICE_ADDRESS, REGISTERS["TST_TYPE"], 2)

                # Читаем текущий тип БИЗ, чтобы подтвердить связь с тестером
                biz_type = self.read_register(DEVICE_ADDRESS, REGISTERS["BIZ_TYPE"])

                # Тестер ответил - теперь можно обновить интерфейс
                # Разблокируем интерфейс при успешном подключении
                self.set_widgets_state("normal")

                # Меняем текст кнопки и цвет индикатора ТОЛЬКО после проверки связи с тестером
                self.connect_btn.config(text="Отключить")
                self.status_indicator.itemconfig(self.indicator_id, fill="green")

                # Обновляем Combobox с типом БИЗ
                if 4 <= biz_type <= 6:
                    # Устанавливаем соответствующий индекс в Combobox
                    self.biz_type_combo.current(biz_type - 4)  # Т.к. индексы начинаются с 0, а БИЗ с 4

                # Считываем значения всех допусков для всех типов БИЗ
                self.read_all_tolerance_values()
                time.sleep(0.3)
                self.send_ctrl_command(12)
                # Запускаем опрос значений
                self.start_polling()

            except Exception as e:
                # Если не удалось получить ответ от тестера
                messagebox.showerror("Ошибка связи с тестером", f"Порт открыт, но тестер не отвечает: {str(e)}")

                # Закрываем порт, так как тестер не отвечает
                if self.ser and hasattr(self.ser, 'is_open') and self.ser.is_open:
                    self.ser.close()
                self.ser = None
                return

        except Exception as e:
            # При ошибке открытия порта показываем сообщение об ошибке
            messagebox.showerror("Ошибка открытия порта", str(e))
            # Если порт был открыт до ошибки, закрываем его
            if self.ser and hasattr(self.ser, 'is_open') and self.ser.is_open:
                self.ser.close()
            self.ser = None

    def send_ctrl_command(self, command):
        # Отправляем команду управления (для кнопок нагрузки)
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Ошибка", "Порт не открыт")
            return

        try:
            self.write_register(self.device_addr, REGISTERS["CTRL"], command)

            # Обновляем текущий режим нагрузки
            if command == 4:  # Выкл
                self.current_load_mode = "off"
            elif command == 5:  # Нагрузка
                self.current_load_mode = "load"
            elif command == 6:  # Короткое
                self.current_load_mode = "short"
            elif command == 10:  # Отказ
                self.current_load_mode = "fail"

            # Обновляем отображение допусков для нового режима нагрузки
            if command in [4, 5, 6, 10]:  # Только для команд изменения нагрузки
                self.update_tolerance_display()
        except Exception as e:
            messagebox.showerror("Ошибка отправки команды", str(e))

    def read_all_tolerance_values(self):
        """Считывает все значения допусков для всех типов БИЗ через Modbus"""
        if not self.ser or not self.ser.is_open:
            return

        try:
            # Очищаем существующие значения допусков
            for biz_type in [4, 5, 6]:
                self.tolerance_values[biz_type] = {}
            
            # Список префиксов для регистров допусков каждого типа БИЗ
            tolerance_prefixes = [
                # БИЗ 4
                (4, "BIZ4"),
                # БИЗ 5
                (5, "BIZ5"),
                # БИЗ 6
                (6, "BIZ6")
            ]

            # Считываем все допуски для каждого типа БИЗ
            for biz_type, suffix in tolerance_prefixes:
                # Определяем диапазон регистров для данного БИЗ
                start_reg = None
                end_reg = None
                
                # Находим начальный и конечный адрес регистров для данного типа БИЗ
                for reg_name, reg_addr in REGISTERS.items():
                    if suffix in reg_name:
                        if start_reg is None or reg_addr < start_reg:
                            start_reg = reg_addr
                        if end_reg is None or reg_addr > end_reg:
                            end_reg = reg_addr
                
                if start_reg is not None and end_reg is not None:
                    # Считываем все регистры в диапазоне за один запрос
                    reg_count = end_reg - start_reg + 1
                    cmd = build_read_command(self.device_addr, start_reg, reg_count)
                    self.ser.write(cmd)
                    
                    response = self.ser.read(5 + 2 * reg_count)
                    if response and len(response) >= 5 + 2 * reg_count:
                        values = parse_response(response, reg_count)
                        
                        # Сохраняем значения в словарь допусков
                        for reg_name, reg_addr in REGISTERS.items():
                            if suffix in reg_name and start_reg <= reg_addr <= end_reg:
                                reg_idx = reg_addr - start_reg
                                if 0 <= reg_idx < len(values):
                                    self.tolerance_values[biz_type][reg_name] = values[reg_idx]
            
            # После чтения всех допусков обновляем отображение
            self.update_tolerance_display()
        except Exception as e:
            print(f"Ошибка при чтении допусков: {e}")

    def update_tolerance_display(self, check_current_values=False, current_values=None):
        """Обновляет отображение допусков на основе текущего типа БИЗ и режима нагрузки

        Parameters:
        check_current_values (bool): Если True, проверяет текущие значения на соответствие допускам
        current_values (dict): Словарь с текущими значениями параметров {addr: value}
        """
        if not self.ser or not self.ser.is_open:
            return

        try:
            # Определяем текущий выбранный тип БИЗ
            selected = self.biz_type_var.get()
            biz_type = 4  # По умолчанию БИЗ 4
            
            if selected == "БИЗ 4":
                biz_type = 4
            elif selected == "БИЗ 5":
                biz_type = 5
            elif selected == "БИЗ 6":
                biz_type = 6
            
            # Проверяем, есть ли данные допусков для этого типа БИЗ
            if not self.tolerance_values[biz_type]:
                # Если допуски еще не считаны, пытаемся их считать
                self.read_all_tolerance_values()
                return
            
            # В зависимости от текущего режима нагрузки и параметра определяем нужные допуски
            # Сопоставление параметров с регистрами допусков согласно комментариям
            tolerance_mapping = {
                # Напряжение на выходе (U_вых, 0x0E)
                0x0E: {
                    # U_вых при выкл и при нагрузке показывать
                    "off": (f"U_OUT_OFF_MIN_BIZ{biz_type}", f"U_OUT_OFF_MAX_BIZ{biz_type}"),
                    "load": (f"U_OUT_OFF_MIN_BIZ{biz_type}", f"U_OUT_OFF_MAX_BIZ{biz_type}"),
                    "short": None,
                    "fail": None,
                },
                # Напряжение Ктр (U_кт, 0x0B)
                0x0B: {
                    # U_кт при любой нагруке
                    "off": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                    "load": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                    "short": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                    "fail": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                },
                # Ток на входе (I_вход, 0x09)
                0x09: {
                    # I_вход при соответствующем режиме
                    "off": (f"I_IN_OFF_MIN_BIZ{biz_type}", f"I_IN_OFF_MAX_BIZ{biz_type}"),
                    "load": (f"I_IN_LOAD_MIN_BIZ{biz_type}", f"I_IN_LOAD_MAX_BIZ{biz_type}"),
                    "short": (f"I_IN_SHORT_MIN_BIZ{biz_type}", f"I_IN_SHORT_MAX_BIZ{biz_type}"),
                    "fail": (f"I_IN_FAIL_MIN_BIZ{biz_type}", f"I_IN_FAIL_MAX_BIZ{biz_type}"),
                },
                # Ток на выходе (I_выход, 0x08)
                0x08: {
                    # I_вых при соответствующем режиме
                    "off": None,
                    "load": None,
                    "short": (f"I_OUT_SHORT_MIN_BIZ{biz_type}", f"I_OUT_SHORT_MAX_BIZ{biz_type}"),
                    "fail": (f"I_OUT_FAIL_MIN_BIZ{biz_type}", f"I_OUT_FAIL_MAX_BIZ{biz_type}"),
                },
            }
            
            # Обновляем отображение допусков
            for param_addr, tolerance_lbl in self.tolerance_labels.items():
                if param_addr in tolerance_mapping and self.current_load_mode in tolerance_mapping[param_addr]:
                    # Получаем регистры допусков для данного параметра и режима
                    tolerance_regs = tolerance_mapping[param_addr][self.current_load_mode]
                    
                    if tolerance_regs is None:
                        # Если нет допусков для данного режима, выводим N/A
                        tolerance_lbl.config(text="N/A", background="white")
                    else:
                        # Получаем мин и макс значения допусков
                        min_reg, max_reg = tolerance_regs
                        
                        # Проверяем, есть ли эти регистры в нашем словаре
                        if min_reg in self.tolerance_values[biz_type] and max_reg in self.tolerance_values[biz_type]:
                            min_val = self.tolerance_values[biz_type][min_reg]
                            max_val = self.tolerance_values[biz_type][max_reg]
                            
                            # Форматируем значения в зависимости от типа параметра
                            min_formatted = format_register_value(REGISTERS[min_reg], min_val)
                            max_formatted = format_register_value(REGISTERS[max_reg], max_val)
                            
                            # Получаем только числовые значения без единиц измерения
                            min_value = min_formatted.split(' ')[0]
                            max_value = max_formatted.split(' ')[0]
                            units = min_formatted.split(' ')[1] if ' ' in min_formatted else ''
                            
                            # Обновляем метку с допусками в формате "мин-макс"
                            tolerance_lbl.config(text=f"{min_value}-{max_value} {units}")
                            
                            # Проверяем текущие значения на соответствие допускам, если включен режим реального времени
                            if check_current_values and current_values and param_addr in current_values:
                                current_val = current_values[param_addr]
                                
                                # Определяем, в допуске ли значение
                                # Для этого нужно преобразовать строковые значения в числа
                                try:
                                    min_num = float(min_value)
                                    max_num = float(max_value)
                                    
                                    # Преобразуем текущее значение в числовой формат без единиц измерения
                                    if param_addr in self.tester_labels:
                                        current_text = self.tester_labels[param_addr].cget("text")
                                        current_num = float(current_text.split(' ')[0])
                                        
                                        # Устанавливаем цвет фона в зависимости от соответствия допускам
                                        if min_num <= current_num <= max_num:
                                            # В допуске - зеленый
                                            tolerance_lbl.config(background="#90EE90")  # светло-зеленый
                                        else:
                                            # Вне допуска - красный
                                            tolerance_lbl.config(background="#FFCCCB")  # светло-красный
                                    else:
                                        tolerance_lbl.config(background="white")
                                except (ValueError, TypeError):
                                    # Не удалось преобразовать в число - не меняем цвет
                                    tolerance_lbl.config(background="white")
                            elif not check_current_values:
                                # Если не проверяем значения, сбрасываем цвет фона
                                tolerance_lbl.config(background="white")
                        else:
                            tolerance_lbl.config(text="N/A", background="white")
                else:
                    # Для параметров без допусков
                    tolerance_lbl.config(text="N/A", background="white")
        except Exception as e:
            print(f"Ошибка при обновлении допусков: {e}")

    def run_auto_test(self):
        """Выполняет автоматическое тестирование, перебирая все режимы нагрузки и проверяя соответствие допускам"""
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Ошибка", "Порт не открыт")
            return
            
        # Очищаем текстовое поле с результатами
        self.results_text.delete(1.0, tk.END)
        #self.results_text.insert(tk.END, "Начало автоматического тестирования...\n")
        #self.results_text.insert(tk.END, "====================================\n")
        self.results_text.see(tk.END)  # Прокрутка к последней строке
        self.update()  # Обновляем интерфейс
        
        # Словарь для хранения состояния тестов (пройден/не пройден)
        test_results = {}
        passed_tests = 0
        total_tests = 0
        
        # Режимы нагрузки для тестирования
        load_modes = [
            (4, "Выкл"),
            (5, "Нагрузка"),
            (6, "Короткое"),
            (10, "Отказ")
        ]
        
        try:
            # Перебираем все режимы нагрузки
            for cmd, mode_name in load_modes:
                self.results_text.insert(tk.END, f"\nПроверка режима: {mode_name}\n")
                self.results_text.insert(tk.END, "-----------------\n")
                self.results_text.see(tk.END)  # Прокрутка к последней строке
                self.update()  # Обновляем интерфейс
                
                # Отправляем команду на установку режима
                self.write_register(self.device_addr, REGISTERS["CTRL"], cmd)
                self.current_load_mode = {
                    4: "off",
                    5: "load",
                    6: "short",
                    10: "fail"
                }[cmd]
                
                # Ждем установки режима
                time.sleep(0.5)
                
                # Отправляем команду 9 для обновления значений в тестере
                self.send_ctrl_command(9)
                time.sleep(0.5)
                
                # Запрашиваем значения основных регистров
                start_reg = min(addr for _, addr in self.display_params)
                end_reg = max(addr for _, addr in self.display_params)
                reg_count = end_reg - start_reg + 1
                
                cmd = build_read_command(self.device_addr, start_reg, reg_count)
                self.ser.write(cmd)
                
                # Читаем ответ
                response = self.ser.read(5 + 2 * reg_count)
                if response and len(response) >= 5 + 2 * reg_count:
                    values = parse_response(response, reg_count)
                    
                    # Словарь для хранения текущих значений
                    current_values = {}
                    
                    # Обновляем отображение
                    for name, addr in self.display_params:
                        reg_idx = addr - start_reg
                        if 0 <= reg_idx < len(values):
                            value = values[reg_idx]
                            formatted_value = format_register_value(addr, value)
                            if addr in self.param_labels:
                                self.param_labels[addr].config(text=formatted_value)
                                current_values[addr] = value  # Сохраняем текущее значение
                    
                    # Определяем текущий выбранный тип БИЗ
                    selected = self.biz_type_var.get()
                    biz_type = 4  # По умолчанию БИЗ 4
                    
                    if selected == "БИЗ 4":
                        biz_type = 4
                    elif selected == "БИЗ 5":
                        biz_type = 5
                    elif selected == "БИЗ 6":
                        biz_type = 6
                    
                    # Проверяем значения на соответствие допускам
                    tolerance_mapping = {
                        # Напряжение на выходе (U_вых, 0x0E)
                        0x0E: {
                            # U_вых при выкл и при нагрузке показывать
                            "off": (f"U_OUT_OFF_MIN_BIZ{biz_type}", f"U_OUT_OFF_MAX_BIZ{biz_type}"),
                            "load": (f"U_OUT_OFF_MIN_BIZ{biz_type}", f"U_OUT_OFF_MAX_BIZ{biz_type}"),
                            "short": None,
                            "fail": None,
                        },
                        # Напряжение Ктр (U_кт, 0x0B)
                        0x0B: {
                            # U_кт при любой нагруке
                            "off": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                            "load": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                            "short": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                            "fail": (f"U_TP_MIN_BIZ{biz_type}", f"U_TP_MAX_BIZ{biz_type}"),
                        },
                        # Ток на входе (I_вход, 0x09)
                        0x09: {
                            # I_вход при соответствующем режиме
                            "off": (f"I_IN_OFF_MIN_BIZ{biz_type}", f"I_IN_OFF_MAX_BIZ{biz_type}"),
                            "load": (f"I_IN_LOAD_MIN_BIZ{biz_type}", f"I_IN_LOAD_MAX_BIZ{biz_type}"),
                            "short": (f"I_IN_SHORT_MIN_BIZ{biz_type}", f"I_IN_SHORT_MAX_BIZ{biz_type}"),
                            "fail": (f"I_IN_FAIL_MIN_BIZ{biz_type}", f"I_IN_FAIL_MAX_BIZ{biz_type}"),
                        },
                        # Ток на выходе (I_выход, 0x08)
                        0x08: {
                            # I_вых при соответствующем режиме
                            "off": None,
                            "load": None,
                            "short": (f"I_OUT_SHORT_MIN_BIZ{biz_type}", f"I_OUT_SHORT_MAX_BIZ{biz_type}"),
                            "fail": (f"I_OUT_FAIL_MIN_BIZ{biz_type}", f"I_OUT_FAIL_MAX_BIZ{biz_type}"),
                        },
                    }
                    
                    # Проверяем каждый параметр
                    for param_addr in [0x0E, 0x0B, 0x09, 0x08]:  # Адреса проверяемых параметров
                        param_name = None
                        for name, addr in self.display_params:
                            if addr == param_addr:
                                param_name = name
                                break
                        
                        if param_addr in tolerance_mapping and self.current_load_mode in tolerance_mapping[param_addr]:
                            tolerance_regs = tolerance_mapping[param_addr][self.current_load_mode]
                            
                            if tolerance_regs is None:
                                self.results_text.insert(tk.END, f"{param_name}: Нет допусков для данного режима\n")
                                self.results_text.see(tk.END)
                                continue
                            
                            min_reg, max_reg = tolerance_regs
                            
                            if min_reg in self.tolerance_values[biz_type] and max_reg in self.tolerance_values[biz_type]:
                                min_val = self.tolerance_values[biz_type][min_reg]
                                max_val = self.tolerance_values[biz_type][max_reg]
                                
                                min_formatted = format_register_value(REGISTERS[min_reg], min_val)
                                max_formatted = format_register_value(REGISTERS[max_reg], max_val)
                                
                                min_value = min_formatted.split(' ')[0]
                                max_value = max_formatted.split(' ')[0]
                                units = min_formatted.split(' ')[1] if ' ' in min_formatted else ''
                                
                                if param_addr in current_values:
                                    current_val = current_values[param_addr]
                                    current_formatted = format_register_value(param_addr, current_val)
                                    
                                    try:
                                        min_num = float(min_value)
                                        max_num = float(max_value)
                                        current_num = float(current_formatted.split(' ')[0])
                                        
                                        test_key = f"{mode_name}_{param_name}"
                                        total_tests += 1
                                        
                                        if min_num <= current_num <= max_num:
                                            test_results[test_key] = True
                                            passed_tests += 1
                                            self.results_text.insert(tk.END, f"{param_name}: {current_formatted} ✓ (в допуске {min_value}-{max_value} {units})\n")
                                        else:
                                            test_results[test_key] = False
                                            self.results_text.insert(tk.END, f"{param_name}: {current_formatted} ❌ (вне допуска {min_value}-{max_value} {units})\n")
                                        
                                        self.results_text.see(tk.END)  # Прокрутка к последней строке
                                    except (ValueError, TypeError):
                                        self.results_text.insert(tk.END, f"{param_name}: Ошибка проверки допусков\n")
                                        self.results_text.see(tk.END)
                                else:
                                    self.results_text.insert(tk.END, f"{param_name}: Нет данных\n")
                                    self.results_text.see(tk.END)
                            else:
                                self.results_text.insert(tk.END, f"{param_name}: Нет данных о допусках\n")
                                self.results_text.see(tk.END)
                        else:
                            if param_name:
                                self.results_text.insert(tk.END, f"{param_name}: Нет допусков для данного режима\n")
                                self.results_text.see(tk.END)
                    
                    self.update()  # Обновляем интерфейс после каждого режима
                else:
                    self.results_text.insert(tk.END, f"Ошибка получения данных для режима {mode_name}\n")
                    self.results_text.see(tk.END)
                    self.update()
            
            # Итоговый результат
            self.results_text.insert(tk.END, "\n====================================\n")
            self.results_text.insert(tk.END, f"Итог: Пройдено {passed_tests} из {total_tests} тестов")
            if passed_tests == total_tests and total_tests > 0:
                self.results_text.insert(tk.END, " ✓\n")
                self.results_text.insert(tk.END, "\n✅ ТЕСТ ПРОЙДЕН УСПЕШНО! ✅\n")
            else:
                self.results_text.insert(tk.END, " ❌\n")
                self.results_text.insert(tk.END, "\n❌ ТЕСТ НЕ ПРОЙДЕН! ❌\n")
            
            # Выключаем нагрузку в конце теста
            #self.results_text.insert(tk.END, "\nВыключение нагрузки...\n")
            self.write_register(self.device_addr, REGISTERS["CTRL"], 4)  # Команда 4 - выключение нагрузки
            self.current_load_mode = "off"
            time.sleep(0.5)
            
            # Отправляем команду 9 для обновления значений после выключения нагрузки
            self.send_ctrl_command(9)
            # self.results_text.insert(tk.END, "Нагрузка выключена\n")
            
            # Показываем диалог сохранения файла
            try:
                # Получаем ID устройства из поля ввода
                device_id = self.device_id_var.get().strip()
                if not device_id:
                    device_id = "NO_ID"
                
                # Получаем текущую дату и время
                current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
                
                # Определяем выбранный тип БИЗ
                selected_biz = self.biz_type_var.get()
                biz_num = ""
                if selected_biz == "БИЗ 4":
                    biz_num = "4"
                elif selected_biz == "БИЗ 5":
                    biz_num = "5"
                elif selected_biz == "БИЗ 6":
                    biz_num = "6"
                
                # Предлагаем имя файла по умолчанию
                default_filename = f"FullTest_Biz_{biz_num}_{current_time}_{device_id}.txt"
                
                # Показываем диалог сохранения файла
                from tkinter import filedialog
                filepath = filedialog.asksaveasfilename(
                    defaultextension=".txt",
                    filetypes=[("Текстовые файлы", "*.txt"), ("Все файлы", "*.*")],
                    initialfile=default_filename,
                    title="Сохранить результаты теста"
                )
                
                # Если пользователь не отменил диалог
                if filepath:
                    # Получаем содержимое текстового поля с результатами
                    test_results = self.results_text.get(1.0, tk.END)
                    
                    # Добавляем заголовок с информацией о тесте
                    result_header = f"""
====================================
Файл результатов тестирования
Дата и время: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Тип БИЗ: {selected_biz}
ID устройства: {device_id}
====================================

"""
                    # Сохраняем в файл
                    with open(filepath, 'w', encoding='utf-8') as f:
                        f.write(result_header + test_results)
                    
                    self.results_text.insert(tk.END, f"\nРезультаты теста сохранены в файл:\n{filepath}\n")
                else:
                    self.results_text.insert(tk.END, "\nСохранение отменено пользователем\n")
                    
            except Exception as e:
                self.results_text.insert(tk.END, f"\nОшибка при сохранении результатов: {str(e)}\n")
            
            self.results_text.see(tk.END)  # Прокрутка к последней строке
            
        except Exception as e:
            self.results_text.insert(tk.END, f"\nОшибка во время выполнения теста: {str(e)}\n")
            self.results_text.see(tk.END)
        
    def poll_device(self):
        """Read current values from the tester and update the UI."""
        if not hasattr(self, 'ser') or self.ser is None or not self.ser.is_open:
            return
            
        try:
            # Read all registers we need in one go for better performance
            # Registers to read: 0x05 (TST_SENT_COUNT), 0x06 (TST_ERROR_COUNT), 0x08-0x0F (current values)
            start_reg = 0x05
            reg_count = 0x0F - 0x05 + 1  # From 0x05 to 0x0F inclusive
            cmd = build_read_command(DEVICE_ADDRESS, start_reg, reg_count)
            
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(cmd)
            
            # Wait for response (5 bytes header + 2 bytes per register)
            response = self.ser.read(5 + 2 * reg_count)
            
            if len(response) == 5 + 2 * reg_count:
                values = parse_response(response, reg_count)
                if values:
                    # Update request and error counters
                    self.request_count = values[0x05 - start_reg]  # TST_SENT_COUNT
                    self.error_count = values[0x06 - start_reg]    # TST_ERROR_COUNT
                    self.req_count_lbl.config(text=str(self.request_count))
                    self.err_count_lbl.config(text=str(self.error_count))
                    
                    # Update tester values (registers 0x08-0x0F)
                    current_values = {}
                    for i, (name, addr) in enumerate(self.display_params):
                        if 0x08 <= addr <= 0x0F:
                            val = values[addr - start_reg]
                            current_values[addr] = val
                            
                            # Format value based on parameter type
                            if name.startswith('U_'):
                                # Voltage: value / 100 with V unit
                                display_val = f"{val / 100:.2f} V"
                            elif name.startswith('I_'):
                                # Current: value / 1000 with mA unit
                                display_val = f"{val / 10:.1f} mA"
                            else:
                                display_val = str(val)
                            
                            if addr in self.tester_labels:
                                self.tester_labels[addr].config(text=display_val)
                    
                    # Update BIZ values
                    for addr, (label, reg_addr) in self.biz_labels.items():
                        if 0x08 <= reg_addr <= 0x0F:
                            val = values[reg_addr - start_reg]
                            # Format as voltage (V)
                            display_val = f"{val / 100:.2f} V"
                            label.config(text=display_val)
                    
                    # Update tolerance display with current values
                    self.update_tolerance_display(check_current_values=True, current_values=current_values)
            
        except Exception as e:
            self.error_count += 1
            self.err_count_lbl.config(text=str(self.error_count))
            print(f"Error polling device: {e}")
        
        # Schedule next poll in 500ms (0.5 seconds)
        self.after_id = self.after(500, self.poll_device)
    
    def start_polling(self):
        """Start the polling loop to read values from the tester."""
        # Cancel any existing polling
        if hasattr(self, 'after_id') and self.after_id:
            self.after_cancel(self.after_id)
        # Start polling
        self.poll_device()
    
    def on_closing(self):
        """Обработчик закрытия окна приложения"""
        # Stop polling
        if hasattr(self, 'after_id') and self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None
            
        try:
            # Проверяем, что соединение установлено
            if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
                # Записываем значение 1 в регистр TST_TYPE
                self.write_register(DEVICE_ADDRESS, REGISTERS["TST_TYPE"], 1)
                
                # Закрываем соединение
                self.ser.close()
        except Exception as e:
            print(f"Ошибка при закрытии приложения: {str(e)}")
        finally:
            # Закрываем приложение
            self.destroy()

if __name__ == "__main__":
    app = FullTestApp()
    app.mainloop()
