import serial
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports

# Modbus register addresses (updated with full list)

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

REGISTERS = {
    # Control
    "TESTER_ADDR": 0x01,
    "BIZ_TYPE": 0x02,
    "TST_TYPE": 0x03,
    "CTRL": 0x04,

    # Current values
    "I_IN": 0x09,
    "I_OUT": 0x08,
    "U_TP": 0x0B,
    "U_C": 0x0C,
    "U_IN": 0x0D,
    "U_OUT": 0x0E,

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
    "U_OUT_OFF_MIN_BIZ4": 0x20,
    "U_OUT_OFF_MAX_BIZ4": 0x21,
    "dU_OUT_MIN_BIZ4": 0x22,
    "dU_OUT_MAX_BIZ4": 0x23,
    "U_TP_MIN_BIZ4": 0x24,
    "U_TP_MAX_BIZ4": 0x25,
    "I_OUT_SHORT_MIN_BIZ4": 0x26,
    "I_OUT_SHORT_MAX_BIZ4": 0x27,
    "I_OUT_FAIL_MIN_BIZ4": 0x28,
    "I_OUT_FAIL_MAX_BIZ4": 0x29,
    "I_IN_OFF_MIN_BIZ4": 0x2A,
    "I_IN_OFF_MAX_BIZ4": 0x2B,
    "I_IN_LOAD_MIN_BIZ4": 0x2C,
    "I_IN_LOAD_MAX_BIZ4": 0x2D,
    "I_IN_SHORT_MIN_BIZ4": 0x2E,
    "I_IN_SHORT_MAX_BIZ4": 0x2F,
    "I_IN_FAIL_MIN_BIZ4": 0x30,
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

# Device parameters
DEVICE_ADDRESS = 1  # Modbus slave address, adjust if needed

def modbus_crc(data):
    """Calculate Modbus RTU CRC16"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def to_signed_16bit(val):
    """Convert unsigned 16-bit integer to signed 16-bit integer."""
    if val >= 0x8000:
        return val - 0x10000
    return val

def to_unsigned_16bit(val):
    """Convert signed 16-bit integer to unsigned 16-bit integer."""
    if val < 0:
        return val + 0x10000
    return val

def build_read_command(device_addr, start_reg, reg_count):
    """Build Modbus RTU command to read holding registers (function code 3)"""
    cmd = bytearray()
    cmd.append(device_addr)
    cmd.append(3)  # Function code 3: Read Holding Registers
    cmd.append((start_reg >> 8) & 0xFF)  # Start register high byte
    cmd.append(start_reg & 0xFF)         # Start register low byte
    cmd.append((reg_count >> 8) & 0xFF)  # Number of registers high byte
    cmd.append(reg_count & 0xFF)         # Number of registers low byte
    crc = modbus_crc(cmd)
    cmd.append(crc & 0xFF)  # CRC low byte
    cmd.append((crc >> 8) & 0xFF)  # CRC high byte
    return cmd

def parse_response(response, reg_count):
    """Parse Modbus RTU response for read holding registers"""
    # Response format: addr(1) + func(1) + byte count(1) + data(2*reg_count) + CRC(2)
    if len(response) < 5 + 2 * reg_count:
        raise ValueError("Ответ слишком короткий")
    # Verify CRC
    crc_calc = modbus_crc(response[:-2])
    crc_recv = response[-2] + (response[-1] << 8)
    if crc_calc != crc_recv:
        raise ValueError("CRC проверка не прошла")
    # Verify function code and byte count
    if response[1] != 3:
        raise ValueError(f"Неправильный код функции: {response[1]}")
    byte_count = response[2]
    if byte_count != 2 * reg_count:
        raise ValueError(f"Неправильное количество байтов: {byte_count}")
    # Extract register values
    registers = []
    for i in range(reg_count):
        high = response[3 + 2 * i]
        low = response[4 + 2 * i]
        val = (high << 8) + low
        registers.append(val)
    return registers

class ModbusApp(tk.Tk):
    def ask_float_dialog(self, title, prompt, callback):
        """Красивое не-блокирующее окно для ввода float. callback(value) вызывается при OK, value=None при Cancel."""
        win = tk.Toplevel(self)
        win.title(title)
        win.geometry("360x200+600+400")
        win.resizable(False, False)
        win.transient(self)
        # Стилизация
        frm = ttk.Frame(win, padding=20)
        frm.pack(expand=True, fill="both")
        label = ttk.Label(frm, text=prompt, font=("Arial", 12), wraplength=320, anchor="center", justify="center")
        label.pack(pady=(0, 10))
        entry = ttk.Entry(frm, width=20, font=("Arial", 14))
        entry.pack(pady=(0, 10))
        entry.focus_set()
        btn_frame = ttk.Frame(frm)
        btn_frame.pack(pady=(0, 5))
        def on_ok():
            val = entry.get().replace(",", ".")
            try:
                fval = float(val)
            except Exception:
                messagebox.showerror("Ошибка", "Введите число с точкой или запятой", parent=win)
                return
            win.destroy()
            callback(fval)
        def on_cancel():
            win.destroy()
            callback(None)
        done_btn = ttk.Button(btn_frame, text="Готово", command=on_ok, width=12, style="Accent.TButton")
        done_btn.pack(side="left", padx=8)
        cancel_btn = ttk.Button(btn_frame, text="Отмена", command=on_cancel, width=10)
        cancel_btn.pack(side="left", padx=8)
        win.bind('<Return>', lambda e: on_ok())
        win.bind('<Escape>', lambda e: on_cancel())
        # Центрирование окна относительно родителя
        self.after(10, lambda: win.lift())

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
        self.title("BIZ TESTER SETTINGS")
        self.geometry("800x600+400+200")
        self.resizable(False, False)
        # Frame for parameter display
        self.param_frame = ttk.LabelFrame(self, text="Параметры", relief="groove", borderwidth=2)
        self.param_frame.grid(column=0, row=1, rowspan = 2, padx=10, pady=10, sticky="nw")

        # Parameters to display as rectangles
        self.display_params = [
            ("U_вход", 0x0D),
            ("U_конд", 0x0C),
            ("U_вых", 0x0E),
            ("U_кт", 0x0B),
            ("I_вход", 0x09),
            ("I_выход", 0x08),
        ]

        # Create labels for each parameter in a vertical column
        self.param_labels = {}
        for i, (name, addr) in enumerate(self.display_params):
            label_name = ttk.Label(self.param_frame, text=name, font=("Arial", 12, "bold"))
            label_name.grid(column=0, row=i, padx=(0, 5), pady=5, sticky="w")

            frame = ttk.Frame(self.param_frame, relief="ridge", borderwidth=2, width=100, height=40)
            frame.grid(column=1, row=i, padx=(0, 10), pady=5, sticky="nsew")
            frame.grid_propagate(False)

            label_value = ttk.Label(frame, text="N/A", font=("Courier New", 14), width=8, anchor="center")
            label_value.pack(expand=True)

            self.param_labels[addr] = label_value

        # Add buttons "off", "load", "short"
        self.off_button = ttk.Button(self.param_frame, text="выкл", command=lambda: (self.send_ctrl_command(4)))
        self.off_button.grid(column=0, row=len(self.display_params) + 1, padx=5, pady=5, sticky="ew", columnspan=2)

        self.load_button = ttk.Button(self.param_frame, text="нагрузка", command=lambda: self.send_ctrl_command(5))
        self.load_button.grid(column=0, row=len(self.display_params) + 2, padx=5, pady=5, sticky="ew", columnspan=2)

        self.short_button = ttk.Button(self.param_frame, text="короткое", command=lambda: self.send_ctrl_command(6))
        self.short_button.grid(column=0, row=len(self.display_params) + 3, padx=5, pady=5, sticky="ew", columnspan=2)

        self.short_button = ttk.Button(self.param_frame, text="отказ", command=lambda: self.send_ctrl_command(10))
        self.short_button.grid(column=0, row=len(self.display_params) + 4, padx=5, pady=5, sticky="ew", columnspan=2)

        # Кнопки Включить/Выключить БИЗ
        self.biz_on_button = ttk.Button(self.param_frame, text="Включить БИЗ", command=lambda: self.send_ctrl_command(14))
        self.biz_on_button.grid(column=0, row=len(self.display_params) + 5, padx=5, pady=(15, 3), sticky="ew", columnspan=2)
        self.biz_off_button = ttk.Button(self.param_frame, text="Выключить БИЗ", command=lambda: self.send_ctrl_command(15))
        self.biz_off_button.grid(column=0, row=len(self.display_params) + 6, padx=5, pady=3, sticky="ew", columnspan=2)

        # Control frame
        self.control_frame = ttk.LabelFrame(self, text="Управление", relief="groove", borderwidth=2)
        self.control_frame.grid(column=2, row=1, padx=10, pady=10, sticky="nw")  # Поместили под display_frame

        # Connection frame
        self.connection_frame = ttk.LabelFrame(self, text="Параметры соединения", relief="groove", borderwidth=2)
        self.connection_frame.grid(column=0, row=0, columnspan = 3, padx=10, pady=10, sticky="nw")  # Поместили справа от остальных фреймов

        # Serial port selection
        self.port_label = ttk.Label(self.connection_frame, text="COM Порт:")
        self.port_label.grid(column=0, row=0, padx=5, pady=5, sticky="w")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            self.connection_frame, textvariable=self.port_var, values=self.get_serial_ports(), state="readonly", width=10
        )
        self.port_combo.grid(column=1, row=0, padx=5, pady=5, sticky="w")
        if self.port_combo["values"]:
            self.port_combo.current(0)

        # Bind dropdown click event to refresh ports
        def on_dropdown_click(event):
            current_value = self.port_var.get()
            self.port_combo['values'] = self.get_serial_ports()
            if current_value in self.port_combo['values']:
                self.port_var.set(current_value)
            elif self.port_combo['values']:
                self.port_combo.current(0)

        self.port_combo.bind('<Button-1>', on_dropdown_click)

        # Baudrate selection
        self.baud_label = ttk.Label(self.connection_frame, text="Скорость:")
        self.baud_label.grid(column=2, row=0, padx=5, pady=5, sticky="w")

        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(
            self.connection_frame,
            textvariable=self.baud_var,
            values=["9600", "19200", "38400", "57600", "115200"],
            state="readonly",
            width=8
        )
        self.baud_combo.grid(column=3, row=0, padx=5, pady=5, sticky="w")
        self.baud_combo.set("115200")

        self.set_connect = ttk.Button(self.connection_frame, text="Подключить", command=self.handle_connect)
        self.set_connect.grid(column=4, row=0, padx=5, pady=5, sticky="we")

        self.calibration_button = ttk.Button(self, text="Допуски", command=self.open_tolerances)
        self.calibration_button.place(x=700, y=20)
        # Окно для адреса тестера (Entry + Button)
        self.tester_addr_var = tk.StringVar(value="N/A")
        self.tester_addr_entry = ttk.Entry(self, textvariable=self.tester_addr_var, width=10)
        self.tester_addr_entry.place(x=500, y=20)
        self.set_addr_button = ttk.Button(self, text="Записать адрес", command=self.set_tester_addr)
        self.set_addr_button.place(x=500, y=50)

        # Для постоянного опроса
        self.polling = False
        self.polling_job = None
        self.ser = None

        self.calibration_params = [
            ("I_вых_A", REGISTERS["I_OUT_CAL_A"]),
            ("I_вых_B", REGISTERS["I_OUT_CAL_B"]),
            ("I_вход_A", REGISTERS["I_IN_CAL_A"]),
            ("I_вход_B", REGISTERS["I_IN_CAL_B"]),
            ("U_кт_A", REGISTERS["U_TP_CAL_A"]),
            ("U_кт_B", REGISTERS["U_TP_CAL_B"]),
            ("U_конд_A", REGISTERS["U_C_CAL_A"]),
            ("U_конд_B", REGISTERS["U_C_CAL_B"]),
            ("U_вход_A", REGISTERS["U_IN_CAL_A"]),
            ("U_вход_B", REGISTERS["U_IN_CAL_B"]),
            ("U_вых_A", REGISTERS["U_OUT_CAL_A"]),
            ("U_вых_B", REGISTERS["U_OUT_CAL_B"]),
        ]

        self.calibration_frame = ttk.LabelFrame(self, text="Калибровка", relief="groove", borderwidth=2)
        self.calibration_frame.place(x=210, y=85)

        self.calib_param_var = tk.StringVar()
        self.calib_param_combo = ttk.Combobox(
            self.calibration_frame,
            textvariable=self.calib_param_var,
            values=["U_вход", "U_конд", "U_вых", "U_кт", "I_вход", "I_выход"],
            state="readonly",
            width=15,
        )
        self.calib_param_combo.grid(column=2, row=0, padx=5, pady=(0,10), sticky="ew")
        self.calib_param_combo.current(0)

        self.calibrate_button = ttk.Button(self.calibration_frame, text="Откалибровать", command=self.calibrate_selected_param)
        self.calibrate_button.grid(column=2, row=1, padx=5, pady=(0,10), sticky="ew")

        self.entries = {}
        for i, (name, addr) in enumerate(self.calibration_params):
            label = ttk.Label(self.calibration_frame, text=name, font=("Arial", 12))
            label.grid(column=0, row=i, padx=10, pady=5, sticky="w")
            entry = ttk.Entry(self.calibration_frame, width=20, font=("Arial", 12))
            entry.grid(column=1, row=i, padx=10, pady=5, sticky="w")
            self.entries[addr] = entry

        self.get_button = ttk.Button(self.calibration_frame, text="Считать таблицы", command=self.get_calibration_values)
        self.get_button.grid(column=0, row=len(self.calibration_params) + 2, padx=10, pady=10, sticky="w")

        self.set_button = ttk.Button(self.calibration_frame, text="Записать", command=self.set_calibration_values)
        self.set_button.grid(column=1, row=len(self.calibration_params) + 2, padx=10, pady=10, sticky="w")

        # Теперь, когда все виджеты созданы, формируем список для блокировки
        self.widgets_to_lock = [
            self.param_frame, self.control_frame, self.calibration_frame,
            self.off_button, self.load_button, self.short_button,
            self.calibration_button, self.tester_addr_entry, self.set_addr_button
        ]
        self.set_widgets_state("disabled")

    def set_tester_addr(self):
        """Записать введённый адрес в регистр TESTER_ADDR"""
        if not self.ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        try:
            value = int(self.tester_addr_var.get())
            if not (0 <= value <= 255):
                raise ValueError("Адрес должен быть от 0 до 255")
            self.write_register(DEVICE_ADDRESS, REGISTERS["TESTER_ADDR"], value)

            self.send_ctrl_command(7)
            messagebox.showinfo("Успех", f"Адрес {value} записан в TESTER_ADDR")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка записи адреса: {e}")

    def calibrate_selected_param(self):
        import json
        param = self.calib_param_var.get()
        param_map = {
            "U_вход": ("U_IN_CAL_A", "U_IN_CAL_B", "U_IN"),
            "U_конд": ("U_C_CAL_A", "U_C_CAL_B", "U_C"),
            "U_вых": ("U_OUT_CAL_A", "U_OUT_CAL_B", "U_OUT"),
            "U_кт": ("U_TP_CAL_A", "U_TP_CAL_B", "U_TP"),
            "I_вход": ("I_IN_CAL_A", "I_IN_CAL_B", "I_IN"),
            "I_выход": ("I_OUT_CAL_A", "I_OUT_CAL_B", "I_OUT"),
        }
        if param not in param_map:
            messagebox.showerror("Ошибка", "Неизвестный параметр для калибровки!")
            return
        reg_A, reg_B, reg_raw = param_map[param]
        from tkinter import simpledialog
        ser = self.ser
        if not ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        # Hardcoded calibration prompts
        cal_text = {
            "I_IN_1": "Введите фактический ток на входе (первое калибровочное)\n в миллиамперах",
            "I_IN_2": "Введите фактический ток на входе (второе калибровочное)\n в миллиамперах",
            "I_OUT_1": "Введите фактический ток на выходе (первое калибровочное)\n в миллиамперах",
            "I_OUT_2": "Введите фактический ток на выходе (второе калибровочное)\n в миллиамперах",
            "U_TP_1": "Введите фактическое напряжение на контрольной точке (первое калибровочное)\nв Вольтах",
            "U_TP_2": "Введите фактическое напряжение на контрольной точке (второе калибровочное)\nв Вольтах",
            "U_C_1": "Введите фактическое напряжение на конденсаторах (первое калибровочное)\nв Вольтах",
            "U_C_2": "Введите фактическое напряжение на конденсаторах (второе калибровочное)\nв Вольтах",
            "U_IN_1": "Введите фактическое напряжение на входе (первое калибровочное)\nв Вольтах",
            "U_IN_2": "Введите фактическое напряжение на входе (второе калибровочное)\nв Вольтах",
            "U_OUT_1": "Введите фактическое напряжение на выходе (первое калибровочное)\nв Вольтах",
            "U_OUT_2": "Введите фактическое напряжение на выходе (второе калибровочное)\nв Вольтах"
        }
        key1 = reg_raw + "_1"
        key2 = reg_raw + "_2"
        prompt1 = cal_text.get(key1, f"Введите эталонное значение для {param} (первое измерение)")
        def after_y1(Y1):
            if Y1 is None:
                messagebox.showinfo("Калибровка", "Калибровка отменена пользователем.")
                return
            if param.startswith("U_"):
                Y1v = int(round(Y1 * 100))
            elif param.startswith("I_"):
                if Y1 == int(Y1):
                    Y1v = int(Y1 * 10)
                else:
                    Y1v = int(round(Y1 * 10))
            else:
                Y1v = int(Y1)
            self.send_ctrl_command_via_serial(ser, 8)
            X1 = self.read_raw_value(ser, DEVICE_ADDRESS, REGISTERS[reg_raw])
            if X1 is None:
                return
            prompt2 = cal_text.get(key2, f"Введите эталонное значение для {param} (второе измерение)")
            def after_y2(Y2):
                if Y2 is None:
                    messagebox.showinfo("Калибровка", "Калибровка отменена пользователем.")
                    return
                if param.startswith("U_"):
                    Y2v = int(round(Y2 * 100))
                elif param.startswith("I_"):
                    if Y2 == int(Y2):
                        Y2v = int(Y2 * 10)
                    else:
                        Y2v = int(round(Y2 * 10))
                else:
                    Y2v = int(Y2)
                self.send_ctrl_command_via_serial(ser, 8)
                X2 = self.read_raw_value(ser, DEVICE_ADDRESS, REGISTERS[reg_raw])
                if X2 is None:
                    return
                if X2 == X1:
                    messagebox.showerror("Ошибка", f"Сырые значения одинаковы (X1 = X2 = {X1}), калибровка невозможна.\nПроверьте, что вы подаете разные значения!")
                    return
                try:
                    A = (1000 * (Y2v - Y1v) / (X2 - X1))
                    B = ((Y1v * 1000 / (A)) - X1)
                except Exception as e:
                    messagebox.showerror("Ошибка", f"Ошибка вычисления коэффициентов: {e}")
                    return
                debug_info = (
                    f"Y1 (эталон 1): {Y1v}\n"
                    f"Y2 (эталон 2): {Y2v}\n"
                    f"X1 (ADC 1): {X1}\n"
                    f"X2 (ADC 2): {X2}\n"
                    f"A: {A}\n"
                    f"B: {B}\n"
                )
                messagebox.showinfo("Отладка калибровки", debug_info)
                try:
                    self.write_register(DEVICE_ADDRESS, REGISTERS[reg_A], int(A))
                    self.write_register(DEVICE_ADDRESS, REGISTERS[reg_B], int(B))
                except Exception as e:
                    messagebox.showerror("Ошибка", f"Ошибка записи коэффициентов: {e}")
                    return
                self.send_ctrl_command(7)
                messagebox.showinfo("Калибровка завершена", f"A={A}, B={B}\nКоэффициенты записаны в регистры.")
            self.ask_float_dialog("Калибровка: второе значение", prompt2, after_y2)
        self.ask_float_dialog("Калибровка: первое значение", prompt1, after_y1)

    def send_ctrl_command_via_serial(self, ser, command):
        try:
            self.write_register(DEVICE_ADDRESS, REGISTERS["CTRL"], command)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка отправки управляющей команды: {e}")

    def read_raw_value(self, ser, device_addr, register_addr):
        try:
            cmd = build_read_command(device_addr, register_addr, 1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(cmd)
            time.sleep(0.1)
            response = ser.read(7)
            if not response or len(response) < 7:
                messagebox.showerror("Ошибка", "Нет ответа от устройства при чтении сырого значения")
                return None
            values = parse_response(response, 1)
            return values[0]
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка чтения сырого значения: {e}")
            return None

    def is_param_A(self, param_name):
        return param_name.endswith('_A')

    def get_calibration_values(self):
        ser = self.ser
        if not ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        start_reg = REGISTERS["I_OUT_CAL_A"]
        end_reg = REGISTERS["U_OUT_CAL_B"]
        total_regs = end_reg - start_reg + 1
        cmd = build_read_command(DEVICE_ADDRESS, start_reg, total_regs)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(cmd)
            time.sleep(0.1)
            response = ser.read(5 + 2 * total_regs)
            if not response:
                messagebox.showerror("Ошибка", "Ответ не получен")
                return
            values = parse_response(response, total_regs)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка чтения калибровочных регистров: {e}")
            return
        for name, addr in self.calibration_params:
            reg_idx = addr - start_reg
            value = values[reg_idx]
            signed_value = to_signed_16bit(value)
            if self.is_param_A(name):
                display_value = f"{signed_value / 1000:.3f}"
            else:
                display_value = str(signed_value)
            self.entries[addr].delete(0, tk.END)
            self.entries[addr].insert(0, display_value)

    def set_calibration_values(self):
        ser = self.ser
        if not ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        for name, addr in self.calibration_params:
            try:
                str_value = self.entries[addr].get().strip()
                if self.is_param_A(name):
                    try:
                        float_value = float(str_value)
                        value = int(round(float_value * 1000))
                    except ValueError:
                        messagebox.showerror("Ошибка", f"Неверный формат числа для {name}. Используйте точку как разделитель (например: 0.736)")
                        return
                else:
                    try:
                        value = int(str_value)
                    except ValueError:
                        messagebox.showerror("Ошибка", f"Неверное значение для {name}. Введите целое число")
                        return
                unsigned_value = to_unsigned_16bit(value)
                self.write_register(DEVICE_ADDRESS, addr, unsigned_value)
            except Exception as e:
                messagebox.showerror("Ошибка", f"Ошибка записи {name}: {e}")
                return
        messagebox.showinfo("Успех", "Калибровочные значения успешно записаны")
        self.send_ctrl_command(7)

    def open_tolerances(self):
        TolerancesApp(self)

    def send_ctrl_command(self, command):
        if not self.ser or not self.polling:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        try:
            self.write_register(DEVICE_ADDRESS, REGISTERS["CTRL"], command)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка записи регистра CTRL: {e}")
            return
        # После команды сразу обновить значения
        try:
            self.read_and_update(self.ser)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка чтения регистров: {e}")

    def write_register(self, device_addr, register_addr, value):
        """Write single register (function code 6) через self.ser"""
        if not self.ser:
            raise ValueError("Нет активного соединения с устройством")
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

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.write(cmd)
        time.sleep(0.1)

        response = self.ser.read(8)
        if len(response) != 8:
            raise ValueError("Неверная длина ответа для записи регистра")
        # Verify response matches request
        if response[:6] != cmd[:6]:
            raise ValueError("Ответ не соответствует запросу")

    def read_and_update(self, ser):
        start_reg = 0x02
        end_reg = 0x55
        total_regs = end_reg - start_reg + 1

        cmd = build_read_command(DEVICE_ADDRESS, start_reg, total_regs)

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(cmd)
        time.sleep(0.1)

        response = ser.read(5 + 2 * total_regs)

        if not response:
            raise ValueError("Ответ не получен")

        regs = parse_response(response, total_regs)

        # Update parameter labels with values
        for name, addr in self.display_params:
            val = regs[addr - start_reg]
            # Format value based on parameter type
            if name.startswith("U_"):
                # Voltage: value / 100 with V unit
                display_val = f"{val / 100:.2f} V"
            elif name.startswith("I_"):
                # Current: value / 10 with mA unit
                display_val = f"{val / 10:.1f} mA"
            else:
                display_val = str(val)
            self.param_labels[addr].config(text=display_val)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def handle_connect(self):
        # Если уже подключено — отключаем
        if self.polling:
            self.handle_disconnect()
            return

        port = self.port_var.get()
        baudrate = int(self.baud_var.get())
        if not port:
            messagebox.showerror("Ошибка", "Пожалуйста, выберите COM порт")
            return
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        except serial.SerialException as e:
            messagebox.showerror("Ошибка", f"Ошибка открытия COM порта {port}: {e}")
            self.ser = None
            return
        # При подключении: записать в TST_TYPE значение 4
        try:
            self.write_register(DEVICE_ADDRESS, REGISTERS["TST_TYPE"], 4)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка записи регистра TST_TYPE: {e}")
        # Прочитать TESTER_ADDR и отобразить в tester_addr_entry
        try:
            cmd = build_read_command(DEVICE_ADDRESS, REGISTERS["TESTER_ADDR"], 1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(cmd)
            time.sleep(0.1)
            response = self.ser.read(7)
            if not response or len(response) < 7:
                self.tester_addr_var.set("Ошибка")
            else:
                values = parse_response(response, 1)
                self.tester_addr_var.set(str(values[0]))
        except Exception as e:
            self.tester_addr_var.set("Ошибка")
            print(f"Ошибка чтения TESTER_ADDR: {e}")
        # Если успешно, активируем интерфейс
        self.set_widgets_state("normal")
        self.set_connect.config(text="Отключить", command=self.handle_disconnect)
        self.polling = True
        self.poll_device()


    def handle_disconnect(self):
        # При отключении: записать в TST_TYPE значение 1
        if self.ser:
            try:
                self.write_register(DEVICE_ADDRESS, REGISTERS["TST_TYPE"], 1)
            except Exception:
                pass
        self.polling = False
        if self.polling_job:
            self.after_cancel(self.polling_job)
            self.polling_job = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.set_widgets_state("disabled")
        self.set_connect.config(text="Подключить", command=self.handle_connect)

    def on_closing(self):
        # Вызываем handle_disconnect для очистки ресурсов
        self.handle_disconnect()
        # Уничтожаем окно
        self.destroy()

    def poll_device(self):
        if not self.polling or not self.ser:
            return
        try:
            self.read_and_update(self.ser)
        except Exception as e:
            # Если ошибка — отключаемся
            messagebox.showerror("Ошибка", f"Ошибка при чтении: {e}")
            self.handle_disconnect()
            return
        # Запланировать следующий опрос через 1 секунду
        self.polling_job = self.after(500, self.poll_device)

class TolerancesApp(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Допуски")
        self.geometry("1250x580")
        self.parent = parent

        # Separate tolerance registers by BIZ4, BIZ5, BIZ6
        self.tolerance_biz4 = [(name, addr) for name, addr in REGISTERS.items() if "BIZ4" in name]
        self.tolerance_biz5 = [(name, addr) for name, addr in REGISTERS.items() if "BIZ5" in name]
        self.tolerance_biz6 = [(name, addr) for name, addr in REGISTERS.items() if "BIZ6" in name]

        self.entries = {}

        # Create frames for each BIZ group
        frame_biz4 = ttk.LabelFrame(self, text="БИЗ 4", relief="groove", borderwidth=2)
        frame_biz4.grid(column=0, row=0, rowspan=4, padx=10, pady=10, sticky="nw")

        frame_biz5 = ttk.LabelFrame(self, text="БИЗ 5", relief="groove", borderwidth=2)
        frame_biz5.grid(column=1, row=0, rowspan=4, padx=10, pady=10, sticky="nw")

        frame_biz6 = ttk.LabelFrame(self, text="БИЗ 6", relief="groove", borderwidth=2)
        frame_biz6.grid(column=2, row=0, rowspan=4, padx=10, pady=10, sticky="nw")

        # Mapping of tolerance names to Russian with units
        tolerance_name_map = {
            "U_OUT_OFF_MIN": "U_вых_мин, В",
            "U_OUT_OFF_MAX": "U_вых_макс, В",
            "dU_OUT_MIN": "ΔU_вых_мин, В",
            "dU_OUT_MAX": "ΔU_вых_макс, В",
            "U_TP_MIN": "U_кт_мин, В",
            "U_TP_MAX": "U_кт_макс, В",
            "I_OUT_SHORT_MIN": "I_короткое_мин, mA",
            "I_OUT_SHORT_MAX": "I_короткое_макс, mA",
            "I_OUT_FAIL_MIN": "I_отказ_мин, mA",
            "I_OUT_FAIL_MAX": "I_отказ_макс, mA",
            "I_IN_OFF_MIN": "I_вход_мин, mA",
            "I_IN_OFF_MAX": "I_вход_макс, mA",
            "I_IN_LOAD_MIN": "I_нагрузка_мин, mA",
            "I_IN_LOAD_MAX": "I_нагрузка_макс, mA",
            "I_IN_SHORT_MIN": "I_короткое_мин, mA",
            "I_IN_SHORT_MAX": "I_короткое_макс, mA",
            "I_IN_FAIL_MIN": "I_отказ_мин, mA",
            "I_IN_FAIL_MAX": "I_отказ_макс, mA"
        }

        # Function to get translated name
        def get_translated_name(name):
            base_name = name.split('_BIZ')[0]
            return tolerance_name_map.get(base_name, name)

        # Populate BIZ4 entries
        for i, (name, addr) in enumerate(self.tolerance_biz4):
            translated_name = get_translated_name(name)
            label = ttk.Label(frame_biz4, text=translated_name, font=("Arial", 10))
            label.grid(column=0, row=i, padx=5, pady=3, sticky="w")

            entry = ttk.Entry(frame_biz4, width=20, font=("Arial", 10))
            entry.grid(column=1, row=i, padx=5, pady=3, sticky="w")
            self.entries[addr] = entry

        # Populate BIZ5 entries
        for i, (name, addr) in enumerate(self.tolerance_biz5):
            translated_name = get_translated_name(name)
            label = ttk.Label(frame_biz5, text=translated_name, font=("Arial", 10))
            label.grid(column=0, row=i, padx=5, pady=3, sticky="w")

            entry = ttk.Entry(frame_biz5, width=20, font=("Arial", 10))
            entry.grid(column=1, row=i, padx=5, pady=3, sticky="w")
            self.entries[addr] = entry

        # Populate BIZ6 entries
        for i, (name, addr) in enumerate(self.tolerance_biz6):
            translated_name = get_translated_name(name)
            label = ttk.Label(frame_biz6, text=translated_name, font=("Arial", 10))
            label.grid(column=0, row=i, padx=5, pady=3, sticky="w")

            entry = ttk.Entry(frame_biz6, width=20, font=("Arial", 10))
            entry.grid(column=1, row=i, padx=5, pady=3, sticky="w")
            self.entries[addr] = entry

        self.tolerances_controrl_frame = ttk.LabelFrame(self, text="Управление", relief="groove", borderwidth=2)
        self.tolerances_controrl_frame.grid(column=3, row=0, padx=10, pady=10, sticky="nw")

        # Add "Get" and "Set" buttons at the bottom spanning all columns
        self.get_button = ttk.Button(self.tolerances_controrl_frame, text="Считать", command=self.get_tolerance_values)
        self.get_button.grid(column=0, row=0, padx=10, pady=10, sticky="ew")

        self.set_button = ttk.Button(self.tolerances_controrl_frame, text="Записать", command=self.set_tolerance_values, state=tk.DISABLED)
        self.set_button.grid(column=0, row=1, padx=10, pady=10, sticky="ew")

        # Add "Save to File" and "Load from File" buttons
        self.save_button = ttk.Button(self.tolerances_controrl_frame, text="Сохранить в файл", command=self.save_to_file, state=tk.DISABLED)
        self.save_button.grid(column=0, row=2, padx=10, pady=10, sticky="ew")

        self.load_button = ttk.Button(self.tolerances_controrl_frame, text="Загрузить из файла", command=self.load_from_file)
        self.load_button.grid(column=0, row=3, padx=10, pady=10, sticky="ew")

        # Track if values have been loaded or modified
        self.values_loaded = False

        # Bind entry changes to enable save/write buttons
        all_params = self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6
        for _, addr in all_params:
            self.entries[addr].bind("<KeyRelease>", self.on_entry_change)
            self.entries[addr].bind("<ButtonRelease>", self.on_entry_change)

    def on_entry_change(self, event=None):
        """Enable save/write buttons when any entry is modified"""
        if not self.values_loaded:
            self.values_loaded = True
            self.set_button.config(state=tk.NORMAL)
            self.save_button.config(state=tk.NORMAL)

    def get_tolerance_values(self):
        """Retrieve tolerance values from device and display them with decimal points, используя уже открытый порт."""
        if not self.parent.ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        self.values_loaded = True
        self.set_button.config(state=tk.NORMAL)
        self.save_button.config(state=tk.NORMAL)
        ser = self.parent.ser
        all_params = self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6
        if not all_params:
            return
        start_reg = min(addr for _, addr in all_params)
        end_reg = max(addr for _, addr in all_params)
        total_regs = end_reg - start_reg + 1
        cmd = build_read_command(DEVICE_ADDRESS, start_reg, total_regs)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(cmd)
            time.sleep(0.1)
            response = ser.read(5 + 2 * total_regs)
            if not response:
                messagebox.showerror("Ошибка", "Ответ не получен")
                return
            values = parse_response(response, total_regs)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка чтения допусков: {e}")
            return
        for name, addr in all_params:
            if addr - start_reg < len(values):
                raw_value = values[addr - start_reg]
                if name.startswith("U_") or name.startswith("dU_"):
                    formatted_value_str = f"{raw_value / 100.0:.2f}"
                elif name.startswith("I_"):
                    formatted_value_str = f"{raw_value / 10.0:.1f}"
                else:
                    formatted_value_str = str(raw_value)
                self.entries[addr].delete(0, tk.END)
                self.entries[addr].insert(0, formatted_value_str)
            else:
                print(f"Предупреждение: индекс за пределами границ для регистра {name} (адрес {addr})")

    def validate_positive_numbers(self):
        """Validate that all entries contain positive numbers (float or int) or are empty."""
        all_params = self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6
        for name, addr in all_params:
            value_str = self.entries[addr].get().strip()
            if not value_str: # Empty string is considered valid (means no change or not set)
                continue
            try:
                value_float = float(value_str)
                if value_float <= 0:
                    messagebox.showerror("Ошибка ввода", f"Значение для '{name}' должно быть положительным числом (например, 10.5 или 10). Введено: '{value_str}'")
                    return False
            except ValueError:
                messagebox.showerror("Ошибка ввода", f"Значение для '{name}' не является допустимым числом (например, 10.5 или 10). Введено: '{value_str}'")
                return False
        return True

    def set_tolerance_values(self):
        """Parse decimal inputs, convert to int, and send tolerance values to device через уже открытый порт."""
        if not self.validate_positive_numbers():
            return
        if not self.parent.ser:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
        ser = self.parent.ser
        all_params = self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6
        success_count = 0
        error_occurred = False
        for name, addr in all_params:
            value_str = self.entries[addr].get().strip()
            if not value_str:
                continue
            try:
                value_float = float(value_str)
                if name.startswith("U_") or name.startswith("dU_"):
                    final_value_int = int(round(value_float * 100.0))
                elif name.startswith("I_"):
                    final_value_int = int(round(value_float * 10.0))
                else:
                    messagebox.showerror("Ошибка конфигурации", f"Неизвестный тип параметра для {name}. Проверьте REGISTERS.")
                    error_occurred = True
                    break
                self.write_register(ser, DEVICE_ADDRESS, addr, final_value_int)
                success_count += 1
            except ValueError:
                messagebox.showerror("Ошибка ввода", f"Недопустимое значение '{value_str}' для {name}. Пожалуйста, введите число (например, 10.5).")
                error_occurred = True
                break
            except Exception as e:
                messagebox.showerror("Ошибка записи", f"Не удалось записать значение для {name}: {e}")
                error_occurred = True
                break
        if not error_occurred and success_count > 0:
            messagebox.showinfo("Успех", f"Значения допусков ({success_count}) успешно обновлены.")
            self.parent.send_ctrl_command(7)
        elif not error_occurred and success_count == 0 and any(self.entries[addr].get().strip() for _, addr in all_params):
            messagebox.showinfo("Информация", "Нет значений для записи или все поля пустые.")

    def write_register(self, ser, device_addr, register_addr, value):
        """Write single holding register (function code 6) через уже открытый порт."""
        cmd = bytearray()
        cmd.append(device_addr)
        cmd.append(6)
        cmd.append((register_addr >> 8) & 0xFF)
        cmd.append(register_addr & 0xFF)
        cmd.append((value >> 8) & 0xFF)
        cmd.append(value & 0xFF)
        crc = modbus_crc(cmd)
        cmd.append(crc & 0xFF)
        cmd.append((crc >> 8) & 0xFF)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(cmd)
        time.sleep(0.1)
        response = ser.read(8)
        if len(response) != 8:
            raise ValueError("Неверная длина ответа для записи регистра")
        if response[:6] != cmd[:6]:
            raise ValueError("Ответ не соответствует запросу")

    def save_to_file(self):
        """Save tolerance values to a JSON file."""
        import json
        from tkinter import filedialog

        if not self.validate_positive_numbers():
            return

        # Prepare data dictionary with register names and values
        data = {}
        for name, addr in self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6:
            value = self.entries[addr].get()
            data[name] = value

        # Ask user for file path to save
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Сохранить допуски в файл"
        )
        if not file_path:
            return  # User cancelled

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
            messagebox.showinfo("Успех", f"Допуски успешно сохранены в файл {file_path}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка сохранения файла: {e}")

    def load_from_file(self):
        """Load tolerance values from a JSON file."""
        import json
        from tkinter import filedialog

        # Ask user for file path to load
        file_path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Загрузить допуски из файла"
        )
        if not file_path:
            return  # User cancelled

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            # Populate entries with loaded data
            for name, addr in self.tolerance_biz4 + self.tolerance_biz5 + self.tolerance_biz6:
                if name in data:
                    self.entries[addr].delete(0, tk.END)
                    self.entries[addr].insert(0, str(data[name]))

            # Enable buttons after successful load
            self.values_loaded = True
            self.set_button.config(state=tk.NORMAL)
            self.save_button.config(state=tk.NORMAL)

            messagebox.showinfo("Успех", "Данные успешно загружены из файла")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось загрузить файл: {e}")

    def write_register(self, ser, device_addr, register_addr, value):
        """Write single holding register (function code 6)."""
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

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(cmd)
        time.sleep(0.1)

        response = ser.read(8)
        if len(response) != 8:
            raise ValueError("Неверная длина ответа для записи регистра")
        if response[:6] != cmd[:6]:
            raise ValueError("Ответ не соответствует запросу")

if __name__ == "__main__":
    app = ModbusApp()
    # Bind window close event to on_closing
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
