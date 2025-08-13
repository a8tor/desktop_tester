import serial
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.ttk import Progressbar
import pandas as pd
from datetime import datetime
import os

# Global serial port
ser = None
from datetime import datetime
import serial.tools.list_ports

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
    "TEST_COUNT": 0x84,
    "ERROR_CODE_FULL": 0x07,
    "CURRENT_RESULT_COUNTERS": 0x85,
    "LONG_TEST_SHORT_PERIOD_ms": 0x1E, # Период длинного теста в миллисекундах
    "LONG_TEST_LONG_PERIOD": 0x1F, # Количество тестов для сохранения
    "LONG_TEST_START_TIME": 0x86 # Время старта последнего длительного теста
}

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


def read_test_count(addr):
    """
    Прочитать количество выполненных тестов (2 регистра, int32).
    Возвращает int.
    """
    global ser
    cmd = build_read_command(addr, REGISTERS["TEST_COUNT"], 2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(cmd)
    response = ser.read(9)
    if len(response) != 9:
        raise ValueError("Нет ответа от устройства")
    values = parse_response(response, 2)
    test_count = (values[0] << 16) | values[1]
    return test_count

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
    if len(response) < 5 + 2 * reg_count:
        raise ValueError("Ответ слишком короткий")
    crc_calc = modbus_crc(response[:-2])
    crc_recv = response[-2] + (response[-1] << 8)
    if crc_calc != crc_recv:
        raise ValueError("CRC проверка не прошла")
    if response[1] != 3:
        raise ValueError(f"Неправильный код функции: {response[1]}")
    byte_count = response[2]
    if byte_count != 2 * reg_count:
        raise ValueError(f"Неправильное количество байтов: {byte_count}")
    registers = []
    for i in range(reg_count):
        high = response[3 + 2 * i]
        low = response[4 + 2 * i]
        val = (high << 8) + low  # Big Endian: старший байт первым
        registers.append(val)
    return registers


class LongTestApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Long Test Modbus RS-485")
        self.geometry("1300x600+250+200")
        self.resizable(1, 1)
        
        # Connection state
        self.connected = False
        self.uptime_job = None  # Store the after job ID for the uptime timer

        # --- Frame: Параметры соединения ---
        self.connection_frame = ttk.LabelFrame(self, text="Параметры соединения", relief="groove", borderwidth=2)
        self.connection_frame.grid(column=0, row=0, padx=10, pady=10, sticky="nw")

        # --- Frame: Время наработки ---
        self.uptime_frame = ttk.LabelFrame(self, text="Время наработки", relief="groove", borderwidth=2)
        self.uptime_frame.place(x=430, y=10)
        self.uptime_var = tk.StringVar(value="0 дн. 00:00:00")
        self.uptime_label = ttk.Label(self.uptime_frame, textvariable=self.uptime_var, font=("Arial", 12, "bold"), foreground="#1a5f1a")
        self.uptime_label.pack(padx=12, pady=12)
        self.test_start_time = None
        
        # --- Frame: Настройки сохранения ---
        self.save_frame = ttk.LabelFrame(self, text="Сохранение результатов", relief="groove", borderwidth=2)
        self.save_frame.place(x=630, y=10, width=300)
        
        # Default save directory (user's documents folder)
        self.save_dir = os.path.join(os.path.expanduser("~"), "Documents")
        
        # --- Кнопки управления ---
        btn_frame = ttk.Frame(self)
        btn_frame.place(x=940, y=10, height=70)
        
        # Кнопка 'Считать все'
        self.read_all_btn = ttk.Button(
            btn_frame, 
            text="Считать все", 
            command=self._on_read_all_clicked,
            width=15
        )
        self.read_all_btn.pack(fill='x', pady=(0, 5))
        
        # Кнопка 'Синхр. время'
        self.sync_time_btn = ttk.Button(
            btn_frame,
            text="Синхр. время",
            command=self.sync_time,
            width=15
        )
        self.sync_time_btn.pack(fill='x')

        # Period input
        period_label = ttk.Label(self, text="Период в сек.:")
        period_label.place(x=1050, y=10)
        
        self.period_var = tk.IntVar(value=5)
        self.period_spin = ttk.Spinbox(
            self,
            from_=1,
            to=20,
            textvariable=self.period_var,
            width=5
        )
        self.period_spin.place(x=1140, y=10)
        
        # Count input
        count_label = ttk.Label(self, text="Количество:")
        count_label.place(x=1050, y=40)
        
        self.count_var = tk.IntVar(value=50)
        self.count_spin = ttk.Spinbox(
            self,
            from_=5,
            to=200,
            textvariable=self.count_var,
            width=5
        )
        self.count_spin.place(x=1140, y=40)
        
        # Write button
        self.write_btn = ttk.Button(
            self,
            text="Записать",
            command=self._on_write_clicked,
            width=15
        )
        self.write_btn.place(x=1195, y=25)
        
        # Directory display
        self.save_dir_var = tk.StringVar(value=f"Папка: {self.save_dir}")
        self.save_dir_label = ttk.Label(
            self.save_frame, 
            textvariable=self.save_dir_var, 
            wraplength=300,
            justify='left',
            font=("Arial", 9)
        )
        self.save_dir_label.pack(padx=12, pady=(5, 0), anchor='w')
        
        # Change directory button
        self.change_dir_btn = ttk.Button(
            self.save_frame, 
            text="Изменить папку", 
            command=self._change_save_directory,
            width=15
        )
        self.change_dir_btn.pack(pady=(5, 10), padx=12, anchor='center')

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
        self.port_combo.bind('<Button-1>', self.on_dropdown_click)

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

        # Scan button
        self.scan_button = ttk.Button(self.connection_frame, text="Подключить", command=self.handle_scan)
        self.scan_button.grid(column=4, row=0, padx=5, pady=5, sticky="we")

        # Фрейм и таблица для найденных тестеров
        self.testers_frame = ttk.LabelFrame(self, text="Тестеры на шине", relief="groove", borderwidth=2)
        self.testers_frame.grid(column=0, row=1, padx=10, pady=10, columnspan=2, sticky="nsw")
        # Список всех допусков (короткие имена)
        tolerance_names = [
            "U_OUT_OFF", "dU_OUT", "U_TP", "I_OUT_SHORT", "I_OUT_FAIL",
            "I_IN_OFF", "I_IN_LOAD", "I_IN_SHORT", "I_IN_FAIL", "UART_FAIL"
        ]
        columns = ["addr", "biz_type"] + tolerance_names
        self.testers_table = ttk.Treeview(self.testers_frame, columns=columns, show="headings", height=20)
        self.testers_table.heading("addr", text="Адрес")
        self.testers_table.heading("biz_type", text="BIZ_TYPE")
        self.testers_table.column("addr", width=70, anchor="center")
        self.testers_table.column("biz_type", width=70, anchor="center")
        for name in tolerance_names:
            self.testers_table.heading(name, text=name)
            self.testers_table.column(name, width=110, anchor="center")
        self.testers_table.pack(fill="both", expand=True)
        # Настраиваем стили строк
        self.testers_table.tag_configure('oddrow', background='#e0e0e0')
        self.testers_table.tag_configure('evenrow', background='#ffffff')
        self.testers_table.tag_configure('error', background='#ffcccc')  # Красный фон для строк с ошибками
        self.tolerance_names = tolerance_names
        
        # Переменные состояния
        self.polling = False
        self.selected_row = None
        
        # Создаем контекстное меню
        self.context_menu = tk.Menu(self, tearoff=0)
        self.context_menu.add_command(label="Считать", command=self._on_read_clicked, state='disabled')
        
        # Привязываем ПКМ к таблице
        self.testers_table.bind("<Button-3>", self._show_context_menu)

        # --- Кнопка Запустить/Остановить под таблицей ---
        self.polling_job = None
        self.start_stop_btn = ttk.Button(self, text="Запустить", width=18, command=self.toggle_polling, state="disabled")
        self.start_stop_btn.grid(row=2, column=0, columnspan=2, pady=(5, 15))
        
        # Обработчик закрытия окна
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _change_save_directory(self):
        """Open a directory selection dialog to choose where to save test results"""
        new_dir = filedialog.askdirectory(initialdir=self.save_dir)
        if new_dir:  # If user didn't cancel
            self.save_dir = new_dir
            self.save_dir_var.set(f"Папка: {self.save_dir}")
            # Create the directory if it doesn't exist
            os.makedirs(self.save_dir, exist_ok=True)
    
    def format_uptime(self, seconds):
        """Format seconds into D days, HH:MM:SS"""
        days = seconds // (24 * 3600)
        seconds = seconds % (24 * 3600)
        hours = seconds // 3600
        seconds %= 3600
        minutes = seconds // 60
        seconds %= 60
        return f"{int(days)} дн. {int(hours):02d}:{int(minutes):02d}:{int(seconds):02d}"

    def update_uptime(self):
        """Update the uptime display"""
        if self.test_start_time is not None:
            elapsed = int(time.time() - self.test_start_time)
            self.uptime_var.set(self.format_uptime(elapsed))
        # Only schedule next update if polling is active
        if self.polling:
            self.uptime_job = self.after(1000, self.update_uptime)

    def toggle_polling(self):
        if not self.polling:
            # Reset uptime counter when starting via the button
            self.test_start_time = time.time()
            self.uptime_var.set("0 дн. 00:00:00")
            self.start_polling()
        else:
            self.stop_polling()

    def start_polling(self):
        global ser
        
        if not self.connected or not ser or not ser.is_open:
            messagebox.showerror("Ошибка", "Нет активного соединения!")
            return
            
        try:
            # Send start command (16) to all testers
            if not self.send_ctrl_command(16):
                raise Exception("Не удалось отправить команду старта")
                
            self.polling = True
            # Only set test_start_time if it hasn't been set yet (for already running tests)
            if self.test_start_time is None:
                self.test_start_time = time.time()
            self.start_stop_btn.config(text="Остановить")
            
            # Start polling
            self.poll_testers()
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка запуска тестирования: {e}")
            self.stop_polling()
            
    def sync_time(self):
        """Синхронизировать время на всех подключенных тестерах"""
        global ser
        
        if not self.connected or not ser or not ser.is_open:
            messagebox.showerror("Ошибка", "Нет активного соединения!")
            return
            
        try:
            # Получаем список адресов подключенных тестеров
            addresses = self._get_all_tester_addresses()
            success_count = 0
            
            for addr in addresses:
                try:
                    if self.send_current_time(ser, addr):
                        success_count += 1
                        time.sleep(0.1)
                    else:
                        print(f"Не удалось синхронизировать время с тестером {addr}")
                except Exception as e:
                    print(f"Ошибка при синхронизации времени с тестером {addr}: {str(e)}")
            
            if success_count > 0:
                messagebox.showinfo("Успех", f"Время синхронизировано на {success_count} устройствах")
            else:
                messagebox.showwarning("Предупреждение", "Не удалось синхронизировать время ни на одном устройстве")
                
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка синхронизации времени: {e}")

    def on_close(self):
        """Обработчик закрытия окна приложения"""
        self.disconnect_serial()
        self.destroy()
        
    def disconnect_serial(self, manual_disconnect=True):
        """Закрыть последовательное соединение
        
        Args:
            manual_disconnect (bool): True если отключение инициировано пользователем
        """
        global ser

        self.send_ctrl_command(20)
        time.sleep(0.1)
        self.manual_disconnect = manual_disconnect  # Сохраняем флаг ручного отключения
        if ser and ser.is_open:
            ser.close()
        self.connected = False
        self.scan_button.config(text="Подключить")
        self.port_combo.config(state="readonly")
        self.baud_combo.config(state="readonly")
        
        # Очищаем таблицу тестеров
        for item in self.testers_table.get_children():
            self.testers_table.delete(item)
            
        # Сбрасываем кнопку запуска
        self.start_stop_btn.config(state="disabled")
            
    def stop_polling(self):
        global ser
        
        self.polling = False
        self.start_stop_btn.config(text="Запустить")
        
        # Cancel any pending uptime updates
        if self.uptime_job is not None:
            self.after_cancel(self.uptime_job)
            self.uptime_job = None
        
        if not self.connected or not ser or not ser.is_open:
            return
            
        try:
            # Send stop command (17) to all testers
            self.send_ctrl_command(17)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка остановки тестирования: {e}")
            # Don't close the port here, let it be managed by disconnect_serial

    def poll_testers(self):
        """Poll all testers for their current result counters"""
        global ser
        
        # Check if we should stop polling
        if not self.polling or not self.connected or not ser or not ser.is_open:
            self.polling = False
            self.start_stop_btn.config(text="Запустить")
            if (not ser or not ser.is_open) and not hasattr(self, 'manual_disconnect'):
                messagebox.showerror("Ошибка", "Соединение потеряно!")
                self.disconnect_serial(manual_disconnect=False)
            return
        
        try:
            # Process each row in the table
            for row_id in self.testers_table.get_children():
                try:
                    # Get the row data
                    row_data = self.testers_table.item(row_id)['values']
                    if not row_data or len(row_data) < 2:  # Skip invalid rows
                        continue
                        
                    # Get tester address from the first column
                    try:
                        addr = int(row_data[0])
                    except (ValueError, TypeError) as e:
                        print(f"Неверный адрес тестера в строке {row_id}: {row_data[0]}")
                        continue  # Skip invalid addresses
                    
                    # Build read command for CURRENT_RESULT_COUNTERS (10 registers)
                    cmd = build_read_command(addr, REGISTERS["CURRENT_RESULT_COUNTERS"], 10)
                    
                    # Send command and read response
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser.write(cmd)
                    time.sleep(0.2)  # Small delay for response
                    
                    # Read response (1 addr + 1 func + 1 byte count + 2*10 data + 2 crc = 25 bytes)
                    response = ser.read(25)
                    if len(response) == 25:
                        # Parse the response (10 registers = 20 bytes of data)
                        counters = parse_response(response, 10)
                        
                        # Update the row with new counter values
                        self._update_tester_row(row_id, counters)
                    
                except Exception as e:
                    print(f"Ошибка при опросе тестера {row_data[0] if row_data else '?'} (строка {row_id}): {e}")
                    # Mark row with error
                    self.testers_table.item(row_id, tags=('error',))
            
            # Update test duration if test is running
            if self.test_start_time:
                self.update_uptime()
                
            # Schedule next poll in 1 second if still polling
            if self.polling:
                self.after(1000, self.poll_testers)
                
        except Exception as e:
            print(f"Критическая ошибка при опросе: {e}")
            self.polling = False
            self.start_stop_btn.config(text="Запустить")

    def _get_all_tester_addresses(self):
        addresses = []
        for row_id in self.testers_table.get_children():
            values = self.testers_table.item(row_id)['values']
            if values and isinstance(values[0], (int, str)) and str(values[0]).isdigit():
                addresses.append(int(values[0]))
        return addresses

    def send_ctrl_command(self, value, addr=0x00):
        """
        Send a command to the CTRL register (0x04)
        :param value: Command value to send
        :param addr: Device address (0x00 for broadcast)
        """
        global ser
        if not ser or not ser.is_open:
            print("Ошибка: Нет активного соединения")
            return False
            
        try:
            cmd = bytearray([
                addr & 0xFF,      # Device address
                0x06,             # Function code 6 (Write Single Register)
                0x00, 0x04,       # Register address (CTRL)
                (value >> 8) & 0xFF,  # Value high byte
                value & 0xFF       # Value low byte
            ])
            
            crc = modbus_crc(cmd)
            cmd.append(crc & 0xFF)
            cmd.append((crc >> 8) & 0xFF)
            
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(cmd)
            time.sleep(0.1)  # Small delay for the command to be processed
            return True
            
        except Exception as e:
            print(f"Ошибка при отправке команды CTRL {value}: {e}")
            return False

    def _show_context_menu(self, event):
        """Показать контекстное меню при ПКМ"""
        # Определяем, по какому элементу был клик
        row_id = self.testers_table.identify_row(event.y)
        if not row_id:
            return
            
        # Выделяем строку, по которой был клик
        self.testers_table.selection_set(row_id)
        self.selected_row = row_id
        
        # Обновляем состояние пункта меню в зависимости от состояния тестирования
        menu_state = 'disabled' if self.polling or not self.connected else 'normal'
        self.context_menu.entryconfigure(0, state=menu_state)
        
        # Показываем контекстное меню
        try:
            self.context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.context_menu.grab_release()

    def _save_test_results_to_excel(self, results, device_addr, filename=None):
        """
        Сохраняет результаты тестов в Excel файл
        
        Args:
            results: Список словарей с результатами тестов
            device_addr: Адрес тестера
            filename: Полный путь к файлу (если None, будет создан автоматически)
            
        Returns:
            str: Путь к сохраненному файлу или None в случае ошибки
        """
        try:
            if not results:
                messagebox.showwarning("Предупреждение", "Нет данных для сохранения")
                return None
                
            # Получаем BIZ_TYPE из таблицы
            biz_type = None
            for item in self.testers_table.get_children():
                values = self.testers_table.item(item)['values']
                if values and values[0] == device_addr and len(values) > 1:
                    biz_type = values[1]
                    break
            
            if filename is None:
                # Формируем имя файла в формате LongTestResult_ADDR-N_BIZ-N_Y-M-D_H-M-S.xlsx
                timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                filename = f"LongTestResult_ADDR-{device_addr}_BIZ-{biz_type or 'N'}_{timestamp}.xlsx"
                filename = os.path.join(self.save_dir, filename)
            else:
                # Убедимся, что директория существует
                os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            # Создаем DataFrame из результатов
            import pandas as pd
            
            # Преобразуем вложенные словари в плоскую структуру
            flat_results = []
            for result in results:
                flat_result = {}
                for key, value in result.items():
                    if isinstance(value, dict):
                        for subkey, subvalue in value.items():
                            flat_result[f"{key}_{subkey}"] = subvalue
                    else:
                        flat_result[key] = value
                flat_results.append(flat_result)
            
            df = pd.DataFrame(flat_results)
            
            # Сохраняем в Excel
            df.to_excel(filename, index=False, engine='openpyxl')
            return filename
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось сохранить результаты: {str(e)}")
            return None
            
    def _on_read_all_clicked(self):
        """
        Обработчик нажатия кнопки 'Считать все'.
        Читает все тесты со всех тестеров и сохраняет их в отдельные Excel файлы
        в папке с текущей датой и временем.
        """
        global ser
        
        if not hasattr(self, 'connected') or not self.connected or not ser or not ser.is_open:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
            
        # Получаем список всех тестеров
        testers = []
        for item in self.testers_table.get_children():
            values = self.testers_table.item(item)['values']
            if values:  # Проверяем, что строка не пустая
                testers.append({
                    'addr': values[0],
                    'biz_type': values[1] if len(values) > 1 else 'N'
                })
        
        if not testers:
            messagebox.showwarning("Предупреждение", "Нет доступных тестеров")
            return
        
        # Создаем папку с текущей датой и временем
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        folder_name = f"TestResults_{timestamp}"
        save_dir = os.path.join(self.save_dir, folder_name)
        os.makedirs(save_dir, exist_ok=True)
        
        # Создаем диалог ожидания
        progress = tk.Toplevel(self)
        progress.title("Чтение тестов")
        progress.geometry("400x120")
        progress.resizable(False, False)
        
        # Центрируем окно
        progress.update_idletasks()
        width = progress.winfo_width()
        height = progress.winfo_height()
        x = (progress.winfo_screenwidth() // 2) - (width // 2)
        y = (progress.winfo_screenheight() // 2) - (height // 2)
        progress.geometry(f'400x120+{x}+{y}')
        
        label = ttk.Label(progress, text="Подготовка к чтению...")
        label.pack(pady=5)
        
        progress_var = tk.DoubleVar()
        progress_bar = ttk.Progressbar(progress, variable=progress_var, maximum=100)
        progress_bar.pack(fill='x', padx=20, pady=5)
        
        status_label = ttk.Label(progress, text="")
        status_label.pack(pady=5)
        
        progress.update()
        
        total_testers = len(testers)
        success_count = 0
        
        for i, tester in enumerate(testers, 1):
            device_addr = tester['addr']
            label.config(text=f"Обработка тестера {i} из {total_testers}")
            status_label.config(text=f"Адрес: {device_addr}")
            progress_var.set((i / total_testers) * 100)
            progress.update()
            
            try:
                # Читаем количество тестов
                test_count = read_test_count(device_addr)
                
                if test_count is None or test_count == 0:
                    continue
                
                results = []
                
                # Читаем все тесты для текущего тестера
                for test_num in range(1, test_count + 1):
                    status_label.config(text=f"Адрес: {device_addr} | Тест {test_num} из {test_count}")
                    progress.update()
                    
                    result = self.read_test_result(test_num, device_addr)
                    if result:
                        results.append(result)
                
                # Сохраняем результаты в Excel, если есть что сохранять
                if results:
                    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                    filename = f"LongTestResult_ADDR-{device_addr}_BIZ-{tester['biz_type']}_{timestamp}.xlsx"
                    filename = os.path.join(save_dir, filename)
                    
                    # Используем существующий метод для сохранения
                    if self._save_test_results_to_excel(results, device_addr, filename):
                        success_count += 1
                
            except Exception as e:
                messagebox.showerror("Ошибка", f"Ошибка при чтении тестера {device_addr}: {str(e)}")
                continue
        
        # Закрываем окно прогресса
        progress.destroy()
        
        if success_count > 0:
            messagebox.showinfo(
                "Готово",
                f"Успешно обработано {success_count} из {total_testers} тестеров.\n"
                f"Результаты сохранены в папку:\n{save_dir}"
            )
            # Открываем папку с результатами
            os.startfile(save_dir)
        else:
            messagebox.showwarning("Предупреждение", "Не удалось прочитать ни одного тестера")

    def _on_read_clicked(self):
        """
        Обработчик нажатия кнопки 'Считать' в контекстном меню.
        Читает все тесты с выбранного тестера и сохраняет их в Excel файл.
        """
        global ser
        
        if not hasattr(self, 'selected_row') or self.selected_row is None:
            messagebox.showwarning("Предупреждение", "Не выбрана строка с тестером")
            return
            
        if not hasattr(self, 'connected') or not self.connected or not ser or not ser.is_open:
            messagebox.showerror("Ошибка", "Нет активного соединения с устройством")
            return
            
        # Получаем адрес тестера из выбранной строки (первая колонка)
        item = self.testers_table.item(self.selected_row)
        device_addr = int(item['values'][0])
        
        try:
            # Создаем диалог ожидания
            progress = tk.Toplevel(self)
            progress.title("Чтение тестов")
            progress.geometry("300x100")
            progress.resizable(False, False)
            
            # Центрируем окно
            progress.update_idletasks()
            width = progress.winfo_width()
            height = progress.winfo_height()
            x = (progress.winfo_screenwidth() // 2) - (width // 2)
            y = (progress.winfo_screenheight() // 2) - (height // 2)
            progress.geometry(f'300x100+{x}+{y}')
            
            label = ttk.Label(progress, text="Чтение тестов...")
            label.pack(pady=10)
            
            progress_var = tk.DoubleVar()
            progress_bar = ttk.Progressbar(progress, variable=progress_var, maximum=100)
            progress_bar.pack(fill='x', padx=20, pady=5)
            
            progress.update()
            
            # Читаем количество тестов
            test_count = read_test_count(device_addr)
            
            if test_count is None or test_count == 0:
                messagebox.showinfo("Информация", "Нет доступных тестов для чтения")
                progress.destroy()
                return
            
            results = []
            
            # Читаем все тесты
            for i in range(1, test_count + 1):
                label.config(text=f"Чтение теста {i} из {test_count}...")
                progress_var.set((i / test_count) * 100)
                progress.update()
                
                result = self.read_test_result(i, device_addr)
                if result:
                    results.append(result)
            
            # Закрываем окно прогресса
            progress.destroy()
            
            if not results:
                messagebox.showwarning("Предупреждение", "Не удалось прочитать ни одного теста")
                return
            
            # Сохраняем результаты в Excel
            filename = self._save_test_results_to_excel(results, device_addr)
            
            if filename:
                messagebox.showinfo("Готово", f"Успешно прочитано {len(results)} тестов.\nРезультаты сохранены в файл:\n{filename}")
                
        except Exception as e:
            if 'progress' in locals():
                progress.destroy()
            messagebox.showerror("Ошибка", f"Не удалось прочитать тесты: {str(e)}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось прочитать результат теста: {str(e)}")

    def _on_write_clicked(self):
        """
        Обработчик нажатия кнопки 'Записать'.
        Записывает значения периода и количества во все тестеры из таблицы.
        """
        global ser
        
        if not hasattr(self, 'connected') or not self.connected or not ser or not ser.is_open:
            messagebox.showerror("Ошибка", "Нет активного соединения")
            return
            
        try:
            # Получаем значения из полей ввода
            period_seconds = self.period_var.get()
            count = self.count_var.get()
            
            # Конвертируем период в миллисекунды
            period_ms = period_seconds * 1000
            
            # Получаем список адресов тестеров из таблицы
            for item in self.testers_table.get_children():
                addr = int(self.testers_table.item(item)['values'][0])
                
                # Записываем LONG_TEST_SHORT_PERIOD_ms
                cmd = struct.pack('>B B H H', 
                                addr,  # Адрес устройства
                                0x06,  # Код функции записи регистра
                                REGISTERS["LONG_TEST_SHORT_PERIOD_ms"],  # Адрес регистра
                                period_ms)  # Значение
                
                # Добавляем CRC и отправляем
                cmd += struct.pack('<H', modbus_crc(cmd))
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(cmd)
                
                # Ждем ответ (должен быть эхо запроса)
                time.sleep(0.1)  # Даем время на ответ
                response = ser.read(8)  # Ожидаем 8 байт ответа
                if not (len(response) == 8 and response[0] == addr and response[1] == 0x06):
                    messagebox.showerror("Ошибка", f"Неверный ответ от тестера {addr} при записи периода")
                    return
                
                # Записываем LONG_TEST_LONG_PERIOD
                cmd = struct.pack('>B B H H', 
                                addr,  # Адрес устройства
                                0x06,  # Код функции записи регистра
                                REGISTERS["LONG_TEST_LONG_PERIOD"],  # Адрес регистра
                                count)  # Значение
                
                # Добавляем CRC и отправляем
                cmd += struct.pack('<H', modbus_crc(cmd))
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(cmd)
                
                # Ждем ответ (должен быть эхо запроса)
                time.sleep(0.1)  # Даем время на ответ
                response = ser.read(8)  # Ожидаем 8 байт ответа
                if not (len(response) == 8 and response[0] == addr and response[1] == 0x06):
                    messagebox.showerror("Ошибка", f"Неверный ответ от тестера {addr} при записи количества")
                    return
            
            messagebox.showinfo("Готово", "Значения записаны во все тестеры")
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка при записи значений: {e}")

        
    def _update_tester_row(self, row_id, counters):
        """
        Update a row in the testers table with new counter values
        :param row_id: ID of the row to update
        :param counters: List of 10 counter values (registers)
        """
        try:
            # Get current row values
            values = list(self.testers_table.item(row_id)['values'])
            if len(values) < 2:  # Need at least address and type
                return
                
            # Update counter values (skip first 2 columns: address and type)
            for i in range(min(len(counters), 10)):  # Ensure we don't go out of bounds
                values[2 + i] = counters[i]
            
            # Check for any non-zero counters (errors)
            has_errors = any(c != 0 for c in counters if isinstance(c, (int, float)))
            
            # Update the row with new values and appropriate styling
            if has_errors:
                self.testers_table.item(row_id, values=values, tags=('error',))
            else:
                # Restore zebra striping if no errors
                # Get the index of the row to determine if it's even or odd
                row_index = list(self.testers_table.get_children()).index(row_id)
                tag = 'evenrow' if row_index % 2 == 0 else 'oddrow'
                self.testers_table.item(row_id, values=values, tags=(tag,))
                
        except Exception as e:
            print(f"Ошибка при обновлении строки {row_id}: {e}")

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def on_dropdown_click(self, event):
        current_value = self.port_var.get()
        self.port_combo['values'] = self.get_serial_ports()
        if current_value in self.port_combo['values']:
            self.port_var.set(current_value)
        elif self.port_combo['values']:
            self.port_combo.current(0)

    def read_test_result(self, test_index, device_addr):
        """Чтение результата конкретного теста с использованием кастомной функции Modbus 0x42
        
        Args:
            test_index: Индекс теста для чтения
            device_addr: Адрес устройства Modbus
            
        Returns:
            dict: Данные теста или None в случае ошибки
        """
        global ser
        if not ser or not ser.is_open:
            raise Exception("Нет активного соединения с устройством")
            
        try:
            # Формируем команду с кодом функции 0x42 и индексом теста
            # Отправляем индекс теста в формате big-endian (старший байт первый)
            command = [device_addr, 0x42, (test_index >> 8) & 0xFF, test_index & 0xFF]
            # Вычисляем CRC
            crc = modbus_crc(command)
            # Добавляем CRC к команде (младший байт, затем старший)
            command.append(crc & 0xFF)
            command.append((crc >> 8) & 0xFF)
            
            # Очищаем буфер перед отправкой
            while ser.in_waiting > 0:
                ser.read(ser.in_waiting)
            
            # Отправляем команду
            ser.write(bytes(command))
            
            # Увеличиваем время ожидания из-за большего объема данных
            time.sleep(0.5)
            
            # Проверяем, есть ли данные для чтения
            if ser.in_waiting < 4:  # Минимальная длина ответа: адрес(1) + функция(1) + количество байт(1) + код ошибки(1) + CRC(2)
                return None
                
            # Читаем ответ
            response = ser.read(ser.in_waiting)
            
            # Проверяем адрес устройства и код функции
            if len(response) < 4 or response[0] != device_addr or response[1] != 0x42:
                return None
                
            # Проверяем количество байт данных
            data_bytes = response[2]
            
            # Если получен код ошибки (1 байт)
            if data_bytes == 1:
                error_code = response[3]
                return {"error": True, "error_code": error_code}
                
            # Проверяем, что размер ответа соответствует ожидаемому
            # Размер Test_data: 4 (timestamp) + 3*2*10 (avg,min,max для 10 параметров) + 10*2 (счетчики) = 74 байта
            expected_size = 3 + data_bytes + 2  # адрес(1) + функция(1) + количество байт(1) + данные(data_bytes) + CRC(2)
            if len(response) < expected_size or data_bytes < 74:  # Минимум 74 байта данных
                return None
                
            # Смещение данных в ответе: 3 (адрес, функция, количество байт)
            offset = 3
            
            # Извлекаем timestamp (4 байта)
            timestamp = (response[offset+3] << 24) | (response[offset+2] << 16) | \
                       (response[offset+1] << 8) | response[offset]
            offset += 4
            date_time = datetime.fromtimestamp(timestamp)
            
            # Функция для извлечения среднего, мин и макс значений
            def read_measurement():
                nonlocal offset
                avg = to_signed_16bit((response[offset+1] << 8) | response[offset]) / 100.0
                offset += 2
                min_val = to_signed_16bit((response[offset+1] << 8) | response[offset]) / 100.0
                offset += 2
                max_val = to_signed_16bit((response[offset+1] << 8) | response[offset]) / 100.0
                offset += 2
                return {"avg": avg, "min": min_val, "max": max_val}
            
            # Извлекаем значения напряжений (В)
            u_out_off = read_measurement()
            du_out = read_measurement()
            u_tp_off = read_measurement()
            
            # Извлекаем значения токов (мА)
            i_out_short = read_measurement()
            i_out_fail = read_measurement()
            i_in_off = read_measurement()
            i_in_load = read_measurement()
            i_in_short = read_measurement()
            i_in_fail = read_measurement()
            
            # Функция для извлечения счетчиков (беззнаковые 16-битные значения)
            def read_counter():
                nonlocal offset
                value = (response[offset+1] << 8) | response[offset]
                offset += 2
                return value
            
            # Извлекаем счетчики
            counters = {
                "U_OUT_OFF_COUNTER": read_counter(),
                "dU_OUT_COUNTER": read_counter(),
                "U_TP_COUNTER": read_counter(),
                "I_OUT_SHORT_COUNTER": read_counter(),
                "I_OUT_FAIL_COUNTER": read_counter(),
                "I_IN_OFF_COUNTER": read_counter(),
                "I_IN_LOAD_COUNTER": read_counter(),
                "I_IN_SHORT_COUNTER": read_counter(),
                "I_IN_FAIL_COUNTER": read_counter(),
                "UART_ERROR_COUNTER": read_counter()
            }
            
            # Формируем словарь с результатами теста
            result = {
                "datetime": date_time.strftime("%d.%m.%Y %H:%M:%S"),
                "u_out_off": u_out_off,
                "du_out": du_out,
                "u_tp_off": u_tp_off,
                "i_out_short": i_out_short,
                "i_out_fail": i_out_fail,
                "i_in_off": i_in_off,
                "i_in_load": i_in_load,
                "i_in_short": i_in_short,
                "i_in_fail": i_in_fail,
                "counters": counters
            }
            
            return result
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось прочитать результат теста: {str(e)}")
            return None

    def send_current_time(self, ser, device_addr):
        """
        Отправляет текущее время компьютера на тестер с использованием функции Modbus 0x43
        
        Args:
            ser: Объект последовательного порта
            device_addr: Адрес устройства Modbus
            
        Returns:
            bool: True если время успешно отправлено, иначе False
        """
        try:
            # Получаем текущее время
            current_time = int(time.time())
            
            # Формируем Modbus сообщение
            # 0x43 - код функции для установки значения счетчика RTC
            data = bytearray([device_addr, 0x43])
            
            # Добавляем 32-битное значение времени (MSB first)
            data.append((current_time >> 24) & 0xFF)  # Старший байт (MSB)
            data.append((current_time >> 16) & 0xFF)  # Второй байт
            data.append((current_time >> 8) & 0xFF)   # Третий байт
            data.append(current_time & 0xFF)          # Младший байт (LSB)
            
            # Вычисляем CRC и добавляем к данным
            crc = modbus_crc(data)
            data.append(crc & 0xFF)         # Младший байт CRC
            data.append((crc >> 8) & 0xFF)  # Старший байт CRC
            
            # Отправляем данные
            if ser and ser.is_open:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(data)
                time.sleep(0.1)  # Небольшая задержка для обработки запроса
                
                # Читаем ответ (если он есть)
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    return True
            return False
            
        except Exception as e:
            print(f"Ошибка при отправке времени на устройство {device_addr}: {str(e)}")
            return False
            
    def handle_scan(self):
        global ser
        
        # If already connected, disconnect
        if self.connected:
            self.disconnect_serial()
            return
            
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showerror("Ошибка", "Не выбран COM-порт!")
            return
            
        try:
            # Open the serial port
            ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.connected = True
            self.scan_button.config(text="Отключить")
            self.port_combo.config(state="disabled")
            self.baud_combo.config(state="disabled")
            
            # Clear the old table
            for row in self.testers_table.get_children():
                self.testers_table.delete(row)
                
            # Scan for testers
            found = 0
            for idx, addr in enumerate(range(2, 21)):
                try:
                    cmd = build_read_command(addr, REGISTERS["BIZ_TYPE"], 1)
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser.write(cmd)
                    response = ser.read(7)
                    if len(response) == 7:
                        try:
                            values = parse_response(response, 1)
                            biz_type = values[0]
                            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                            row_values = [addr, biz_type] + ["-...-" for _ in self.tolerance_names]
                            self.testers_table.insert("", "end", values=row_values, tags=(tag,))
                            found += 1
                            time.sleep(0.1)
                        except Exception:
                            continue
                except Exception:
                    continue
                    
            if found == 0:
                messagebox.showerror("Ошибка", "Не найдено ни одного тестера!")
                self.disconnect_serial()
            else:
                # Check if any tester already has a test in progress (TST_TYPE == 3)
                # We only need to check the first tester since all testers should be in the same state
                test_in_progress = False
                self.send_ctrl_command(19)
                time.sleep(0.1)
                
                # Get the first tester's address
                first_item = next(iter(self.testers_table.get_children()), None)
                if first_item:
                    try:                               
                        addr = int(self.testers_table.item(first_item)['values'][0])
                        # Read LONG_TEST_SHORT_PERIOD_ms (1 register, ms)
                        cmd = build_read_command(addr, REGISTERS["LONG_TEST_SHORT_PERIOD_ms"], 1)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(cmd)
                        time.sleep(0.1)
                        response = ser.read(7)
                        if len(response) == 7:
                            values = parse_response(response, 1)
                            if values:
                                # Convert ms to seconds and update the period field
                                self.period_var.set(values[0] // 1000)
                                
                        # Read LONG_TEST_LONG_PERIOD (1 register, seconds)
                        cmd = build_read_command(addr, REGISTERS["LONG_TEST_LONG_PERIOD"], 1)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(cmd)
                        time.sleep(0.1)
                        response = ser.read(7)
                        if len(response) == 7:
                            values = parse_response(response, 1)
                            if values:
                                # Update the count field
                                self.count_var.set(values[0])
                        
                        time.sleep(0.1)
                        # Read TST_TYPE register only from the first tester
                        cmd = build_read_command(addr, REGISTERS["TST_TYPE"], 1)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(cmd)
                        response = ser.read(7)
                        if len(response) == 7:
                            values = parse_response(response, 1)
                            if values and values[0] == 3:
                                test_in_progress = True
                    except Exception as e:
                        print(f"Error checking TST_TYPE: {e}")
                
                if test_in_progress:
                    # If test is already running, read the start time and calculate elapsed time
                    try:
                        # Read LONG_TEST_START_TIME (2 registers = 4 bytes)
                        cmd = build_read_command(addr, REGISTERS["LONG_TEST_START_TIME"], 2)
                        
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(cmd)
                        
                        # Read response with timeout
                        response = bytearray()
                        start_time = time.time()
                        while len(response) < 9 and (time.time() - start_time) < 1.0:  # 1 second timeout
                            if ser.in_waiting > 0:
                                response.extend(ser.read(ser.in_waiting))
                            time.sleep(0.01)
                        
                        if len(response) >= 7:  # At least 7 bytes for a valid response
                            values = parse_response(response, 2)  # Read 2 registers (4 bytes)
                            
                            if values and len(values) >= 2:
                                # Combine two 16-bit values into one 32-bit value (big-endian)
                                start_time = (values[0] << 16) | values[1]
                                current_time = int(time.time())
                                elapsed_seconds = current_time - start_time
                                
                                # Update the test start time to reflect the elapsed time
                                if self.test_start_time is None:
                                    self.test_start_time = time.time() - elapsed_seconds
                    except Exception as e:
                        pass
                    
                    # Enable polling
                    self.start_polling()
                else:
                    # Send broadcast command to write 18 to CTRL register (0x04)
                    time.sleep(0.2)
                    self.send_ctrl_command(18)
                
                self.start_stop_btn.config(state="active")
                
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось открыть порт: {e}")
            if ser and ser.is_open:
                ser.close()
            self.connected = False

if __name__ == "__main__":
    app = LongTestApp()
    app.mainloop()
