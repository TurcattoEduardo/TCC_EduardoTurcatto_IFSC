import tkinter as tk
from tkinter import ttk, messagebox
import serial
import struct
import threading

# Constantes definidas no protocolo
PREFIX_DATA = 0x01
ID_ACC = 0xA1  # Acelerômetro
ID_RPS = 0xB1  # Velocidade (RPS)
ID_ENC = 0xB2  # Posição do encoder

# Função para calcular CRC-16 (polinômio 0xA001)
def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# Função para decodificar mensagens
def decode_message(buffer):
    try:
        # Verifica o tamanho mínimo da mensagem
        if len(buffer) < 7:  # HEADER_SIZE + CRC_SIZE
            return None

        # Extrai o cabeçalho
        prefix, identifier, payload_size = struct.unpack("BBB", buffer[:3])

        # Valida o prefixo
        if prefix != PREFIX_DATA:
            print("Erro: Prefixo inválido")
            return None

        # Verifica se o tamanho do buffer é consistente
        if len(buffer) != (3 + payload_size + 2):
            print("Erro: Tamanho inconsistente")
            return None

        # Extrai o payload e o CRC
        payload = buffer[3:3 + payload_size]
        received_crc = struct.unpack("<H", buffer[-2:])[0]
        calculated_crc = calculate_crc(buffer[:-2])

        if received_crc != calculated_crc:
            print(f"Erro no CRC. Recebido: {received_crc:04X}, Calculado: {calculated_crc:04X}")
            return None

        # Mensagem válida
        return {"prefix": prefix, "identifier": identifier, "payload": payload}
    except Exception as e:
        print(f"Erro na decodificação: {e}")
        return None

# Classe principal do GUI
class SerialApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Serial Communication GUI")
        self.geometry("400x400")

        self.serial_connection = None
        self.running = False
        self.buffer = b""

        self.create_widgets()

    def create_widgets(self):
        self.port_label = tk.Label(self, text="Porta Serial:")
        self.port_label.pack()
        self.port_combobox = ttk.Combobox(self, values=["COM3", "COM4", "COM5"], width=20)
        self.port_combobox.pack()

        self.connect_button = tk.Button(self, text="Conectar", command=self.connect_serial)
        self.connect_button.pack()

        self.data_frame = tk.Frame(self)
        self.data_frame.pack(pady=10)

        self.labels = {}
        for name in ["Accel X", "Accel Y", "Accel Z", "RPS", "ENC"]:
            frame = tk.Frame(self.data_frame)
            frame.pack()
            label = tk.Label(frame, text=f"{name}:")
            label.pack(side=tk.LEFT)
            self.labels[name] = tk.Label(frame, text="N/A")
            self.labels[name].pack(side=tk.LEFT)

        self.disconnect_button = tk.Button(self, text="Desconectar", command=self.disconnect_serial, state=tk.DISABLED)
        self.disconnect_button.pack()

    def connect_serial(self):
        port = self.port_combobox.get()
        try:
            self.serial_connection = serial.Serial(port, 115200, timeout=1)
            self.running = True
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            threading.Thread(target=self.read_serial, daemon=True).start()
            messagebox.showinfo("Conexão", "Conectado com sucesso!")
        except Exception as e:
            messagebox.showerror("Erro", f"Não foi possível conectar: {e}")

    def disconnect_serial(self):
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        messagebox.showinfo("Conexão", "Desconectado.")

    def read_serial(self):
        while self.running:
            try:
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    self.buffer += data

                    while len(self.buffer) >= 7:  # HEADER_SIZE + CRC_SIZE
                        # Encontre o prefixo inicial
                        if self.buffer[0] != PREFIX_DATA:
                            self.buffer = self.buffer[1:]  # Remova bytes inválidos
                            continue

                        # Verifique o tamanho da mensagem
                        _, _, payload_size = struct.unpack("BBB", self.buffer[:3])
                        message_length = 3 + payload_size + 2

                        if len(self.buffer) < message_length:
                            break  # Aguardando mais dados

                        # Extraia e processe a mensagem completa
                        message = self.buffer[:message_length]
                        self.buffer = self.buffer[message_length:]  # Remova a mensagem do buffer

                        decoded = decode_message(message)
                        if decoded:
                            self.update_fields(decoded)
            except Exception as e:
                print(f"Erro na leitura serial: {e}")

    def update_fields(self, message):
        identifier = message["identifier"]
        payload = message["payload"]

        if identifier == ID_ACC:  # Accel
            accel_x, accel_y, accel_z = struct.unpack("<fff", payload)
            self.labels["Accel X"].config(text=f"{accel_x:.2f}")
            self.labels["Accel Y"].config(text=f"{accel_y:.2f}")
            self.labels["Accel Z"].config(text=f"{accel_z:.2f}")
        elif identifier == ID_RPS:  # RPS
            rps = struct.unpack("<H", payload)[0]
            self.labels["RPS"].config(text=f"{rps}")
        elif identifier == ID_ENC:  # ENC
            enc = struct.unpack("<H", payload)[0]
            self.labels["ENC"].config(text=f"{enc}")

if __name__ == "__main__":
    app = SerialApp()
    app.mainloop()
