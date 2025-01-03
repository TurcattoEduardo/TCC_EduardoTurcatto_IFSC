import tkinter as tk
from tkinter import ttk, messagebox
import serial
import struct
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

# Constantes definidas no protocolo
PREFIX_DATA = 0x01
PREFIX_CMD = 0x02
ID_ACC = 0xA1
ID_RPS = 0xB1
ID_ENC = 0xB2
ID_CON = 0xC1

# Função para calcular CRC-16 (polinômio 0xA001)


# Função para decodificar mensagens
def decode_message(buffer):
    try:
        if len(buffer) < 7:  # HEADER_SIZE + CRC_SIZE
            return None

        prefix, identifier, payload_size = struct.unpack("BBB", buffer[:3])

        if prefix != PREFIX_DATA:
            return None

        if len(buffer) != (3 + payload_size + 2):
            return None

        payload = buffer[3:3 + payload_size]
        received_crc = struct.unpack("<H", buffer[-2:])[0]
        calculated_crc = calculate_crc(buffer[:-2])

        if received_crc != calculated_crc:
            return None

        return {"prefix": prefix, "identifier": identifier, "payload": payload}
    except Exception as e:
        return None

import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

# Classe principal do GUI
class SerialApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Laboratório - GUI de Comunicação Serial")
        self.geometry("1000x600")

        # Inicia em tela cheia
        self.attributes('-fullscreen', True)

        # Permite sair da tela cheia com Esc
        self.bind('<Escape>', self.exit_fullscreen)

        self.serial_connection = None
        self.running = False
        self.buffer = b""
        self.data = {"Accel X": deque(maxlen=100), "Accel Y": deque(maxlen=100),
                     "Accel Z": deque(maxlen=100), "RPS": deque(maxlen=100), "ENC": deque(maxlen=100)}

        self.current_plot = "Accel X"
        self.status_text = tk.StringVar(value="Desconectado")

        self.create_widgets()

    def exit_fullscreen(self, event=None):
        self.attributes('-fullscreen', False)

    def create_widgets(self):
        # PanedWindow para divisão ajustável
        paned_window = tk.PanedWindow(self, orient=tk.HORIZONTAL, sashrelief=tk.RAISED)
        paned_window.pack(fill=tk.BOTH, expand=True)

        # Frames para as metades ajustáveis
        self.left_frame = tk.Frame(paned_window, width=50, height=600, bg="#F5F5F5")
        self.right_frame = tk.Frame(paned_window, width=950, height=600)

        paned_window.add(self.left_frame)
        paned_window.add(self.right_frame)

        # Parte Esquerda: Configuração e Dados
        self.create_left_panel()

        # Parte Direita: Gráfico e Ações
        self.create_right_panel()

    def create_left_panel(self):
        # Configurações de Conexão
        config_frame = tk.LabelFrame(self.left_frame, text="Configurações de Conexão", bg="#F5F5F5")
        config_frame.pack(fill=tk.X, padx=10, pady=10)

        self.port_label = tk.Label(config_frame, text="Porta Serial:", bg="#F5F5F5")
        self.port_label.pack(anchor="w", padx=10, pady=5)
        self.port_combobox = ttk.Combobox(config_frame, values=["COM3", "COM4", "COM5"], width=20)
        self.port_combobox.pack(padx=10, pady=5)

        self.connect_button = tk.Button(config_frame, text="Conectar", command=self.connect_serial)
        self.connect_button.pack(padx=10, pady=5)

        self.disconnect_button = tk.Button(config_frame, text="Desconectar", command=self.disconnect_serial, state=tk.DISABLED)
        self.disconnect_button.pack(padx=10, pady=5)

        self.status_label = tk.Label(config_frame, textvariable=self.status_text, fg="blue", bg="#F5F5F5")
        self.status_label.pack(padx=10, pady=5)

        # Controle de Dados
        control_frame = tk.LabelFrame(self.left_frame, text="Controle de Dados", bg="#F5F5F5")
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        self.start_button = tk.Button(control_frame, text="Iniciar", command=self.send_start_command)
        self.start_button.pack(padx=10, pady=5)

        self.stop_button = tk.Button(control_frame, text="Parar", command=self.send_stop_command)
        self.stop_button.pack(padx=10, pady=5)

        # Exibição de Dados
        data_frame = tk.LabelFrame(self.left_frame, text="Valores Recebidos", bg="#F5F5F5")
        data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.labels = {}
        for name in ["Accel X", "Accel Y", "Accel Z", "RPS", "ENC"]:
            frame = tk.Frame(data_frame, bg="#F5F5F5")
            frame.pack(anchor="w", padx=10, pady=2)
            label = tk.Label(frame, text=f"{name}:", bg="#F5F5F5")
            label.pack(side=tk.LEFT)
            self.labels[name] = tk.Label(frame, text="N/A", bg="#F5F5F5", width=10, anchor="e")
            self.labels[name].pack(side=tk.RIGHT)

        # Logs
        log_frame = tk.LabelFrame(self.left_frame, text="Logs", bg="#F5F5F5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.log_text = tk.Text(log_frame, height=10, bg="#FFFFFF", state=tk.DISABLED)
        self.log_text.pack(fill=tk.X, padx=5, pady=5)

        # Botão de Fechar no Final
        close_frame = tk.Frame(self.left_frame, bg="#F5F5F5")
        close_frame.pack(fill=tk.X, padx=10, pady=10)

        self.close_button = tk.Button(close_frame, text="Fechar", command=self.close_program)
        self.close_button.pack(padx=10, pady=5)

    def close_program(self):
        """Fecha o programa."""
        self.destroy()


    def create_right_panel(self):
        # Organização do painel de controle do gráfico
        graph_control_frame = tk.LabelFrame(self.right_frame, text="Configuração do Gráfico")
        graph_control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        self.plot_select = ttk.Combobox(graph_control_frame, values=["Accel X", "Accel Y", "Accel Z", "RPS", "ENC"], state="readonly")
        self.plot_select.set("Accel X")
        self.plot_select.pack(side=tk.LEFT, padx=5)
        self.plot_select.bind("<<ComboboxSelected>>", self.update_plot)

        self.clear_button = tk.Button(graph_control_frame, text="Limpar Gráfico Selecionado", command=self.clear_current_plot)
        self.clear_button.pack(side=tk.LEFT, padx=5)

        self.clear_all_button = tk.Button(graph_control_frame, text="Limpar Todos os Gráficos", command=self.clear_all_plots)
        self.clear_all_button.pack(side=tk.LEFT, padx=5)

        # Adicionando o gráfico
        graph_frame = tk.LabelFrame(self.right_frame, text="Gráfico")
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.figure, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlabel("Tempo (s)")
        self.ax.set_ylabel("Aceleração (mm/s²)")  # Inicialmente para Accel X
        self.ax.set_title("Accel X")
        self.canvas = FigureCanvasTkAgg(self.figure, graph_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

    def update_plot(self, event=None):
        """Atualiza o gráfico quando o dado selecionado muda."""
        self.current_plot = self.plot_select.get()

        # Atualiza o título do gráfico
        self.ax.set_title(self.current_plot)

        # Atualiza o eixo Y com base no dado selecionado
        if self.current_plot.startswith("Accel"):
            self.ax.set_ylabel("Aceleração (mm/s²)")
        elif self.current_plot == "RPS":
            self.ax.set_ylabel("Frequência (Hz)")
        elif self.current_plot == "ENC":
            self.ax.set_ylabel("mm")

        # Redesenha o gráfico
        self.update_graph()

    def update_graph(self):
        """Atualiza os dados do gráfico."""
        if self.current_plot in self.data:
            values = list(self.data[self.current_plot])
            self.line.set_ydata(values)
            self.line.set_xdata(range(len(values)))

            # Ajusta dinamicamente os limites dos eixos
            if values:
                min_y, max_y = min(values), max(values)
                self.ax.set_ylim(min_y - abs(min_y) * 0.1, max_y + abs(max_y) * 0.1)
            else:
                self.ax.set_ylim(-100, 100)

            self.ax.set_xlim(0, max(100, len(values)))
            self.ax.figure.canvas.draw()

    def update_fields(self, message):
        """Atualiza os valores dos campos com base na mensagem recebida."""
        identifier = message["identifier"]
        payload = message["payload"]

        if identifier == ID_ACC:  # Accel
            accel_x, accel_y, accel_z = struct.unpack("<fff", payload)
            self.labels["Accel X"].config(text=f"{accel_x:.2f}")
            self.labels["Accel Y"].config(text=f"{accel_y:.2f}")
            self.labels["Accel Z"].config(text=f"{accel_z:.2f}")
            self.data["Accel X"].append(accel_x)
            self.data["Accel Y"].append(accel_y)
            self.data["Accel Z"].append(accel_z)
        elif identifier == ID_RPS:  # RPS
            rps = struct.unpack("<H", payload)[0]
            self.labels["RPS"].config(text=f"{rps}")
            self.data["RPS"].append(rps)
        elif identifier == ID_ENC:  # ENC
            enc = struct.unpack("<H", payload)[0]
            self.labels["ENC"].config(text=f"{enc}")
            self.data["ENC"].append(enc)

        # Sempre atualiza o gráfico após processar os dados
        self.update_graph()

    def clear_current_plot(self):
        """Limpa o gráfico atual."""
        if self.current_plot in self.data:
            self.data[self.current_plot].clear()
            self.update_graph()

    def clear_all_plots(self):
        """Limpa todos os gráficos."""
        for key in self.data:
            self.data[key].clear()
        self.update_graph()


    def update_graph(self):
        if self.current_plot in self.data:
            values = list(self.data[self.current_plot])
            self.line.set_ydata(values)
            self.line.set_xdata(range(len(values)))
            if values:
                min_y, max_y = min(values), max(values)
                self.ax.set_ylim(min_y - abs(min_y) * 0.1, max_y + abs(max_y) * 0.1)
            else:
                self.ax.set_ylim(-100, 100)
            self.ax.set_xlim(0, max(100, len(values)))
            self.canvas.draw()

    def connect_serial(self):
        port = self.port_combobox.get()
        try:
            self.serial_connection = serial.Serial(port, 115200, timeout=1)
            self.running = True
            self.status_text.set("Conectado")
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Erro", f"Não foi possível conectar: {e}")

    def disconnect_serial(self):
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        self.status_text.set("Desconectado")
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)

    def read_serial(self):
        while self.running:
            try:
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    self.buffer += data

                    while len(self.buffer) >= 7:  # HEADER_SIZE + CRC_SIZE
                        if self.buffer[0] != PREFIX_DATA:
                            self.buffer = self.buffer[1:]
                            continue

                        _, identifier, payload_size = struct.unpack("BBB", self.buffer[:3])
                        message_length = 3 + payload_size + 2

                        if len(self.buffer) < message_length:
                            break

                        message = self.buffer[:message_length]
                        self.buffer = self.buffer[message_length:]

                        decoded = decode_message(message)
                        if decoded:
                            self.update_fields(decoded)
                            self.update_data(decoded)
                            self.update_graph()
            except Exception as e:
                self.status_text.set("Erro na leitura")

    def send_command(self, command):
        if self.serial_connection and self.serial_connection.is_open:
            payload = command.encode('utf-8')
            buffer = struct.pack("BBB", PREFIX_CMD, ID_CON, len(payload)) + payload
            crc = calculate_crc(buffer)
            buffer += struct.pack("<H", crc)
            self.serial_connection.write(buffer)
        else:
            messagebox.showerror("Erro", "Conexão serial não está aberta. Conecte antes de enviar comandos.")


    def send_start_command(self):
        self.send_command("start")
        self.status_text.set("Lendo dados")

    def send_stop_command(self):
        self.send_command("stop")
        self.status_text.set("Envio de dados parado")

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

    def update_data(self, message):
        identifier = message["identifier"]
        payload = message["payload"]

        if identifier == ID_ACC:  # Accel
            accel_x, accel_y, accel_z = struct.unpack("<fff", payload)
            self.data["Accel X"].append(accel_x)
            self.data["Accel Y"].append(accel_y)
            self.data["Accel Z"].append(accel_z)
        elif identifier == ID_RPS:  # RPS
            rps = struct.unpack("<H", payload)[0]
            self.data["RPS"].append(rps)
        elif identifier == ID_ENC:  # ENC
            enc = struct.unpack("<H", payload)[0]
            self.data["ENC"].append(enc)

if __name__ == "__main__":
    app = SerialApp()
    app.mainloop()
