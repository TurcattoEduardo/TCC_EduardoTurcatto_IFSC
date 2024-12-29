import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap import Style
import serial
import threading
import time
import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class SerialApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Sistema de Controle e Monitoramento")
        self.style = Style(theme="superhero")

        # Variáveis para exibir os dados
        self.var_x = ttk.StringVar(value="X: --")
        self.var_y = ttk.StringVar(value="Y: --")
        self.var_z = ttk.StringVar(value="Z: --")
        self.var_rps = ttk.StringVar(value="RPS: --")
        self.var_encsin = ttk.StringVar(value="ENCSIN: --")
        self.selected_data = ttk.StringVar(value="X")  # Dado padrão
        self.connection_status = ttk.StringVar(value="Desconectado")

        # Controle de conexão e dados
        self.running = False
        self.esp = None
        self.data_points = {"X": [], "Y": [], "Z": [], "RPS": [], "ENCSIN": []}
        self.time_points = []

        # Layout da interface
        self.create_widgets()

    def create_widgets(self):
        # Frame principal
        main_frame = ttk.Frame(self.root, padding=20)
        main_frame.pack(fill=BOTH, expand=True)

        # Configurações de conexão
        config_frame = ttk.Labelframe(main_frame, text="Configuração de Conexão", padding=10)
        config_frame.pack(fill=X, padx=10, pady=10)

        ttk.Label(config_frame, text="Porta COM:").grid(row=0, column=0, padx=5, pady=5)
        self.com_port = ttk.Combobox(config_frame, values=self.get_serial_ports(), width=15)
        self.com_port.grid(row=0, column=1, padx=5, pady=5)
        self.com_port.set("COM3")

        ttk.Label(config_frame, text="Baud Rate:").grid(row=0, column=2, padx=5, pady=5)
        self.baud_rate = ttk.Combobox(config_frame, values=["9600", "115200"], width=15)
        self.baud_rate.grid(row=0, column=3, padx=5, pady=5)
        self.baud_rate.set("9600")

        ttk.Button(config_frame, text="Conectar", bootstyle=SUCCESS, command=self.establish_connection).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(config_frame, text="Desconectar", bootstyle=DANGER, command=self.close_connection).grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(config_frame, text="Status:").grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.status_label = ttk.Label(config_frame, textvariable=self.connection_status, bootstyle=INFO, anchor=CENTER, width=20)
        self.status_label.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky=W)

        # Dados em tempo real
        data_frame = ttk.Labelframe(main_frame, text="Dados Recebidos", padding=10)
        data_frame.pack(fill=X, padx=10, pady=10)

        ttk.Label(data_frame, textvariable=self.var_x, font=("Arial", 12)).grid(row=0, column=0, padx=10, pady=5, sticky=W)
        ttk.Label(data_frame, textvariable=self.var_y, font=("Arial", 12)).grid(row=1, column=0, padx=10, pady=5, sticky=W)
        ttk.Label(data_frame, textvariable=self.var_z, font=("Arial", 12)).grid(row=2, column=0, padx=10, pady=5, sticky=W)
        ttk.Label(data_frame, textvariable=self.var_rps, font=("Arial", 12)).grid(row=3, column=0, padx=10, pady=5, sticky=W)
        ttk.Label(data_frame, textvariable=self.var_encsin, font=("Arial", 12)).grid(row=4, column=0, padx=10, pady=5, sticky=W)

        # Seletor de dado e gráfico
        graph_frame = ttk.Labelframe(main_frame, text="Gráfico em Tempo Real", padding=10)
        graph_frame.pack(fill=BOTH, padx=10, pady=10, expand=True)

        ttk.Label(graph_frame, text="Selecionar Dado:").pack(side=TOP, padx=5, pady=5)
        self.data_selector = ttk.Combobox(graph_frame, textvariable=self.selected_data, values=["X", "Y", "Z", "RPS", "ENCSIN"], width=15)
        self.data_selector.pack(side=TOP, padx=5, pady=5)

        self.figure = plt.Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Gráfico em Tempo Real")
        self.ax.set_xlabel("Amostras")
        self.ax.set_ylabel("Valor")
        self.canvas = FigureCanvasTkAgg(self.figure, graph_frame)
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        # Botões de controle
        control_frame = ttk.Frame(main_frame, padding=10)
        control_frame.pack(fill=X, padx=10, pady=10)

        ttk.Button(control_frame, text="Iniciar (START)", bootstyle=PRIMARY, command=lambda: self.send_command("START")).pack(side=LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="Parar (STOP)", bootstyle=SECONDARY, command=lambda: self.send_command("STOP")).pack(side=LEFT, padx=5, pady=5)

    def get_serial_ports(self):
        """Retorna uma lista de portas COM disponíveis."""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def establish_connection(self):
        """Estabelece a conexão serial."""
        try:
            port = self.com_port.get()
            baud = int(self.baud_rate.get())
            self.esp = serial.Serial(port, baud, timeout=1)
            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.update_plot, daemon=True).start()
            self.update_status("Conectado", "success")
        except serial.SerialException as e:
            self.update_status("Erro na conexão", "danger")
            ttk.Messagebox.show_error("Erro", f"Erro ao estabelecer conexão: {e}")

    def close_connection(self):
        """Fecha a conexão serial."""
        if self.esp and self.esp.is_open:
            self.running = False
            self.esp.close()
            self.update_status("Desconectado", "info")

    def update_status(self, status, style):
        """Atualiza o estado de conexão no indicador."""
        self.connection_status.set(status)
        self.status_label.configure(bootstyle=style)

    def read_serial(self):
        """Lê os dados da serial continuamente."""
        while self.running:
            if self.esp and self.esp.in_waiting > 0:
                message = self.esp.readline().decode().strip()
                self.process_message(message)
            time.sleep(0.1)

    def process_message(self, message):
        """Processa as mensagens recebidas."""
        pattern = r"\[(\w+)\] ([^\[]+)\[(\d+)\]"
        match = re.match(pattern, message)
        if match:
            tag = match.group(1)
            data = match.group(2)

            # Atualiza os valores nos mostradores
            if tag == "MPU":
                parts = data.split()
                for part in parts:
                    key, value = part.split('=')
                    if key == "X":
                        self.var_x.set(f"X: {value}")
                        self.data_points["X"].append(float(value))
                    elif key == "Y":
                        self.var_y.set(f"Y: {value}")
                        self.data_points["Y"].append(float(value))
                    elif key == "Z":
                        self.var_z.set(f"Z: {value}")
                        self.data_points["Z"].append(float(value))
            elif tag == "RPS":
                self.var_rps.set(f"RPS: {data}")
                self.data_points["RPS"].append(float(data))
            elif tag == "ENCSIN":
                self.var_encsin.set(f"ENCSIN: {data}")
                self.data_points["ENCSIN"].append(float(data))

    def update_plot(self):
        """Atualiza o gráfico em tempo real."""
        while self.running:
            selected = self.selected_data.get()
            if selected in self.data_points and len(self.data_points[selected]) > 0:
                self.ax.clear()
                self.ax.plot(self.data_points[selected][-50:], label=selected)  # Mostra os últimos 50 pontos
                self.ax.set_title("Gráfico em Tempo Real")
                self.ax.set_xlabel("Amostras")
                self.ax.set_ylabel("Valor")
                self.ax.legend()
                self.canvas.draw()
            time.sleep(1)

    def send_command(self, command):
        """Envia comandos ao ESP32."""
        if self.esp and self.esp.is_open:
            self.esp.write((command + "\n").encode())
            if command == "START":
                self.update_status("Transmitindo...", "primary")
            elif command == "STOP":
                self.update_status("Conectado", "success")
        else:
            ttk.Messagebox.show_error("Erro", "Conexão não estabelecida.")


# Inicializa a interface gráfica
root = ttk.Window(themename="superhero")
app = SerialApp(root)
root.mainloop()
