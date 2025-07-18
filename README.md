# Toshiba VF-S15 Inverter Control GUI

A PyQt5-based GUI for controlling and monitoring a **Markham M-Tec P20V** mortar pump, with a **Toshiba VF-S15 inverter** connected to an industrial motor system via RS485 (ASCII protocol).

## ⚙️ System Overview
- **Pump**: Markham M-Tec P20V
- **Inverter**: Toshiba VF-S15 (ASCII protocol over RS485)
- **Motor**: VEM Motors GmbH, Model: **K21R 132 M 4**, 4-pole, 4.0 kW, 3-phase, 230/400 V
- **Gear Reducer**: **STM Riduttori** (Italy), Type: **RG075**, Gear Ratio: **10.07:1**
- **Connection**: USB-to-RS485 adapter (19200 baud, Even parity)

## 📊 Monitored Parameters

| Signal           | Source Code Tag | Description                                 |
|------------------|------------------|---------------------------------------------|
| Output Frequency | `RFD00`         | Displayed in Hz (scaled by 0.01)            |
| Output Current   | `RFD03`         | Scaled by inverter-rated current from `RFE70` (percent × rated A) |
| Output Voltage   | `RFD05`         | Scaled by inverter-rated voltage from `RFE71` (percent × rated V) |
| Motor RPM        | —               | Calculated as `RPM = (Freq × 120) / 4`      |
| Shaft RPM        | —               | Calculated as `Motor RPM / 10.07`           |

---

## 🖥️ Features

- Set frequency or RPM
- Start/Stop motor
- Forward/Reverse run toggle
- RS485/manual toggle
- Emergency Stop & Reset
- Real-time monitoring (100 ms interval)
- Logging to CSV (with timestamp and ms precision)

---

## 📁 Project Structure
```
.
├── Pump_Control_v1.py  
├── requirements.txt  
├── logs/  
└── doc/  
    ├── S15_RS485_Communication_Manual.pdf  
    └── Toshiba VFS15 User Manual.pdf  
```


## 📦 Installation

```bash
git clone https://github.com/YOUR_USERNAME/toshiba-vfs15-gui.git
cd toshiba-vfs15-gui
pip install -r requirements.txt


```
## ▶️ Running the GUI
This project requires **Python 3.10+**
```bash
python3 Pump_Control_v1.py
```
- Ensure the inverter is connected via `/dev/ttyUSB0`  
- Modify the serial port in code if using a different device (e.g., `/dev/ttyUSB1` or `COM3` on Windows)

```
## 📁 Logs
CSV log files are saved to the `logs/` directory and include:
- Timestamp  
- Frequency (Hz)  
- Current (A)  
- Voltage (V)  
- Motor RPM  
- Shaft RPM  
