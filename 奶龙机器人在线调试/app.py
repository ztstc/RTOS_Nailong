import struct
import time
import threading
import serial
import serial.tools.list_ports
from enum import Enum
import logging
import queue
import collections
import tkinter as tk
from tkinter import ttk, scrolledtext

# 设置日志记录
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("uart_communication.log",encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("UARTProtocol")

# 协议定义
PROTOCOL_HEADER = 0xAA55
PROTOCOL_FOOTER = 0x0D0A
HEARTBEAT_INTERVAL_MS = 300
HEARTBEAT_TIMEOUT_MS = 1000
MAX_FRAME_LENGTH = 256

# 命令字定义
class ProtocolCmd(Enum):
    CMD_MOTION_CONTROL = 0x01
    CMD_HEARTBEAT = 0x02
    CMD_HEARTBEAT_ACK = 0x03
    CMD_STATUS_FEEDBACK = 0x04
    CMD_ERROR_REPORT = 0x05
    CMD_REQUEST_STATUS = 0x06
    CMD_SERVO_CONTROL = 0x07
    CMD_REQUEST_MOTOR_SPEED = 0x08  # 新增：电机转速请求命令
    CMD_MOTOR_SPEED_FEEDBACK = 0x09  # 新增：电机转速反馈

# 接收状态机
class RxState(Enum):
    FRAME_HEADER1 = 1
    FRAME_HEADER2 = 2
    FRAME_LENGTH = 3
    FRAME_CMD = 4
    FRAME_DATA = 5
    FRAME_CRC = 6
    FRAME_TAIL1 = 7
    FRAME_TAIL2 = 8

# 运动控制结构
class MotionCmd:
    def __init__(self, target_speedL=0.0, target_speedR=0.0):  # 修改：默认值改为浮点数
        self.target_speedL = float(target_speedL)  # 确保转换为浮点数
        self.target_speedR = float(target_speedR)  # 确保转换为浮点数
    
    def pack(self):
        # 使用'ff'格式打包两个浮点数（4字节每个）
        return struct.pack('<ff', self.target_speedL, self.target_speedR)
    
    @classmethod
    def unpack(cls, data):
        if len(data) != 8:  # 修改：浮点数需要8字节（2*4）
            return None
        target_speedL, target_speedR = struct.unpack('<ff', data)
        return cls(target_speedL, target_speedR)

# 修改send_motion_cmd方法以处理浮点数输入
def send_motion_cmd(self):
    try:
        # 修改：使用float()转换输入值
        speedL = float(self.speedL_entry.get())
        speedR = float(self.speedR_entry.get())
        self.protocol.send_motion_cmd(speedL, speedR)
    except ValueError:
        logger.error("Invalid speed value (must be a number)")
# 舵机控制结构
class ServoCmd:
    def __init__(self, servo_id=0, angle=0):
        self.servo_id = servo_id
        self.angle = angle
    
    def pack(self):
        return struct.pack('<Bh', self.servo_id, self.angle)
    
    @classmethod
    def unpack(cls, data):
        if len(data) != 3:
            return None
        servo_id, angle = struct.unpack('<Bh', data)
        return cls(servo_id, angle)

# 机器人状态信息
class STAT:
    def __init__(self):
        self.Ready = False
        self.motorEN = False
        self.servoEN = False
        self.onlineservoid = [0xFF] * 10
        self.req_servo = [0] * 10
        self.servo = [0] * 10
        self.speedLimit = 0
        self.req_speedL = 0.0
        self.req_speedR = 0.0
        self.speedL = 0.0
        self.speedR = 0.0
        self.batteryVoltage = 0.0
        self.motorCurrentL = 0.0
        self.motorCurrentR = 0.0
        self.motorCurrent = 0.0
        self.CPU = 0
        self.RAM = 0
        self.error_flags = 0
    
    @classmethod
    def unpack(cls, data):
        if len(data) != 69:
            logger.warning(f"STAT.unpack: 数据长度不符，收到 {len(data)} 字节: {data.hex()}")
            return None
        
        # 解析数据
        stat = cls()
        # 前3个字节: bool Ready, motorEN, servoEN
        flags = struct.unpack('<BBB', data[:3])
        stat.Ready = bool(flags[0])
        stat.motorEN = bool(flags[1])
        stat.servoEN = bool(flags[2])
        
        # 接下来30个字节: 3个uint8_t数组，每个10个元素
        idx = 3
        stat.onlineservoid = list(data[idx:idx+10])
        idx += 10
        stat.req_servo = list(data[idx:idx+10])
        idx += 10
        stat.servo = list(data[idx:idx+10])
        idx += 10
        
        # 1字节 speedLimit
        stat.speedLimit = data[idx]
        idx += 1
        
        # 8个float (4字节每个)
        floats = struct.unpack('<ffffffff', data[idx:idx+32])
        idx += 32
        (stat.req_speedL, stat.req_speedR, 
         stat.speedL, stat.speedR, 
         stat.batteryVoltage, 
         stat.motorCurrentL, stat.motorCurrentR, 
         stat.motorCurrent) = floats
        
        # 最后3个字节: CPU, RAM, error_flags
        stat.CPU, stat.RAM, stat.error_flags = struct.unpack('<BBB', data[idx:idx+3])
        
        return stat

    def __str__(self):
        return (f"STAT: Ready={self.Ready}, motorEN={self.motorEN}, servoEN={self.servoEN}\n"
                f"Online Servos: {self.onlineservoid}\n"
                f"Req Servo Angles: {self.req_servo}\n"
                f"Actual Servo Angles: {self.servo}\n"
                f"Speed Limit: {self.speedLimit} RPM\n"
                f"Req Speeds: L={self.req_speedL:.2f}, R={self.req_speedR:.2f} RPM\n"
                f"Actual Speeds: L={self.speedL:.2f}, R={self.speedR:.2f} RPM\n"
                f"Battery: {self.batteryVoltage:.2f} V\n"
                f"Currents: L={self.motorCurrentL:.2f}A, R={self.motorCurrentR:.2f}A, Total={self.motorCurrent:.2f}A\n"
                f"CPU: {self.CPU}%, RAM: {self.RAM}%\n"
                f"Error Flags: 0x{self.error_flags:02X}")

# CRC8计算 (多项式0x07)
def crc8_update(crc, data):
    crc ^= data
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0x07
        else:
            crc <<= 1
        crc &= 0xFF
    return crc

class UARTProtocol:
    def __init__(self, port=None, baudrate=115200, auto_reconnect=True):
        self.port = port
        self.baudrate = baudrate
        self.auto_reconnect = auto_reconnect
        self.serial = None
        self.connected = False
        self.running = False
        self.last_heartbeat_ack = 0
        self.missed_beats = 0
        self.rx_state = RxState.FRAME_HEADER1
        self.rx_buffer = bytearray()
        self.rx_index = 0
        self.expected_length = 0
        self.current_cmd = 0
        self.calculated_crc = 0
        self.receive_thread = None
        self.heartbeat_thread = None
        self.status_queue = queue.Queue()
        self.error_queue = queue.Queue()
        self.heartbeat_event = threading.Event()
        self.lock = threading.Lock()
        self.callbacks = {
            'status': None,
            'error': None,
            'heartbeat_ack': None,
            'connect': None,
            'disconnect': None,
            'motor_speed': None,
            'serial_data': None
        }
        self.serial_data_queue = queue.Queue(maxsize=100)
        
        # 性能统计
        self.stats_lock = threading.Lock()
        self.reset_stats()
    
    def reset_stats(self):
        """重置性能统计"""
        with self.stats_lock:
            # 使用双端队列存储时间戳，自动过期旧数据
            self.tx_timestamps = collections.deque(maxlen=1000)
            self.rx_timestamps = collections.deque(maxlen=1000)
            self.tx_count = 0
            self.rx_count = 0
            self.last_stat_reset = time.time()
    
    def record_tx_frame(self, cmd):
        """记录发送的帧（排除心跳和ACK）"""
        if cmd not in (ProtocolCmd.CMD_HEARTBEAT.value, ProtocolCmd.CMD_HEARTBEAT_ACK.value):
            with self.stats_lock:
                self.tx_timestamps.append(time.time())
                self.tx_count += 1
    
    def record_rx_frame(self, cmd):
        """记录接收的帧（排除心跳和ACK）"""
        if cmd not in (ProtocolCmd.CMD_HEARTBEAT.value, ProtocolCmd.CMD_HEARTBEAT_ACK.value):
            with self.stats_lock:
                self.rx_timestamps.append(time.time())
                self.rx_count += 1
    
    def get_stats(self):
        """获取性能统计"""
        now = time.time()
        with self.stats_lock:
            # 计算最近10秒的帧数
            recent_tx = sum(1 for ts in self.tx_timestamps if now - ts <= 10)
            recent_rx = sum(1 for ts in self.rx_timestamps if now - ts <= 10)
            
            # 计算帧率 (FPS)
            tx_fps = recent_tx / 10.0 if recent_tx > 0 else 0.0
            rx_fps = recent_rx / 10.0 if recent_rx > 0 else 0.0
            
            return {
                "total_tx": self.tx_count,
                "total_rx": self.rx_count,
                "recent_tx": recent_tx,
                "recent_rx": recent_rx,
                "tx_fps": tx_fps,
                "rx_fps": rx_fps
            }
    
    def connect(self, port=None):
        if port:
            self.port = port
        
        if not self.port:
            available_ports = self.find_ports()
            if available_ports:
                self.port = available_ports[1]
                logger.info(f"Auto-selecting port: {self.port}")
            else:
                logger.error("No serial ports found!")
                return False
        
        try:
            # 确保之前没有打开的串口
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01  # 更短的超时时间以提高响应性
            )
            
            if self.serial.is_open:
                self.connected = True
                self.running = True
                self.last_heartbeat_ack = time.time()
                self.missed_beats = 0
                
                # 重置性能统计
                self.reset_stats()
                
                # 启动接收线程
                self.receive_thread = threading.Thread(target=self._receive_task, daemon=True)
                self.receive_thread.start()
                
                # 启动心跳线程
                self.heartbeat_thread = threading.Thread(target=self._heartbeat_task, daemon=True)
                self.heartbeat_thread.start()
                
                logger.info(f"Connected to {self.port} at {self.baudrate} baud")
                
                if self.callbacks['connect']:
                    self.callbacks['connect']()
                
                return True
        
        except Exception as e:
            logger.error(f"Connection failed: {str(e)}")
            self.connected = False
            return False

    def disconnect(self):
        self.running = False
        self.connected = False

        # 关闭串口
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                logger.info("Serial port closed")
            except Exception as e:
                if "FileNotFoundError" not in str(e):
                    logger.error(f"Error closing serial port: {str(e)}")
        self.serial = None

        # 等待接收线程结束
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=0.5)
        # 等待心跳线程结束
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=0.5)
        # 等待重连线程结束
        if hasattr(self, "reconnect_thread") and self.reconnect_thread and self.reconnect_thread.is_alive():
            self.reconnect_thread.join(timeout=2)

        if self.callbacks['disconnect']:
            self.callbacks['disconnect']()
    
    def register_callback(self, event, callback):
        if event in self.callbacks:
            self.callbacks[event] = callback
        else:
            logger.warning(f"Unknown callback event: {event}")

    def send_frame(self, cmd, data=None):
        if not self.connected or not self.serial or not self.serial.is_open:
            logger.warning("Cannot send frame: not connected")
            return False
        
        if data is None:
            data = bytes()
        
        # 构建帧
        frame = bytearray()
        
        # 帧头
        frame.extend(struct.pack('>H', PROTOCOL_HEADER))
        
        # 数据长度
        data_len = len(data)
        frame.append(data_len)
        
        # 命令字
        cmd_value = cmd.value if isinstance(cmd, ProtocolCmd) else cmd
        frame.append(cmd_value)
        
        # 数据域
        frame.extend(data)
        
        # 计算CRC (从长度开始到数据结束)
        crc = 0
        crc = crc8_update(crc, data_len)
        crc = crc8_update(crc, cmd_value)
        for b in data:
            crc = crc8_update(crc, b)
        frame.append(crc)
        
        # 帧尾
        frame.extend(struct.pack('>H', PROTOCOL_FOOTER))
        
        # 发送
        try:
            with self.lock:
                self.serial.write(frame)
            # 记录发送数据
            self._record_serial_data('TX', frame)
            # 记录发送统计（排除心跳和ACK）
            self.record_tx_frame(cmd_value)
            logger.debug(f"Sent frame: cmd={cmd_value:02X}, data={data.hex()}")
            return True
        except Exception as e:
            logger.error(f"Send frame error: {str(e)}")
            self.connected = False
            return False
    
    def send_heartbeat(self):
        return self.send_frame(ProtocolCmd.CMD_HEARTBEAT)
    
    def send_motion_cmd(self, target_speedL, target_speedR):
        cmd = MotionCmd(target_speedL, target_speedR)
        return self.send_frame(ProtocolCmd.CMD_MOTION_CONTROL, cmd.pack())
    
    def send_servo_cmd(self, servo_id, angle):
        if servo_id < 0 or servo_id > 9:
            logger.error(f"Invalid servo ID: {servo_id}. Must be 0-9")
            return False
        cmd = ServoCmd(servo_id, angle)
        return self.send_frame(ProtocolCmd.CMD_SERVO_CONTROL, cmd.pack())
    
    def request_status(self):
        return self.send_frame(ProtocolCmd.CMD_REQUEST_STATUS)
    
    def send_motor_speed_request(self):
        """发送电机转速请求"""
        return self.send_frame(ProtocolCmd.CMD_REQUEST_MOTOR_SPEED)
    
    def find_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]
    
    def _receive_task(self):
        logger.info("Receive task started")
        while self.running and self.connected:
            try:
                data = self.serial.read(MAX_FRAME_LENGTH)
                if not data:
                    time.sleep(0.005)  # 更短的休眠时间以提高响应性
                    continue
                # 记录每次串口接收到的原始数据
                self._record_serial_data('RX', data)
                for byte in data:
                    self._process_rx_byte(byte)
            except serial.SerialException as e:
                logger.error(f"Serial error: {str(e)}")
                self.connected = False
                break
            except Exception as e:
                logger.error(f"Unexpected error in receive task: {str(e)}")
                self.connected = False
                break
        logger.info("Receive task stopped")
        # 自动重连逻辑
        if self.auto_reconnect and self.running:
            logger.warning("Attempting to reconnect...")
            if not hasattr(self, "reconnect_thread") or not (self.reconnect_thread and self.reconnect_thread.is_alive()):
                self.reconnect_thread = threading.Thread(target=self._reconnect_task, daemon=True)
                self.reconnect_thread.start()
    
    def _process_rx_byte(self, byte):
        # 状态机处理
        if self.rx_state == RxState.FRAME_HEADER1:
            if byte == 0xAA:
                self.rx_state = RxState.FRAME_HEADER2
                self.rx_buffer = bytearray()
                self.rx_buffer.append(byte)
        
        elif self.rx_state == RxState.FRAME_HEADER2:
            if byte == 0x55:
                self.rx_state = RxState.FRAME_LENGTH
                self.rx_buffer.append(byte)
            else:
                self.rx_state = RxState.FRAME_HEADER1
                logger.debug("Invalid header2")
        
        elif self.rx_state == RxState.FRAME_LENGTH:
            self.expected_length = byte
            self.calculated_crc = crc8_update(0, byte)
            self.rx_index = 0
            self.rx_buffer.append(byte)
            
            if self.expected_length <= MAX_FRAME_LENGTH - 4:
                self.rx_state = RxState.FRAME_CMD
            else:
                logger.error(f"Frame length error: {self.expected_length}")
                self._handle_protocol_error(0x02)
                self.rx_state = RxState.FRAME_HEADER1
        
        elif self.rx_state == RxState.FRAME_CMD:
            self.current_cmd = byte
            self.calculated_crc = crc8_update(self.calculated_crc, byte)
            self.rx_buffer.append(byte)
            
            if self.expected_length > 0:
                self.rx_state = RxState.FRAME_DATA
            else:
                self.rx_state = RxState.FRAME_CRC
        
        elif self.rx_state == RxState.FRAME_DATA:
            self.rx_buffer.append(byte)
            self.calculated_crc = crc8_update(self.calculated_crc, byte)
            self.rx_index += 1
            
            if self.rx_index >= self.expected_length:
                self.rx_state = RxState.FRAME_CRC
        
        elif self.rx_state == RxState.FRAME_CRC:
            if byte == self.calculated_crc:
                self.rx_buffer.append(byte)
                self.rx_state = RxState.FRAME_TAIL1
            else:
                logger.error(f"CRC error: expected {self.calculated_crc:02X}, got {byte:02X}")
                self._handle_protocol_error(0x01)
                self.rx_state = RxState.FRAME_HEADER1
        
        elif self.rx_state == RxState.FRAME_TAIL1:
            if byte == 0x0D:
                self.rx_buffer.append(byte)
                self.rx_state = RxState.FRAME_TAIL2
            else:
                logger.debug("Invalid tail1")
                self.rx_state = RxState.FRAME_HEADER1
        
        elif self.rx_state == RxState.FRAME_TAIL2:
            if byte == 0x0A:
                self.rx_buffer.append(byte)
                # 完整帧处理
                self._process_frame()
            else:
                logger.debug("Invalid tail2")
            
            self.rx_state = RxState.FRAME_HEADER1
    
    def _process_frame(self):
        cmd = self.current_cmd
        data = self.rx_buffer[4:4+self.expected_length]  # 跳过帧头、长度和命令
        
        logger.debug(f"Received frame: cmd={cmd:02X}, data={data.hex()}")
        
        # 记录接收统计（排除心跳和ACK）
        self.record_rx_frame(cmd)
        
        try:
            if cmd == ProtocolCmd.CMD_HEARTBEAT_ACK.value:
                self.last_heartbeat_ack = time.time()
                self.missed_beats = 0
                self.heartbeat_event.set()
                if self.callbacks['heartbeat_ack']:
                    self.callbacks['heartbeat_ack']()
            
            elif cmd == ProtocolCmd.CMD_STATUS_FEEDBACK.value:
                status = STAT.unpack(data)
                if status:
                    self.status_queue.put(status)
                    if self.callbacks['status']:
                        self.callbacks['status'](status)
                else:
                    logger.warning(f"状态帧解析失败，原始数据: {data.hex()}")
            
            elif cmd == ProtocolCmd.CMD_ERROR_REPORT.value:
                if data:
                    error_code = data[0]
                    self.error_queue.put(error_code)
                    logger.error(f"Device reported error: 0x{error_code:02X}")
                    if self.callbacks['error']:
                        self.callbacks['error'](error_code)

            elif cmd == ProtocolCmd.CMD_MOTOR_SPEED_FEEDBACK.value:
                # 解析电机速度反馈
                if len(data) == 8:
                    try:
                        speedL, speedR = struct.unpack('<ff', data)
                        # 通过回调传递速度数据
                        if self.callbacks['motor_speed']:
                            self.callbacks['motor_speed'](speedL, speedR)
                        logger.info(f"Motor speeds: L={speedL:.2f} RPM, R={speedR:.2f} RPM")
                    except Exception as e:
                        logger.error(f"Error parsing motor speed feedback: {str(e)}")
                else:
                    logger.warning(f"Invalid motor speed feedback length: {len(data)} bytes")
            
            elif cmd == ProtocolCmd.CMD_HEARTBEAT.value:
                # 收到STM32心跳包，不回复ACK（减少不必要的通信）
                pass
            else:
                logger.warning(f"Unhandled command: 0x{cmd:02X}")
        
        except Exception as e:
            logger.error(f"Error processing frame: {str(e)}")
    
    def _heartbeat_task(self):
        logger.info("Heartbeat task started")
        while self.running and self.connected:
            if not self.send_heartbeat():
                break
            self.heartbeat_event.clear()
            if not self.heartbeat_event.wait(HEARTBEAT_TIMEOUT_MS / 1000):
                self.missed_beats += 1
                logger.warning(f"Heartbeat missed ({self.missed_beats})")
                if self.missed_beats >= 3:
                    logger.error("Heartbeat timeout, disconnecting")
                    self.connected = False
                    break
            else:
                self.missed_beats = 0
            time.sleep(HEARTBEAT_INTERVAL_MS / 1000)
        logger.info("Heartbeat task stopped")
        # 自动重连逻辑
        if self.auto_reconnect and self.running:
            if not hasattr(self, "reconnect_thread") or not (self.reconnect_thread and self.reconnect_thread.is_alive()):
                self.reconnect_thread = threading.Thread(target=self._reconnect_task, daemon=True)
                self.reconnect_thread.start()
    
    def _reconnect_task(self):
        logger.warning("Starting reconnect task")
        while self.running and not self.connected:
            if not self.running:
                logger.info("Reconnect task stopped by user.")
                break
            try:
                logger.info("Attempting to reconnect...")
                # 释放串口资源
                if self.serial:
                    try:
                        if self.serial.is_open:
                            self.serial.close()
                    except Exception:
                        pass
                    self.serial = None
                if not self.running:
                    logger.info("Reconnect task stopped by user.")
                    break
                if self.connect():
                    break
                time.sleep(1)
            except Exception as e:
                logger.error(f"Reconnect error: {str(e)}")
                time.sleep(3)
        if self.connected:
            logger.info("Reconnect successful")
        else:
            logger.warning("Reconnect failed")
    
    def _handle_protocol_error(self, error_code):
        self.error_queue.put(error_code)
        if self.callbacks['error']:
            self.callbacks['error'](error_code)

    def _record_serial_data(self, direction, data):
        # direction: 'TX' or 'RX'
        hexstr = ' '.join(f'{b:02X}' for b in data)
        msg = f'{direction}: {hexstr}'
        try:
            self.serial_data_queue.put_nowait(msg)
        except queue.Full:
            pass
        if self.callbacks.get('serial_data'):
            self.callbacks['serial_data'](msg)

    def get_serial_data_history(self, limit=50):
        items = list(self.serial_data_queue.queue)[-limit:]
        return items

# 自定义日志处理器，避免在后台线程中更新UI
class SafeTextHandler(logging.Handler):
    def __init__(self, text_widget, root):
        super().__init__()
        self.text_widget = text_widget
        self.root = root
        self.queue = queue.Queue()
        
        # 启动定时器处理日志消息
        self.root.after(50, self.process_queue)
    
    def emit(self, record):
        msg = self.format(record)
        self.queue.put(msg)
    
    def process_queue(self):
        while not self.queue.empty():
            try:
                msg = self.queue.get_nowait()
                self.text_widget.config(state=tk.NORMAL)
                self.text_widget.insert(tk.END, msg + "\n")
                self.text_widget.see(tk.END)
                self.text_widget.config(state=tk.DISABLED)
            except queue.Empty:
                break
        
        # 继续检查队列
        self.root.after(50, self.process_queue)

class UARTControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Robot Controller/奶龙机器人在线调试")
        self.root.geometry("900x900")
        
        # 创建协议对象
        self.protocol = UARTProtocol(baudrate=115200, auto_reconnect=True)
        self.protocol.register_callback('status', self.on_status_update)
        self.protocol.register_callback('error', self.on_error)
        self.protocol.register_callback('heartbeat_ack', self.on_heartbeat_ack)
        self.protocol.register_callback('connect', self.on_connect)
        self.protocol.register_callback('disconnect', self.on_disconnect)
        self.protocol.register_callback('motor_speed', self.on_motor_speed_update)
        self.protocol.register_callback('serial_data', self.on_serial_data)
        
        # 自动刷新状态标志
        self.auto_refresh_active = False
        
        # 速度自动刷新标志
        self.speed_auto_refresh = False
        self.speed_refresh_rate = 25  # 20Hz = 50ms
        
        # 创建UI
        self.create_widgets()
        
        # 添加日志处理器
        safe_handler = SafeTextHandler(self.log_text, self.root)
        safe_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logger.addHandler(safe_handler)
        
        # 启动连接
        self.connect()
        
        # 启动状态更新
        self.update_status()
        
        # 启动日志更新
        self.update_log()
        
        # 启动性能统计更新
        self.update_perf_stats()
    
    def create_widgets(self):
        # 连接控制框架
        conn_frame = ttk.LabelFrame(self.root, text="连接")
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="端口:").grid(row=0, column=0, padx=5, pady=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        self.port_combo.bind("<Button-1>", self.refresh_ports)
        
        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5)
        self.baud_combo = ttk.Combobox(conn_frame, width=10, values=["9600", "19200", "38400", "57600", "115200","512000"])
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=3, padx=5, pady=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=5)
        self.connect_btn.config(state=tk.NORMAL)
        
        self.status_label = ttk.Label(conn_frame, text="断开连接")
        self.status_label.grid(row=0, column=5, padx=10, pady=5)
        
        # 性能统计框架
        perf_frame = ttk.LabelFrame(self.root, text="性能统计")
        perf_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 第一行：发送统计
        ttk.Label(perf_frame, text="发送帧数:").grid(row=0, column=0, padx=5, pady=2)
        self.tx_total_label = ttk.Label(perf_frame, text="0", width=8)
        self.tx_total_label.grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(perf_frame, text="最近10秒:").grid(row=0, column=2, padx=5, pady=2)
        self.tx_recent_label = ttk.Label(perf_frame, text="0", width=8)
        self.tx_recent_label.grid(row=0, column=3, padx=5, pady=2)
        
        ttk.Label(perf_frame, text="发送帧率:").grid(row=0, column=4, padx=5, pady=2)
        self.tx_fps_label = ttk.Label(perf_frame, text="0.0 fps", width=10)
        self.tx_fps_label.grid(row=0, column=5, padx=5, pady=2)
        
        # 第二行：接收统计
        ttk.Label(perf_frame, text="接收帧数:").grid(row=1, column=0, padx=5, pady=2)
        self.rx_total_label = ttk.Label(perf_frame, text="0", width=8)
        self.rx_total_label.grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(perf_frame, text="最近10秒:").grid(row=1, column=2, padx=5, pady=2)
        self.rx_recent_label = ttk.Label(perf_frame, text="0", width=8)
        self.rx_recent_label.grid(row=1, column=3, padx=5, pady=2)
        
        ttk.Label(perf_frame, text="接收帧率:").grid(row=1, column=4, padx=5, pady=2)
        self.rx_fps_label = ttk.Label(perf_frame, text="0.0 fps", width=10)
        self.rx_fps_label.grid(row=1, column=5, padx=5, pady=2)
        
        # 重置按钮
        ttk.Button(perf_frame, text="重置统计", command=self.reset_stats).grid(row=1, column=6, padx=10, pady=2)
        
        # 运动控制框架
        motion_frame = ttk.LabelFrame(self.root, text="移动控制")
        motion_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(motion_frame, text="Left Speed (RPM):").grid(row=0, column=0, padx=5, pady=5)
        self.speedL_entry = ttk.Entry(motion_frame, width=10)
        self.speedL_entry.grid(row=0, column=1, padx=5, pady=5)
        self.speedL_entry.insert(0, "0")
        
        ttk.Label(motion_frame, text="Right Speed (RPM):").grid(row=0, column=2, padx=5, pady=5)
        self.speedR_entry = ttk.Entry(motion_frame, width=10)
        self.speedR_entry.grid(row=0, column=3, padx=5, pady=5)
        self.speedR_entry.insert(0, "0")
        
        ttk.Button(motion_frame, text="发送", command=self.send_motion_cmd).grid(row=0, column=4, padx=5, pady=5)
        
        # 添加速度显示区域
        speed_display_frame = ttk.Frame(motion_frame)
        speed_display_frame.grid(row=1, column=0, columnspan=5, sticky="we", padx=5, pady=5)
        
        ttk.Label(speed_display_frame, text="实际速度 (RPM):").pack(side=tk.LEFT, padx=(0, 5))
        
        ttk.Label(speed_display_frame, text="左:").pack(side=tk.LEFT)
        self.actual_speedL_label = ttk.Label(speed_display_frame, text="0.00", width=8)
        self.actual_speedL_label.pack(side=tk.LEFT)
        
        ttk.Label(speed_display_frame, text="右:").pack(side=tk.LEFT, padx=(10, 0))
        self.actual_speedR_label = ttk.Label(speed_display_frame, text="0.00", width=8)
        self.actual_speedR_label.pack(side=tk.LEFT)
        
        # 添加速度请求控制
        speed_request_frame = ttk.Frame(motion_frame)
        speed_request_frame.grid(row=2, column=0, columnspan=5, sticky="we", padx=5, pady=5)
        
        self.speed_auto_refresh_var = tk.BooleanVar(value=False)
        self.speed_auto_refresh_btn = ttk.Checkbutton(
            speed_request_frame, 
            text="自动刷新速度 (20Hz)", 
            variable=self.speed_auto_refresh_var,
            command=self.toggle_speed_auto_refresh
        )
        self.speed_auto_refresh_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(speed_request_frame, text="手动请求", command=self.request_motor_speed).pack(side=tk.LEFT, padx=5)
        
        # 舵机控制框架
        servo_frame = ttk.LabelFrame(self.root, text="舵机控制")
        servo_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(servo_frame, text="Servo ID (0-9):").grid(row=0, column=0, padx=5, pady=5)
        self.servo_id_entry = ttk.Entry(servo_frame, width=5)
        self.servo_id_entry.grid(row=0, column=1, padx=5, pady=5)
        self.servo_id_entry.insert(0, "0")
        
        ttk.Label(servo_frame, text="Angle (0-180):").grid(row=0, column=2, padx=5, pady=5)
        self.servo_angle_entry = ttk.Entry(servo_frame, width=5)
        self.servo_angle_entry.grid(row=0, column=3, padx=5, pady=5)
        self.servo_angle_entry.insert(0, "90")
        
        ttk.Button(servo_frame, text="发送", command=self.send_servo_cmd).grid(row=0, column=4, padx=5, pady=5)
        
        # 状态框架
        status_frame = ttk.LabelFrame(self.root, text="状态")
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.status_text = scrolledtext.ScrolledText(status_frame, width=100, height=6)
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.status_text.config(state=tk.DISABLED)
        
        # 自动刷新开关
        auto_refresh_frame = ttk.Frame(status_frame)
        auto_refresh_frame.pack(fill=tk.X, pady=5)
        
        self.auto_refresh_var = tk.BooleanVar(value=False)
        self.auto_refresh_btn = ttk.Checkbutton(
            auto_refresh_frame, 
            text="自动刷新(2Hz)", 
            variable=self.auto_refresh_var,
            command=self.toggle_auto_refresh
        )
        self.auto_refresh_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(auto_refresh_frame, text="手动刷新", command=self.request_status).pack(side=tk.LEFT, padx=5)
        
        # 串口数据框架
        serial_frame = ttk.LabelFrame(self.root, text="串口数据")
        serial_frame.pack(fill=tk.BOTH, padx=10, pady=(5,0))
        self.serial_data_text = scrolledtext.ScrolledText(serial_frame, width=100, height=8)
        self.serial_data_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.serial_data_text.config(state=tk.DISABLED)
        
        # 日志框架
        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.pack(fill=tk.BOTH, padx=10, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, width=100, height=8)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.log_text.config(state=tk.DISABLED)
    
    def reset_stats(self):
        """重置性能统计"""
        self.protocol.reset_stats()
        self.update_perf_stats()
        logger.info("性能统计已重置")
    
    def update_perf_stats(self):
        """更新性能统计显示"""
        stats = self.protocol.get_stats()
        self.tx_total_label.config(text=str(stats["total_tx"]))
        self.tx_recent_label.config(text=str(stats["recent_tx"]))
        self.tx_fps_label.config(text=f"{stats['tx_fps']:.1f} fps")
        
        self.rx_total_label.config(text=str(stats["total_rx"]))
        self.rx_recent_label.config(text=str(stats["recent_rx"]))
        self.rx_fps_label.config(text=f"{stats['rx_fps']:.1f} fps")
        
        # 每秒更新一次
        self.root.after(500, self.update_perf_stats)
    
    def refresh_ports(self, event=None):
        ports = self.protocol.find_ports()
        self.port_combo['values'] = ports
        if ports and self.port_combo.get() not in ports:
            self.port_combo.set(ports[0])
    
    def on_serial_data(self, msg):
        self.serial_data_text.config(state=tk.NORMAL)
        self.serial_data_text.insert(tk.END, msg + "\n")
        self.serial_data_text.see(tk.END)
        self.serial_data_text.config(state=tk.DISABLED)
    
    def on_motor_speed_update(self, speedL, speedR):
        """更新电机速度显示"""
        self.actual_speedL_label.config(text=f"{speedL:.2f}")
        self.actual_speedR_label.config(text=f"{speedR:.2f}")
    
    def request_motor_speed(self):
        """手动请求电机速度"""
        if self.protocol.connected:
            self.protocol.send_motor_speed_request()
    
    def toggle_speed_auto_refresh(self):
        """切换速度自动刷新状态"""
        self.speed_auto_refresh = self.speed_auto_refresh_var.get()
        if self.speed_auto_refresh:
            self.speed_auto_refresh_task()
            logger.info("Motor speed auto refresh enabled (20Hz)")
        else:
            logger.info("Motor speed auto refresh disabled")
    
    def speed_auto_refresh_task(self):
        """自动刷新速度的任务"""
        if self.speed_auto_refresh and self.protocol.connected:
            self.request_motor_speed()
            self.root.after(self.speed_refresh_rate, self.speed_auto_refresh_task)
    
    def toggle_auto_refresh(self):
        self.auto_refresh_active = self.auto_refresh_var.get()
        if self.auto_refresh_active:
            self.request_status()
            self.auto_refresh_status()
            logger.info("Auto refresh enabled")
        else:
            logger.info("Auto refresh disabled")
    
    def auto_refresh_status(self):
        if self.auto_refresh_active and self.protocol.connected:
            self.request_status()
            self.root.after(1000, self.auto_refresh_status)
    
    def toggle_connection(self):
        if self.protocol.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        self.refresh_ports()
        baudrate = int(self.baud_combo.get())
        self.protocol.baudrate = baudrate
        self.protocol.connect()
    
    def disconnect(self):
        self.auto_refresh_active = False
        self.auto_refresh_var.set(False)
        self.speed_auto_refresh = False
        self.speed_auto_refresh_var.set(False)
        self.protocol.disconnect()
    
    def send_motion_cmd(self):
        try:
            speedL = float(self.speedL_entry.get())
            speedR = float(self.speedR_entry.get())
            self.protocol.send_motion_cmd(speedL, speedR)
        except ValueError:
            logger.error("Invalid speed value")
    
    def send_servo_cmd(self):
        try:
            servo_id = int(self.servo_id_entry.get())
            angle = int(self.servo_angle_entry.get())
            if servo_id < 0 or servo_id > 9:
                logger.error("Servo ID must be 0-9")
                return
            if angle < 0 or angle > 180:
                logger.error("Angle must be 0-180")
                return
            self.protocol.send_servo_cmd(servo_id, angle)
        except ValueError:
            logger.error("Invalid servo value")
    
    def request_status(self):
        if self.protocol.connected:
            self.protocol.request_status()
    
    def on_status_update(self, status):
        self.status_text.config(state=tk.NORMAL)
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(tk.END, str(status))
        self.status_text.config(state=tk.DISABLED)
    
    def on_error(self, error_code):
        logger.error(f"Protocol error: 0x{error_code:02X}")
    
    def on_heartbeat_ack(self):
        pass
    
    def on_connect(self):
        self.connect_btn.config(text="断开连接")
        self.status_label.config(text="连接")
        if self.auto_refresh_active:
            self.auto_refresh_status()
        if self.speed_auto_refresh:
            self.speed_auto_refresh_task()
    
    def on_disconnect(self):
        self.connect_btn.config(text="连接")
        self.status_label.config(text="断开连接")
    
    def update_status(self):
        if self.protocol.connected:
            self.status_label.config(text=f"连接中 | 丢失心跳: {self.protocol.missed_beats}")
        else:
            self.status_label.config(text="断开连接")
        self.root.after(50, self.update_status)
    
    def update_log(self):
        """定期更新日志显示（在主线程中执行）"""
        # 定期检查状态更新
        self.root.after(20, self.update_log)
    
    def on_closing(self):
        self.auto_refresh_active = False
        self.speed_auto_refresh = False
        if self.protocol.connected:
            self.protocol.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
