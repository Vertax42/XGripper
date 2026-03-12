import serial


class SerialDevice:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        print(f"Serial port {port} init!")
    
    @staticmethod
    def crc16_custom(data: bytes) -> int:
        poly = 0x0007
        crc = 0xFFFF

        for byte in data:
            byte = int(f"{byte:08b}"[::-1], 2)# 输入反转
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ poly) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF

        crc = int(f"{crc:016b}"[::-1], 2)  # 输出反转
        return crc ^ 0x0000        # 异或输出（没变化）

    def build_packet(self, device_id: int, data_bytes: bytes) -> bytes:
        assert len(data_bytes) == 8, "数据域必须8字节"
        header = b'\x3E'
        id_byte = device_id.to_bytes(1, 'little')
        length = (8).to_bytes(1, 'little')
        body = header + id_byte + length + data_bytes
        crc = self.crc16_custom(body).to_bytes(2, 'little')  # 低位在前
        return body + crc
    
    def send(self, data_packed):
        assert len(data_packed) == 13, "full data should be 13 bytes"
        self.ser.write(data_packed)

    def clear_rx_buf(self):
        self.ser.reset_input_buffer()

    def receive(self, expected_len=13):
        resp = self.ser.read(expected_len)

        if len(resp) != expected_len:
            # print(f"接收失败，仅收到 {len(resp)} 字节")
            return None
        # CRC 验证
        crc_calc = self.crc16_custom(resp[:11])
        crc_recv = int.from_bytes(resp[11:], 'little')
        if crc_calc != crc_recv:
            print(f"CRC 校验失败：收到 {crc_recv:04X} ≠ 计算 {crc_calc:04X}")
            self.ser.reset_input_buffer()
            return None
        return resp
    
    def close(self):
        self.ser.close()
        print("serial port close!")