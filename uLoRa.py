# Ported By Cyril Anthony (ArduDev - dailyduino.com).
# Source : sandeepmistry - arduino-LoRa

import machine
import time

'''
        self._implicitHeaderMode = False
        self._onTxDone = False
        self._frequency = 0
        self.cs = None
        self.dio0 = None
        self.rst = None
        self.spi = None
        self.pindex = 0

        self.REG_FIFO = 0x00
        self.REG_OP_MODE = 0x01
        self.REG_FRF_MSB = 0x06
        self.REG_FRF_MID = 0x07
        self.REG_FRF_LSB = 0x08
        self.REG_PA_CONFIG = 0x09
        self.REG_OCP = 0x0b
        self.REG_LNA = 0x0c
        self.REG_FIFO_ADDR_PTR = 0x0d
        self.REG_FIFO_TX_BASE_ADDR = 0x0e
        self.REG_FIFO_RX_BASE_ADDR = 0x0f
        self.REG_FIFO_RX_CURRENT_ADDR = 0x10
        self.REG_IRQ_FLAGS = 0x12
        self.REG_RX_NB_BYTES = 0x13
        self.REG_PKT_SNR_VALUE = 0x19
        self.REG_PKT_RSSI_VALUE = 0x1a
        self.REG_RSSI_VALUE = 0x1b
        self.REG_MODEM_CONFIG_1 = 0x1d
        self.REG_MODEM_CONFIG_2 = 0x1e
        self.REG_PREAMBLE_MSB = 0x20
        self.REG_PREAMBLE_LSB = 0x21
        self.REG_PAYLOAD_LENGTH = 0x22
        self.REG_MODEM_CONFIG_3 = 0x26
        self.REG_FREQ_ERROR_MSB = 0x28
        self.REG_FREQ_ERROR_MID = 0x29
        self.REG_FREQ_ERROR_LSB = 0x2a
        self.REG_RSSI_WIDEBAND = 0x2c
        self.REG_DETECTION_OPTIMIZE = 0x31
        self.REG_INVERTIQ = 0x33
        self.REG_DETECTION_THRESHOLD = 0x37
        self.REG_SYNC_WORD = 0x39
        self.REG_INVERTIQ2 = 0x3b
        self.REG_DIO_MAPPING_1 = 0x40
        self.REG_VERSION = 0x42
        self.REG_PA_DAC = 0x4d

        # Modes
        self.MODE_LONG_RANGE_MODE = 0x80
        self.selfMODE_SLEEP = 0x00
        self.MODE_STDBY = 0x01
        self.MODE_TX = 0x03
        self.MODE_RX_CONTINUOUS = 0x05
        self.MODE_RX_SINGLE = 0x06

        # PA config
        self.PA_BOOST = 0x80

        # IRQ masks
        self.IRQ_TX_DONE_MASK = 0x08
        self.IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
        self.IRQ_RX_DONE_MASK = 0x40

        self.RF_MID_BAND_THRESHOLD = 525E6
        self.RSSI_OFFSET_HF_PORT = 157
        self.RSSI_OFFSET_LF_PORT = 164

        self.MAX_PKT_LENGTH = 255
'''
class LoRa:

    _implicitHeaderMode = False
    _onTxDone = False
    _frequency = 0

    cs = None
    dio0 = None
    rst = None
    spi = None
    pindex = 0

    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_PA_CONFIG = 0x09
    REG_OCP = 0x0b
    REG_LNA = 0x0c
    REG_FIFO_ADDR_PTR = 0x0d
    REG_FIFO_TX_BASE_ADDR = 0x0e
    REG_FIFO_RX_BASE_ADDR = 0x0f
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_PKT_SNR_VALUE = 0x19
    REG_PKT_RSSI_VALUE = 0x1a
    REG_RSSI_VALUE = 0x1b
    REG_MODEM_CONFIG_1 = 0x1d
    REG_MODEM_CONFIG_2 = 0x1e
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_PAYLOAD_LENGTH = 0x22
    REG_MODEM_CONFIG_3 = 0x26
    REG_FREQ_ERROR_MSB = 0x28
    REG_FREQ_ERROR_MID = 0x29
    REG_FREQ_ERROR_LSB = 0x2a
    REG_RSSI_WIDEBAND = 0x2c
    REG_DETECTION_OPTIMIZE = 0x31
    REG_INVERTIQ = 0x33
    REG_DETECTION_THRESHOLD = 0x37
    REG_SYNC_WORD = 0x39
    REG_INVERTIQ2 = 0x3b
    REG_DIO_MAPPING_1 = 0x40
    REG_VERSION = 0x42
    REG_PA_DAC = 0x4d

    # Modes
    MODE_LONG_RANGE_MODE = 0x80
    MODE_SLEEP = 0x00
    MODE_STDBY = 0x01
    MODE_TX = 0x03
    MODE_RX_CONTINUOUS = 0x05
    MODE_RX_SINGLE = 0x06

    # PA config
    PA_BOOST = 0x80

    # IRQ masks
    IRQ_TX_DONE_MASK = 0x08
    IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
    IRQ_RX_DONE_MASK = 0x40

    RF_MID_BAND_THRESHOLD = 525E6
    RSSI_OFFSET_HF_PORT = 157
    RSSI_OFFSET_LF_PORT = 164

    MAX_PKT_LENGTH = 255


    def spiWrite(self,reg, data):
        msg = bytearray()
        msg.append(0x00 | reg)
        msg.append(data)
        self.cs.value(0)
        self.spi.write(msg)
        self.cs.value(1)


    def spiRead(self,reg, nbytes=1):
        if nbytes < 1:
            return bytearray()
        msg = bytearray()
        af = reg & 0x7F
        msg.append(af)
        self.cs.value(0)
        self.spi.write(msg)
        data = self.spi.read(nbytes)
        self.cs.value(1)
        return data


    def writeRegister(self,reg, val):
        msg = bytearray()
        msg.append(reg | 0x80)
        msg.append(val)
        self.cs.value(0)
        self.spi.write(msg)
        self.cs.value(1)


    def readRegister(self,reg):
        msg = bytearray()
        msg.append(reg & 0x7F)
        self.cs.value(0)
        self.spi.write(msg)
        self.data = self.spi.read(1)
        self.cs.value(1)
        return self.data[0]


    def sleep(self):
        self.writeRegister(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_SLEEP)


    def idle(self):
        self.writeRegister(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_STDBY)


    def enableCRC(self,en):
        mreg = self.readRegister(self.REG_MODEM_CONFIG_1)
        if en:
            self.writeRegister(self.REG_MODEM_CONFIG_1, mreg | 0x04)
        else:
            self.writeRegister(self.REG_MODEM_CONFIG_1, mreg & 0xFB)


    def setFrequency(self,freq):
        global _frequency
        if freq < 410 or freq > 525:
            return False
        else:
            lfrq = freq * 1000000
            self._frequency = lfrq
            frf = int((lfrq << 19) / 32000000)
            self.writeRegister(self.REG_FRF_MSB, frf >> 16)
            self.writeRegister(self.REG_FRF_MID, (frf >> 8) & 0xFF)
            self.writeRegister(self.REG_FRF_LSB, frf & 0xFF)
            return True


    def LoRaOCP(self,ma):
        ocptrim = 27
        if ma <= 120:
            ocptrim = int((ma - 45) / 5)
        elif ma <= 240:
            ocptrim = int((ma + 30) / 10)
        self.writeRegister(self.REG_OCP, 0x20 | (0x1F & ocptrim))


    def setTxPower(self,power):
        self.writeRegister(0x4D, 0x87)
        self.LoRaOCP(140)
        self.writeRegister(0x09, 0x80 | 0x0C)


    def isTransmitting(self):
        egx = self.readRegister(self.REG_OP_MODE) & self.MODE_TX
        if egx == self.MODE_TX:
            return True
        pmx = self.readRegister(self.REG_IRQ_FLAGS) & self.IRQ_TX_DONE_MASK

        if pmx:
            self.writeRegister(self.REG_IRQ_FLAGS, self.IRQ_TX_DONE_MASK)
        return False


    def explicitHeaderMode(self):
        global _implicitHeaderMode
        _implicitHeaderMode = False
        px1 = self.readRegister(self.REG_MODEM_CONFIG_1) & 0xfe
        self.writeRegister(self.REG_MODEM_CONFIG_1, px1)


    def implicitHeaderMode(self):
        global _implicitHeaderMode
        _implicitHeaderMode = True
        px1 = self.readRegister(self.REG_MODEM_CONFIG_1) & 0x01
        self.writeRegister(self.REG_MODEM_CONFIG_1, px1)


    def beginPacket(self,header=0):
        psm = self.isTransmitting()
        if psm:
            print("Lora is in TX Mode")
            return False
        self.idle()
        if header > 0:
            self.implicitHeaderMode()
        else:
            self.explicitHeaderMode()

        self.writeRegister(self.REG_FIFO_ADDR_PTR, 0)
        self.writeRegister(self.REG_PAYLOAD_LENGTH, 0)

        return True


    def dataPacket(self,buff):
        size = len(buff)
        paylen = self.readRegister(self.REG_PAYLOAD_LENGTH)
        if paylen + size > 255:
            size = 255 - paylen

        for x in buff:
            self.writeRegister(0x00, x)
        self.writeRegister(self.REG_PAYLOAD_LENGTH, paylen + size)
        return size


    def endPacket(self,async1=False):
        #print("Switching to TX mode")
        if async1 == True and _onTxDone == True:
            self.writeRegister(self.REG_DIO_MAPPING_1, 0x40)
        self.writeRegister(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_TX)
        #print("Waiting for TX done...")
        if not async1:
            counter = 0
            while self.readRegister(self.REG_IRQ_FLAGS) & self.IRQ_TX_DONE_MASK == 0:
                counter += 1
                if counter > 100000:  # Add a timeout to prevent infinite loop
                    #print("TX done timeout")
                    break
            self.writeRegister(self.REG_IRQ_FLAGS, self.IRQ_TX_DONE_MASK)
        #print("TX done")
        return True


    def parsePacket(self,size=0):
        irqFlags = self.readRegister(self.REG_IRQ_FLAGS)
        packetLength = 0

        # Clear the IRQ's
        self.writeRegister(self.REG_IRQ_FLAGS, irqFlags)

        if (irqFlags & self.IRQ_RX_DONE_MASK) and (irqFlags & self.IRQ_PAYLOAD_CRC_ERROR_MASK) == 0:
            if self._implicitHeaderMode:
                packetLength = self.readRegister(self.REG_PAYLOAD_LENGTH)
            else:
                packetLength = self.readRegister(self.REG_RX_NB_BYTES)
            self.writeRegister(self.REG_FIFO_ADDR_PTR, self.readRegister(self.REG_FIFO_RX_CURRENT_ADDR))
            self.idle()
        elif self.readRegister(self.REG_OP_MODE) != (self.MODE_LONG_RANGE_MODE | self.MODE_RX_SINGLE):
            self.writeRegister(self.REG_FIFO_ADDR_PTR, 0)
            self.writeRegister(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_RX_SINGLE)

        return packetLength

    def read(self):
        if not available():
            return -1
        global pindex
        pindex += 1
        return self.readRegister(self.REG_FIFO)

    def available(self):
        return self.readRegister(self.REG_RX_NB_BYTES) - pindex

    def readBuffer(self):
        packet_length = self.parsePacket()
        payload = []

        if packet_length > 0:
            for i in range(packet_length):
                payload.append(self.readRegister(self.REG_FIFO))

        return payload


    def receive(self,psize):
        self.writeRegister(self.REG_DIO_MAPPING_1, 0x00)
        if psize > 0:
            self.implicitHeaderMode()
        else:
            self.explicitHeaderMode()
        self.writeRegister(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_RX_CONTINUOUS)
        print("Receiver set to continuous mode")


    def packetRssi(self):
        pgx = self.readRegister(self.REG_PKT_RSSI_VALUE)
        if self._frequency < self.RF_MID_BAND_THRESHOLD:
            return pgx - self.RSSI_OFFSET_LF_PORT
        else:
            return pgx - self.RSSI_OFFSET_HF_PORT


    def packetSnr(self):
        return self.readRegister(self.REG_PKT_SNR_VALUE) * 0.25


    def begin(self,spip, csp, rstp, dio0p, freq):
        global rst, spi, cs, dio0
        self.spi = spip
        self.cs = csp
        self.rst = rstp
        self.dio0 = dio0p
        if rstp is not None:
            self.rst.value(0)
            time.sleep(1)
            self.rst.value(1)
            print("Reset OK")
        ndata = self.readRegister(self.REG_VERSION)
        if ndata != 0x12:
            print("LoRa Module not found!")
            return False
        print("LoRa Module found")
        self.sleep()
        self.setFrequency(freq)
        self.writeRegister(self.REG_FIFO_TX_BASE_ADDR, 0)
        self.writeRegister(self.REG_FIFO_RX_BASE_ADDR, 0)
        erpm = self.readRegister(self.REG_LNA)
        self.writeRegister(self.REG_LNA, erpm | 0x03)
        self.writeRegister(self.REG_MODEM_CONFIG_3, 0x04)
        self.setTxPower(17)
        self.idle()
        print("Frequency:", freq)
        print("Tx Power: 17")
        print("LNA:", self.readRegister(self.REG_LNA))
        print("Mode Config 3:", self.readRegister(self.REG_MODEM_CONFIG_3))
        return True




