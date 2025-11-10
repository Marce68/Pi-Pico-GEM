# This example demonstrates a UART periperhal.

import bluetooth
import random
import struct
import time
from ble_advertising import advertising_payload

from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_FLAG_INDICATE = const(0x0020)

# Xpress Streaming Service
_XPRESS_UUID = bluetooth.UUID("331a36f5-2459-45ea-9d95-6142f0c4b307")
_XPRESS_TX = (
    bluetooth.UUID("a73e9a10-628f-4494-a099-12efaf72258f"),
    _FLAG_NOTIFY | _FLAG_INDICATE,
)
_XPRESS_RX = (
    bluetooth.UUID("a9da6040-0823-4995-94ec-9ce41ca28833"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_XPRESS_MODE = (
    bluetooth.UUID("75a9f022-af03-4e41-b4bc-9de90a47d50b"),
    _FLAG_READ | _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE | _FLAG_NOTIFY | _FLAG_INDICATE,

)
_XPRESS_SERVICE = (
    _XPRESS_UUID,
    (_XPRESS_RX, _XPRESS_TX, _XPRESS_MODE),
)


class BLESimplePeripheral:
    def __init__(self, ble, name="TOOA-pi"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_rx, self._handle_tx, self._handle_mode),) = self._ble.gatts_register_services((_XPRESS_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[_XPRESS_UUID])
        self._advertise()
        self._ble.gatts_set_buffer(self._handle_rx, 250, True)

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)
            self._ble.gap_advertise(interval_us=500000, adv_data=self._payload)  # Test
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)

    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def on_write(self, callback):
        self._write_callback = callback


def demo():
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)

    def on_rx(v):
        print("RX", v)

    p.on_write(on_rx)

    i = 0
    while True:
        if p.is_connected():
            # Short burst of queued notifications.
            for _ in range(3):
                data = str(i) + "_"
                print("TX", data)
                p.send(data)
                i += 1
        time.sleep_ms(1000)


if __name__ == "__main__":
    demo()