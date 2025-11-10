from machine import Pin, time_pulse_us, disable_irq, enable_irq

start_pin = Pin('GPIO15', Pin.IN)

def handle_start_irq(pin):
    """Handler interrupt per il pin START"""
    state= disable_irq() # Disabilita interrupt
    d1 = time_pulse_us(pin, 0, 20000) # Timeout 20ms
    if 1900 <= d1 <= 16100:
        d2 = time_pulse_us(pin, 0, 10000) # Timeout 10ms
        if 900 <= d2 <= 3100:
            print(f"First duration: {d1} us, Second duration: {d2} us")
        else:
            print("Pulse out of range")
    enable_irq(state) # Riabilita interrupt

start_pin.irq(trigger=Pin.IRQ_FALLING, handler=handle_start_irq )

# Note: The above code sets up an interrupt on GPIO15 to measure the duration of a frame of two low pulses.
# The duration is printed in microseconds when the second pulse ends.

def main():
    """Funzione principale"""
    try:
        while True:
            pass
    except KeyboardInterrupt:
        # CTRL+C in Windows
        print("\nArresto richiesto dall'utente")
    except Exception as e:
        print(f"Errore critico: {e}")
        # Reset del sistema in caso di errore grave
        import machine
        machine.reset()

if __name__ == "__main__":
    main()