from machine import Pin
import os

start_pin = Pin('GPIO15', Pin.IN)

def handle_start_irq(pin):
    duration = machine.time_pulse_us(pin, 0, 1000000) # Timeout 1s
    print(f"Pulse duration: {duration} us")

start_pin.irq(trigger=Pin.IRQ_FALLING, handler=handle_start_irq )

# Note: The above code sets up an interrupt on GPIO15 to measure the duration of a low pulse.
# The duration is printed in microseconds when the pulse ends.

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