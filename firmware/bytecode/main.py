# Il file main.mpy originale va rinominato ad esempio start.mpy
import start

if hasattr(start, 'main'):
    start.main()
else:
    print("Errore: main() non trovata in start.mpy")