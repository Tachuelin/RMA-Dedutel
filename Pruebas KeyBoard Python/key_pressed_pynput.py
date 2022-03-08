from pynput import keyboard

def on_press(key):
    try:
        print("Tecla alfanumerica fue presionada: {0} ".format(key.char))
        if key.char == 'w':
            print("Avanza hacia adelante")
        elif key.char == 's':
            print("Avanza hacia atras")
        elif key.char == 'a':
            print("Avanza hacia la izquierda")
        elif key.char == 'd':
            print("Avanza hacia la derecha")
        
    except AttributeError:
        print("Tecla especial fue pulsada: {0} ".format(key))
        if key == key.space:  
            print("Detente")
    
def on_release(key):
    print('Tecla liberada: {0}'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()