import keyboard

while True:
    if keyboard.read_key() == "w":
        print ("Avanza hacia adelante")
        #break
    elif keyboard.read_key() == "s":
        print ("Avanza hacia atras")
        #break
    elif keyboard.read_key() == "a":
        print ("Avanza hacia la izquierda")
        #break
    elif keyboard.read_key() == "d":
        print ("Avanza hacia la derecha")
        #break

# while True:
#     if keyboard.is_pressed("q"):
#         print ("You pressed q")
#         break

# keyboard.on_press_key("r", lambda _:print("You pressed r"))