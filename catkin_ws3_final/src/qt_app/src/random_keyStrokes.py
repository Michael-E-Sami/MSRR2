# from curtsies import Input
#
# def main():
#     with Input(keynames='curses') as input_generator:
#         for e in input_generator:
#             print(repr(e))
#
# if __name__ == '__main__':
#     main()

import keyboard  # using module keyboard
while True:  # making a loop
    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('q'):  # if key 'q' is pressed
            print('You Pressed A Key!')
            break  # finishing the loop
    except:
        print("exception")
        break  # if user pressed a key other than the given key the loop will break