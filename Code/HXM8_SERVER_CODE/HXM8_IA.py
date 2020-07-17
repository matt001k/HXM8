#!/usr/bin/env python3

import os
import getch
import sys
import pyfiglet
import colored
import time
from multiprocessing.connection import Client


def title_bar():
    # Clear the terminal screen and displays a title bar
    os.system("clear")

    print("**********************************************************************************************************************************")
    result = pyfiglet.figlet_format('\tHXM8 COMMAND INTERFACE', font = 'slant')
    print(colored.stylize(result, colored.fg("green"), colored.attr("bold")))
    print("**********************************************************************************************************************************")


def list_of_commands():
    red_text = colored.fg("red") + colored.attr("bold")
    print("\nList of commands are as follows: ")
    print(colored.stylize("\n\t-h", red_text) + " or " + colored.stylize("--help", red_text) + "\t\t\t\tdisplay list of commands")
    print(colored.stylize("\tlog", red_text) + "\t\t\t\t\tdisplay logging information of current session")
    print(colored.stylize("\tmode -*mode_option*", red_text) + "\t\t\tsets the mode of HXM8")
    print(colored.stylize("\t\tavailable modes:", colored.attr("bold")))
    print(colored.stylize("\t\t  m", red_text) + "\tmanual mode")
    print(colored.stylize("\t\t  a", red_text) + "\tautomatic(default)")
    print(colored.stylize("\t\t  s", red_text) + "\tsleep mode")
    print(colored.stylize("\thome", red_text) + "\t\t\t\t\tsends HXM8 home")
    print(colored.stylize("\texit", red_text) + "\t\t\t\t\texits server interface")
    print("\nPress enter to exit this screen:")
    command = input("")


def manual_mode():
    print("\nThe following are instructions on commanding the HXM8: ")
    print("\tw/a/s/d\t\tdirection to move HXM8")
    print("\tn\t\t\tautomatic mode and exit interface")
    while(1):
        d = getch.getch()
        if d == "w":
            print("forward")
            conn.send("forward")
        elif d == "a":
            print("left")
            conn.send("left")
        elif d == "s":
            print("backwards")
            conn.send("back")
        elif d == "d":
            print("right")
            conn.send("right")
        elif d == "n":
            conn.send("auto")
            break
        else:
            continue

def open_log_file():
    time_now = time.localtime()
    log_filename = "/root/.HXM8/LOG_FILES/LOG" + str(time_now.tm_year) + str(time_now.tm_mon) + str(time_now.tm_mday) + ".txt"
    print("\n\n" + log_filename + ":\n")
    f = open(log_filename, "r")
    file_contents = f.read()
    print(file_contents)
    f.close()
    print("\n\nPress enter to exit this screen:")
    command = input("")

def command_interface(command):
    if command == "-h" or command == "--help" or command == "-help":
        print("\nHELP MENU")
        list_of_commands()
    elif command == "mode -m":
        title_bar()
        manual_mode()
    elif command == "mode -s":
        conn.send("sleep")
    elif command == "mode -a":
        conn.send("auto")
    elif command == "log":
        open_log_file()
    elif command == "home":
        conn.send("home")
    elif command == "exit":
        conn.send("ui_exit")
        conn.close()
        sys.exit()


def main():
    global conn

    PORT = 6000
    address = ('localhost', PORT)
    conn = Client(address, authkey=b'HXM8_UI_1')

    title_bar()
    list_of_commands()

    while(1):
        title_bar()
        command = input("CMD: ")
        command_interface(command)


if __name__ == "__main__":
    main()
