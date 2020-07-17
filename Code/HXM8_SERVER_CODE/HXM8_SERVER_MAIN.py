#!/usr/bin/env python3

import HXM8_TCP_INTERFACE
import threading
import time
from multiprocessing.connection import Listener

def inter_module_process():
    while(1):
        PORT = 6000
        address = ('localhost', PORT)
        listener = Listener(address, authkey=b'HXM8_UI_1')
        conn = listener.accept()
        while(1):
            msg = conn.recv()
            if(msg == 'ui_exit'):
                conn.close()
                listener.close()
                break
            HXM8_TCP_INTERFACE.send_to_hxm8(msg)


def heartbeat_process():
    while(1):
        time.sleep(3)
        HXM8_TCP_INTERFACE.send_to_hxm8("OK")


def main():
    t1 = threading.Thread(target=HXM8_TCP_INTERFACE.tcp_task_start, args="")
    t2 = threading.Thread(target=inter_module_process, args = "")
    t3 = threading.Thread(target=heartbeat_process, args = "")

    t1.start()
    t2.start()
    t3.start()

if __name__ == "__main__":
    main()
