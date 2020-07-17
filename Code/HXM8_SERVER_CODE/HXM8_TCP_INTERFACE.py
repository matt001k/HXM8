from socket import*
import binascii 
import unicodedata
import codecs
import time
import threading
import os



def convert_to_command(identifier, data):
    """determine the device that is communicating to the server"""
    if(identifier & 0x4):
        device = 1
    elif(identifier & 0x8):
        device = 2
    elif(identifier & 0x10):
        device = 3
    elif(identifier & 0x20):
        device = 4
    elif(identifier & 0x40):
        device = 5
    elif(identifier & 0x80):
        device = 6
    elif(identifier & 0x100):
        device = 7
    elif(identifier & 0x200):
        device = 8
    elif(identifier & 0x400):
        device = 9
    
    """determine the type of message being sent"""
    if(identifier & 0x1):
        message = "W"
    elif(identifier & 0x2):
        message = "E"
    elif(identifier & 0x3):
        message = "CE"
    else:
        message = "I"

    """configure time locally message was received"""
    time_now = time.localtime()
    
    log_msg = (message + " (" + str(time_now.tm_year) + str(time_now.tm_mon) + str(time_now.tm_mday) + " | " + str(time_now.tm_hour) + str(time_now.tm_min) + ":" + str(time_now.tm_sec) + ") DEVICE " + str(device) + ": " + data + "\n")
    
    #print(log_msg)
    log_filename = "/root/.HXM8/LOG_FILES/LOG" + str(time_now.tm_year) + str(time_now.tm_mon) + str(time_now.tm_mday) + ".txt"
    #print(log_filename)
    f = open(log_filename, "a")
    f.write(log_msg)
    f.close()
 

def tcp_task_start():
    global connectionSocket
    global addr
    global serverSocket

    PORT = 25250
    MSG_RECEIVE = ''

    i = 0
    data_top = 0
    data_bottom = 0
    identifier = 0

    #print('Application Ready')

    serverSocket = socket(AF_INET,SOCK_STREAM)
    serverSocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serverSocket.bind(('',PORT))
    serverSocket.listen(1)
    (connectionSocket, addr) = serverSocket.accept()


    while(1):
        try:
            MSG_RECEIVE = connectionSocket.recv(1024) #wait for device to send messages to server
            #MSG_RECEIVE = int(MSG_RECEIVE)
            MSG_RECEIVE = binascii.hexlify(MSG_RECEIVE).decode() #turn message to proper hexadecimal format
            MSG_RECEIVE = bytearray.fromhex(MSG_RECEIVE)    #turn message into a byte array
            MSG_RECEIVE.reverse()   #reverse the message format
            identifier = MSG_RECEIVE[8:]
            data = [MSG_RECEIVE[4:8], MSG_RECEIVE[:4]]
            data[1].reverse()
            data[0].reverse()
            data[1] = data[1].decode()
            data[0] = data[0].decode()
            data = ''.join(data)
            identifier = int(binascii.hexlify(identifier).decode(), 16) #turn the message into an integer hexadecimal form

            
            convert_to_command(identifier, data)#used to convert into text that will be displayed on the terminal and written to a log file.

            response = 'RECEIVED'

            connectionSocket.send(response.encode())

            i+=1

            if((i > 2) or (data_top > 0xFFFFFFFF)):
                i = 0
                identifier = 0
                data_top = 0
                data_bottom = 0
        except:
            await_tcp_conn()


def send_to_hxm8(command):
    try:
        connectionSocket.send(command.encode())
    except:
        print("No connection is currently established with HXM8")
        return False
    return True


def await_tcp_conn():
    global connectionSocket

    print("connection lost with HXM8")

    connectionSocket.close()
    serverSocket.listen(1)
    (connectionSocket, addr) = serverSocket.accept()

