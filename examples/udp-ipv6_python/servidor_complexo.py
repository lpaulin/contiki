import socket
import struct
import random

HOST = '' #all interfaces
UDP_PORT = 8802

# sock para IPV6
#sock = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM) # UDP IPv6
# sock para IPV4
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

sock.bind((HOST, UDP_PORT))

LED_TOGGLE_REQUEST = (0x79)
LED_SET_STATE = (0x7A)
LED_GET_STATE = (0x7B)
LED_STATE = (0x7C)

LED_CURRENT_STATE = 2

while True:
    print "waiting for message..."
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data, "from: [", addr[0].strip(), "]:", addr[1]
    offset = 0
    
    #op = struct.unpack_from(">B", data, offset)
    #offset += struct.calcsize(">B")

    fx, led_current_state = struct.unpack('bb', data)
    print "fx: ", fx, " estado: ", led_current_state

    if fx == LED_TOGGLE_REQUEST:
        print "enviando resposta: ", fx, " estado: ", led_current_state
        pkt_to_send = struct.pack('bb', LED_SET_STATE, LED_CURRENT_STATE)
        sock.sendto(pkt_to_send, addr)
