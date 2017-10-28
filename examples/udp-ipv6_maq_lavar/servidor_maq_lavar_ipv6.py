import socket
import struct
import random

HOST = '' #all interfaces
UDP_PORT = 8802

# sock para IPV6
sock = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM) # UDP IPv6
# sock para IPV4
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

sock.bind((HOST, UDP_PORT))

WASH_MODE_REQUEST = (0x79)
WASH_MODE_RESPONSE = (0x7B)
WASH_SET_MODE = (0x7A)

Modo_lavagem_atual = 2

while True:
    print "waiting for message..."
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data, "from: [", addr[0].strip(), "]:", addr[1]
    
    fx, fx_data = struct.unpack('bb', data)
    print "fx recebida: ", fx, "fx_data: ", fx_data

    if fx == WASH_MODE_REQUEST:
        print "enviando Modo atual: ", Modo_lavagem_atual
        pkt_to_send = struct.pack('bb', WASH_MODE_RESPONSE, Modo_lavagem_atual)
        sock.sendto(pkt_to_send, addr)
    elif fx == WASH_SET_MODE:
        print "Modo desejado: ", fx_data
        Modo_lavagem_atual = fx_data
