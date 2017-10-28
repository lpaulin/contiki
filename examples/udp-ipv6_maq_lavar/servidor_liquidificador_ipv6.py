import socket
import struct
import random

HOST = '' #all interfaces
UDP_PORT = 8802

# sock para IPV6
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) # UDP IPv6
# sock para IPV4
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

sock.bind((HOST, UDP_PORT))

FUNC_MODE_REQUEST = (0x79)
FUNC_MODE_RESPONSE = (0x7B)
FUNC_SET_MODE = (0x7A)

modo_func_atual = 2

while True:
    print("waiting for message...")
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message:", data, "from: [", addr[0].strip(), "]:", addr[1])
    
    fx, fx_data = struct.unpack('bb', data)
    print("fx recebida: ", fx, "fx_data: ", fx_data)

    if fx == FUNC_MODE_REQUEST:
        print("enviando Modo atual: ", modo_func_atual)
        pkt_to_send = struct.pack('bb', FUNC_MODE_RESPONSE, modo_func_atual)
        sock.sendto(pkt_to_send, addr)
    elif fx == FUNC_SET_MODE:
        print("Modo desejado: ", fx_data)
        modo_func_atual = fx_data
