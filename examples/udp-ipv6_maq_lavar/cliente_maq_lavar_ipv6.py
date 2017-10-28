import socket
import struct

HOST = '' #all interfaces
UDP_PORT = 8802
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:d70:7"
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:797:6981"
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:aff:5d06"
#UDP_IP = "raspberry-6lbr.local"
#UDP_IP = "localhost"
UDP_IP = "::1"

WASH_MODE_REQUEST = (0x79)
WASH_MODE_RESPONSE = (0x7B)
WASH_SET_MODE = (0x7A)

# socket.SOCK_DGRAM = UDP connection
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) # UDP IPv6
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

#sock.bind((HOST, UDP_PORT))

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

# send WASH_MODE_REQUEST
print "Sending Wash Mode Request..."
pkt_to_send = struct.pack('bb', WASH_MODE_REQUEST, 0)
sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))

print "waiting for response..."
data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
fx, modo_lav_response = struct.unpack('bb', data)
print "resposta recebida fx: ", fx, " valor recedido: ", modo_lav_response

if fx == WASH_MODE_RESPONSE:
    print "Modo lavagem atual: ", modo_lav_response
    Modo_lavagem_atual = modo_lav_response

# mostra menu para usuario
#aguarda usuario escolher uma opcao de modo de lavagem

Modo_lavagem_desejado = 4

# send WASH_SET_MODE
print "Sendind Set Wash Mode: ", Modo_lavagem_desejado
pkt_to_send = struct.pack('bb', WASH_SET_MODE, Modo_lavagem_desejado)
sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))

