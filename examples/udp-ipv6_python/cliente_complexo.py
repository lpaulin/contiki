import socket
import struct

UDP_PORT = 8802
#UDP_IP = "aaaa::212:4b00:7c3:b5bb"
UDP_IP = "localhost"

# OPERACOES
OP_MULTIPLY = (0x22)
OP_DIVIDE = (0x23)
OP_SUM = (0x24)
OP_SUBTRACT = (0x25)

PACKET_MASK_TX = 'bb'
PACKET_MASK_RX = 'bb'

# socket.SOCK_DGRAM = UDP connection
#sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) # UDP IPv6
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

# send
OP1 = 1
OP2 = 2
OPERACAO = OP_SUM
FC = 100
print "enviando operacao: ", OPERACAO, " op1: ", OP1, " op2: ", OP2
pkt_to_send = struct.pack(PACKET_MASK_TX, OP1, OPERACAO, OP2, FC)
sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))

print "waiting for response..."
data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
op_result, intpart, fracpart, FPresult, crc = struct.unpack(PACKET_MASK_RX, data)
print "resposta operacao: ", op_result, " intpart: ", intpart, " intpart: ", fracpart, " intpart: ", FPresult, " intpart: ", crc
