import socket
import struct

HOST = '' #all interfaces
UDP_PORT = 8802
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:d70:7"
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:797:6981"
UDP_IP = "2804:7f4:3b80:c6e8:212:4b00:f28:c486"
#UDP_IP = "raspberry-6lbr.local"
#UDP_IP = "localhost"
#UDP_IP = "::1"

FUNC_MODE_REQUEST = (0x79)
FUNC_MODE_RESPONSE = (0x7B)
FUNC_SET_MODE = (0x7A)

modo_func_atual = 0

# =======================
#      FUNCTIONS
# =======================
def print_menu():
    print(30 * "-", "LIQUIDIFICADOR", 30 * "-")
    print("Modo Atual:", modo_func_atual)
    print("Escolha a opcao desejada:")
    print(" 0 - DESLIGADO")
    print(" 1 - LIGADO")
    print(" 2 - GIRAR DEVAGAR")
    print(" 3 - GIRAR E PARAR")
    print(" 4 - GIRAR BEM RAPIDO COM MUSICA")
    print(" q - SAIR")
    print(74 * "-")

# =======================
#      MAIN
# =======================

# socket.SOCK_DGRAM = UDP connection
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) # UDP IPv6
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

sock.bind((HOST, UDP_PORT))

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

while True:
    # send FUNC_MODE_REQUEST
    print("Sending Func Mode Request...")
    pkt_to_send = struct.pack('bb', FUNC_MODE_REQUEST, 0)
    sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))

    print("waiting for response...")
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    fx, modo_func_response = struct.unpack('bb', data)
    print("resposta recebida fx: ", fx, " valor recebido: ", modo_func_response)

    if fx == FUNC_MODE_RESPONSE:
        print("Modo atual recebido: ", modo_func_response)
        modo_func_atual = modo_func_response

    # mostra menu para usuario
    print_menu()
    #aguarda usuario escolher uma opcao de funcionamento
    opcao = input("Informe a sua opcao: ")

    UpdateMode=True
    if opcao == 0:
        modo_func_desejado = 0
    elif opcao == 1:
        modo_func_desejado = 1
    elif opcao == 2:
        modo_func_desejado = 2
    elif opcao == 3:
        modo_func_desejado = 3
    elif opcao == 4:
        modo_func_desejado = 4
    elif opcao == q:
        print("Finalizando...")
        quit()
    else:
        print "Opcao invalida"
        UpdateMode=False

    if UpdateMode==True:
        # send FUNC_SET_MODE
        print("Sending Set Func Mode: ", modo_func_desejado)
        pkt_to_send = struct.pack('bb', FUNC_SET_MODE, modo_func_desejado)
        sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))
