import serial

# Configurações da porta serial (ajuste conforme necessário)
porta_serial = "/dev/ttyUSB0"  # Substitua pelo nome da sua porta serial
baud_rate = 9600
data_bits = 8
paridade = serial.PARITY_NONE
stop_bits = 1

# Função para calcular o checksum XOR
def calcular_checksum(mensagem):
    checksum = 0
    for byte in mensagem:
        checksum ^= byte
    return checksum

# Função para aplicar byte stuffing
def byte_stuffing(mensagem):
    bytes_codificados = []
    for byte in mensagem:
        if byte in [0x01, 0x04, 0x10, 0x11, 0x13]:
            bytes_codificados.extend([0x10, byte ^ 0x20])
        else:
            bytes_codificados.append(byte)
    return bytes_codificados

# Exemplo de envio de string
def enviar_string(serial_port, mensagem):
    try:
        with serial.Serial(porta_serial, baud_rate, timeout=1) as ser:
            # Calcula o checksum
            checksum = calcular_checksum(mensagem)
            mensagem_com_checksum = mensagem + bytes([checksum])

            # Aplica byte stuffing
            mensagem_codificada = byte_stuffing(mensagem_com_checksum)

            # Envia a mensagem
            ser.write(mensagem_codificada)

            # Aguarda resposta
            resposta = ser.read(1)
            if resposta == b'\x06':
                print("ACK recebido. Comunicação bem-sucedida!")
            elif resposta == b'\x05':
                print("NACK recebido. Houve um erro na comunicação.")
            else:
                print("Resposta desconhecida:", resposta.hex())

    except serial.SerialException:
        print("Erro ao abrir a porta serial.")

# Exemplo de uso
if __name__ == "__main__":
    mensagem_para_periferico = b"\x01\xAC\x33\xCB\xD3\x6A\x00\x20\x10\x21\x10\x21\x3D\x8E\x04"
    enviar_string(porta_serial, mensagem_para_periferico)
