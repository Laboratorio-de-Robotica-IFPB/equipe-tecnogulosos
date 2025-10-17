#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Inicializa o bloco EV3
ev3 = EV3Brick()

# Inicialização dos motores e sensores
motor_esquerdo = Motor(Port.A)
motor_direito = Motor(Port.D)
motores = DriveBase(motor_esquerdo, motor_direito, 69, 110)

ultrasonic = UltrasonicSensor(Port.S3)
sensor_cor_esquerdo = ColorSensor(Port.S1)
sensor_cor_direito = ColorSensor(Port.S4)

# --- Constantes de calibração ---
PRETO = 5
BRANCO = 35
LIMITE_BRANCO = (BRANCO + PRETO) / 2

# --- Controle PD ---
erro_anterior = 0
ultima_direcao = 1  # 1 = direita, -1 = esquerda

def seguir_linha(leitura_esquerdo, leitura_direito):
    global erro_anterior, ultima_direcao

    # --- GANHOS AJUSTADOS ---
    Kp = 4
    Kd = 1.2

    VELOCIDADE_MAX = 80
    VELOCIDADE_MIN = 20

    desvio_esq = leitura_esquerdo - LIMITE_BRANCO
    desvio_dir = leitura_direito - LIMITE_BRANCO
    erro = desvio_dir - desvio_esq
    derivada = erro - erro_anterior
    correcao = (Kp * erro) + (Kd * derivada)

    # --- GAP DETECTION ---
    if leitura_esquerdo > LIMITE_BRANCO and leitura_direito > LIMITE_BRANCO:
        motores.drive(VELOCIDADE_MAX, 0)
        erro_anterior = 0
        return erro, 0, VELOCIDADE_MAX, VELOCIDADE_MAX

    # --- AMBOS PRETOS (CURVA FECHADA) ---
    if leitura_esquerdo < PRETO + 3 and leitura_direito < PRETO + 3:
        motores.stop()
        motores.drive(0, 200 * ultima_direcao)
        wait(100)
        return erro, 200 * ultima_direcao, 0, 0

    # --- CONTROLE PD NORMAL ---
    erro_abs = abs(erro)

    fator_reducao = min(erro_abs / 3, 1)
    velocidade_linear = VELOCIDADE_MAX - (VELOCIDADE_MAX - VELOCIDADE_MIN) * fator_reducao

    motores.drive(velocidade_linear, -correcao)

    erro_anterior = erro
    ultima_direcao = 1 if correcao < 0 else -1

    return erro, correcao, velocidade_linear - correcao, velocidade_linear + correcao


# --- Detectação e desvio de obstáculos ---
def desviar_obstaculo():
    ev3.speaker.beep()
    wait(20)
    ev3.speaker.beep()

    motores.stop()
    motores.straight(-20)
    motores.turn(90)
    motores.straight(200)
    motores.turn(-90)
    motores.straight(400)
    motores.turn(-90)

    # --- Busca pela linha ---
    ev3.screen.clear()
    ev3.screen.print("Procurando linha...")
    motores.drive(60, 0)

    while True:
        leitura_esq = sensor_cor_esquerdo.reflection()
        leitura_dir = sensor_cor_direito.reflection()
        if leitura_esq < LIMITE_BRANCO or leitura_dir < LIMITE_BRANCO:
            motores.stop()
            break
        wait(50)

    # --- Curva leve pra direita pra se realinhar ---
    motores.drive(0, 100)
    wait(300)
    motores.stop()

    # Retorna ao segue-linha automaticamente


# ----- LOOP PRINCIPAL -----
while True:
    leitura_ultrassonico = ultrasonic.distance()
    leitura_esquerdo = sensor_cor_esquerdo.reflection()
    leitura_direito = sensor_cor_direito.reflection()

    erro, correcao, velocidade_esq, velocidade_dir = seguir_linha(leitura_esquerdo, leitura_direito)

    if leitura_ultrassonico < 65:
        desviar_obstaculo()

    ev3.screen.clear()
    ev3.screen.print("Dist:", round(leitura_ultrassonico))
    ev3.screen.print("Erro:", round(erro, 2))
    ev3.screen.print("V_esq:", int(velocidade_esq))
    ev3.screen.print("V_dir:", int(velocidade_dir))
    ev3.screen.print("L_esq:", int(leitura_esquerdo))
    wait(10)
