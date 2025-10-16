#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Inicializa o bloco EV3
ev3 = EV3Brick()

# Inicialização dos motores
motor_esquerdo = Motor(Port.A)
motor_direito = Motor(Port.D)
motores = DriveBase(motor_esquerdo, motor_direito, 69, 110)

# Inicialização do ultrassônico
ultrasonic = UltrasonicSensor(Port.S3)

# Inicialização dos sensores de cor
sensor_cor_esquerdo = ColorSensor(Port.S1)
sensor_cor_direito = ColorSensor(Port.S4)

# --- Constantes de calibração ---
PRETO = 5      # mutável
BRANCO = 60    # mutável <----- acho que 60 é muito o sensor ve entre 4 ~ 47
LIMITE_BRANCO = (BRANCO + PRETO) / 2

# --- Variável global para o controle derivativo ---
erro_anterior = 0

#############################################################################################################
""" Estas funções não estão em outro arquivo pois teria que criar objetos """

# PD completo com gap detection
def seguir_linha(leitura_esquerdo, leitura_direito):
    """Controle PD de segue-linha com detecção de gaps e ajuste dinâmico de velocidade."""

    global erro_anterior

    # Constantes de controle
    Kp = 3.7
    Kd = 1.8
    VELOCIDADE_MAX = 100
    VELOCIDADE_MIN = 10

    # --- GAP DETECTION ---
    if leitura_esquerdo > LIMITE_BRANCO and leitura_direito > LIMITE_BRANCO:
        motores.drive(VELOCIDADE_MAX, 0)
        erro_anterior = 0
        return 0, 0, VELOCIDADE_MAX, VELOCIDADE_MAX

    # --- CÁLCULO DO ERRO ---
    desvio_esq = leitura_esquerdo - LIMITE_BRANCO
    desvio_dir = leitura_direito - LIMITE_BRANCO
    erro = desvio_dir - desvio_esq   # linha entre sensores → inversão

    derivada = erro - erro_anterior

    # --- CONTROLE PD ---
    correcao = (Kp * erro) + (Kd * derivada)

    # --- AJUSTE DINÂMICO DE VELOCIDADE ---
    # Reduz velocidade proporcionalmente ao erro
    erro_abs = abs(erro)
    fator_reducao = min(erro_abs / 5, 1)  # normaliza o erro (40 pode ajustar)
    velocidade_linear = VELOCIDADE_MAX - (VELOCIDADE_MAX - VELOCIDADE_MIN) * fator_reducao

    # Aplica no drive base
    motores.drive(velocidade_linear, -correcao)

    # Atualiza erro
    erro_anterior = erro

    # Retorna pra debug
    return erro, correcao, velocidade_linear - correcao, velocidade_linear + correcao


# Detectação de objetos
def desviar_obstaculo():
    """Executa uma manobra completa para desviar de um obstáculo e procurar a linha novamente."""
    
    # aviso sonoro que entrou no modo desvio
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
    motores.straight(200)
    motores.turn(90)


# Curva 90° direita
def curva_direita_acentuada():
    motores.straight(60)
    motores.drive(0, -70)
    while True:
        if sensor_cor_direito.reflection() <= LIMITE_BRANCO:
            motores.stop()
            break

# Curva 90° esquerda
def curva_esquerda_acentuada():
    motores.straight(60)
    motores.drive(0, 70)
    while True:
        if sensor_cor_esquerdo.reflection() <= LIMITE_BRANCO:
            motores.stop()
            break
#######################################################################################################################

# ----- LOOP PRINCIPAL -----
while True:
    leitura_ultrassonico = ultrasonic.distance()
    leitura_esquerdo = sensor_cor_esquerdo.reflection()
    leitura_direito = sensor_cor_direito.reflection()

    erro, correcao, velocidade_esq, velocidade_dir = seguir_linha(leitura_esquerdo, leitura_direito)
    
    if (leitura_ultrassonico < 65):
        desviar_obstaculo()

    # Debug opcional na tela
    ev3.screen.clear()

    ev3.screen.print("Dist:", round(leitura_ultrassonico))
    ev3.screen.print("Erro:", round(erro, 2))
    ev3.screen.print("V_esq:", int(velocidade_esq))
    ev3.screen.print("V_dir:", int(velocidade_dir))
    ev3.screen.print("L_esq:", int(leitura_esquerdo))
    wait(10)

