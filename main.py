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
motores = DriveBase( Motor(Port.A) , Motor(Port.D) , 69 , 110 )

# Inicialização do ultrasonico
ultrasonic = UltrasonicSensor(Port.S4)

# Inicialização dos sensores de cor
sensor_cor_esquerdo = ColorSensor(Port.S3)
sensor_cor_direita = ColorSensor(Port.S1)

# --- Constantes de Controle e Movimento ---
# É crucial começar com valores mais baixos para estabilidade
Kp = 1.0
VELOCIDADE_BASE = 80
# --- Constantes para a Lógica de Curva ---
PRETO = 17
BRANCO = 73
LIMITE_BRANCO = BRANCO + PRETO / 2
VELOCIDADE_GIRO = 60
LIMITE_ULTRASONIC = 8

#if sensor_cor_esquerdo.reflection() <= PRETO:
motores.straight(200)
motores.drive( 0 ,-25)
while True:
    ev3.speaker.beep(1 , 1)
'''while sensor_cor_direita.reflection() <= PRETO:
    motores.stop()
    break


#if sensor_cor_direita.reflection() <= PRETO:
motores.straight(200)
motores.drive( 0 , 25 )
while sensor_cor_esquerdo.reflection() <= PRETO:
    motores.stop()
    break
'''
# ----- LOOP PRINCIPAL -----
'''while True:

    # 0. Dectectação de objetos 
    if ultrasonic.distance() < 100:
        motores.straight(-20)
        motores.turn(90)
        motores.straight(100)
        motores.turn(-90)
        motores.straight(120)
        motores.turn(-90)
        while True:
            motores.drive(50, 0)
            if sensor_cor_esquerdo.reflection() <= LIMITE_BRANCO or sensor_cor_direita.reflection() <= LIMITE_BRANCO:
                break


    if sensor_cor_esquerdo.reflection() <= PRETO:
        motores.straight(20)
            motores.turn(-1)
            if sensor_cor_direita.reflection() <= PRETO:
                break


    if sensor_cor_direita.reflection() <= PRETO:
        motores.straight(20)
        while True:
            motores.turn(1)
            if sensor_cor_esquerda.reflection() <= PRETO:
                break
    

        

    #MODO NORMAL / SEGUIR LINHA
    # 2. Calcular o Erro
    erro = leitura_esquerda - leitura_direita
        
    # 4. Calcular a Correção Proporcional
    correcao = Kp * erro
        
    # 5. Aplicar a correção aos motores
    velocidade_esquerdo = VELOCIDADE_BASE + correcao
    velocidade_direito = VELOCIDADE_BASE - correcao
        
    motor_esquerdo.run(velocidade_esquerdo)
    motor_direito.run(velocidade_direito)
    
    # Pequena pausa para o processador
    wait(10)
'''