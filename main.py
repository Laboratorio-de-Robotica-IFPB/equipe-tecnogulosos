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
ultrasonic = UltrasonicSensor(Port.S3)

# Inicialização dos sensores de cor
sensor_cor_esquerdo = ColorSensor(Port.S1)
sensor_cor_direito = ColorSensor(Port.S4)

# --- Constantes de Controle e Movimento ---
Kp = 5
VELOCIDADE_BASE = 300

# --- Constantes para a Lógica de Curva ---
PRETO = 4
BRANCO = 26
LIMITE_BRANCO = ( BRANCO + PRETO )/ 2

# ----- LOOP PRINCIPAL -----
while True:

    leitura_esquerdo = sensor_cor_esquerdo.reflection()
    leitura_direito = sensor_cor_direito.reflection()

    print( sensor_cor_esquerdo.reflection()  )
    print( sensor_cor_direito.reflection() )

    if ultrasonic.distance() < 100:
        motores.straight(-20)
        motores.turn(90)
        motores.straight( 300 )
        motores.turn(-90)
        motores.straight(320)
        motores.turn(-90)
        while True:
            motores.drive(50, 0)
            if sensor_cor_esquerdo.reflection() <= LIMITE_BRANCO or sensor_cor_direito.reflection() <= LIMITE_BRANCO:
                motores.stop()
                break

    print("leiteura direito" ,leitura_direito)
    print("leiteura esquerdo" ,leitura_esquerdo)

    if (sensor_cor_esquerdo.reflection() <= PRETO) and  (sensor_cor_direito.reflection() >= BRANCO):
        motores.straight( 60 )
        motores.drive( 0 , -70 )
        while True:
            if  sensor_cor_direito.reflection() <=  ( LIMITE_BRANCO - 1 ): 
                motores.stop()
                break

    if (sensor_cor_direito.reflection() <= PRETO ) and (sensor_cor_esquerdo.reflection() >= BRANCO):
        motores.straight( 60 )
        motores.drive( 0 , 70 )
        while True:
            if  sensor_cor_esquerdo.reflection() <= (  LIMITE_BRANCO - 11 ): 
                motores.stop()
                break

    #MODO NORMAL / SEGUIR LINHA
    # 2. Calcular o Erro
    erro = leitura_esquerdo - leitura_direito
        
    # 4. Calcular a Correção Proporcional
    correcao = Kp * erro
        
    # 5. Aplicar a correção aos motores
    velocidade_esquerdo = VELOCIDADE_BASE + correcao
    velocidade_direito = VELOCIDADE_BASE - correcao
        
    motor_esquerdo.run(velocidade_esquerdo)
    motor_direito.run(velocidade_direito)

    # Pequena pausa para o processador
    wait(10)

    # 0. Dectectação de objetos 