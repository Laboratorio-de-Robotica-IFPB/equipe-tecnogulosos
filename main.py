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


# --- Constantes para a Lógica de Curva ---
PRETO = 6    # mutavel
BRANCO = 60 # mutavel
LIMITE_BRANCO = ( BRANCO + PRETO )/ 2

#############################################################################################################
""" Estas funções não estão em outro arquivo pois teria que criar objetos"""

#PID padrão
def seguir_linha(leitura_esquerdo, leitura_direito):
    
    """Calcula e aplica a correção para manter o robô na linha.    """

    # --- Constantes de Controle e Movimento ---
    Kp = 5 # mutavel
    VELOCIDADE_BASE = 300 #mutavel
    #--------------------------------------------
    
    erro = leitura_esquerdo - leitura_direito
    correcao = Kp * erro
    
    velocidade_esquerdo = VELOCIDADE_BASE + correcao
    velocidade_direito = VELOCIDADE_BASE - correcao
    
    motor_esquerdo.run(velocidade_esquerdo)
    motor_direito.run(velocidade_direito)

#Detectação de objetos
def desviar_obstaculo():

    """Executa uma manobra completa para desviar de um obstáculo e procurar a linha novamente."""
    motores.stop()
    motores.straight(-30) # Recua para garantir espaço // mutavel
    motores.turn(90)
    motores.straight(200) # mutavel
    motores.turn(-90)
    motores.straight(230) #mutavel
    motores.turn(-90)
    
    # Procura a linha novamente
    while True:
        motores.drive(100, 0) # Anda para frente devagar // mutavel
        if sensor_cor_esquerdo.reflection() <= LIMITE_BRANCO or sensor_cor_direito.reflection() <= LIMITE_BRANCO:
            motores.stop()
            break


"""AS DUAS FUNÇÕES ABAIXO PRECISAM SER REVISADAS"""
#Curva 90° direita
def curva_direita_acentuada():
    
    """Executa uma curva de 90 graus para a direita ao detectar uma linha preta."""
    #todos os numeros são mutaveis 
    motores.straight(60) # Anda para frente para garantir ultrapassagem da curva // mutavel
    motores.drive(0, -70) # Começa a girar no seu proprio eixo ate achar a linha que estava a direita // mutavel
    while True:
        if sensor_cor_direito.reflection() <= LIMITE_BRANCO: 
            motores.stop()
            break

#Curva 90° esquerda
def curva_esquerda_acentuada():
    
    """Executa uma curva de 90 graus para a esquerda ao detectar uma linha preta."""
    motores.straight(60) # Anda para frente para garantir ultrapassagem da curva // mutavel
    motores.drive(0, 70) # # Começa a girar no seu proprio eixo ate achar a linha que estava a esquerda // mutavel
    while True:
        if sensor_cor_esquerdo.reflection() <= LIMITE_BRANCO:
            motores.stop()
            break
#######################################################################################################################

# ----- LOOP PRINCIPAL -----
while True:

    #leitura dos sensores
    leitura_esquerdo = sensor_cor_esquerdo.reflection()
    leitura_direito = sensor_cor_direito.reflection()

    distancia_obstaculo = ultrasonic.distance()

    print( sensor_cor_esquerdo.reflection() )
    print( sensor_cor_direito.reflection() )
    
    if distancia_obstaculo < 500:
        desviar_obstaculo()

    elif ( leitura_esquerdo <= PRETO ) and  ( leitura_direito >= BRANCO ):
        curva_direita_acentuada()

    elif ( leitura_direito <= PRETO ) and ( leitura_esquerdo >= BRANCO):
        curva_esquerda_acentuada()

    else:
        seguir_linha(leitura_esquerdo, leitura_direito)

    # Pequena pausa para o processador
 
