# Resumo :

Projeto de Computação Embarcada monofônica.

Desenvolvedores:
- Rafael dos Santos
- João Pedro Meirelles

Professores/Monitores:
- Rafael Corsi ( Professor )
- Marco (do Lab)

Periféricos uC:

    - Parallel Input/Output Controller (PIO)
    - Serial Protocol Interface (SPI)
    - Power Management Controller (PMC)
 
Módulos : 

    - OLED 1 XPLAINED PRO
    - Buzzer

## Ligações

- OLED 1 EXPLAINED PRO -> EXT1
- Buzzer (Negative)    -> PD30
- Buzzer (Positive)    -> +VCC

## Fucionalidades

- OLED botão 1 -> Volta uma música
- OLED botão 2 -> Pausa/Começa a música
- OLED botão 3 -> Avança uma música

- OLED Dispay -> Escreve qual música está tocando no momento (3 músicas carregadas)

- OLED LED (1-3) Dançam de acordo com a mudança de tons, se for o tom aumentar o LED da direita liga, e se diminuir o da esquerda, caso tom se mantenha o LED mantem o que está ligado

- Atmel SAME70 LED D301 -> Caso ligado música está parada, caso desligado música está tocando

## Resultado

https://youtu.be/vTgGX9Sotz8
