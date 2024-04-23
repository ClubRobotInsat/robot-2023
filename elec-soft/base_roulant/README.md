# Base roulant CDF 2024
Auteur : Triet Nguyen (trietngh)

Le code dans ce dossier permet de contrôler un (plusieur) moteur DC avec un (plusieur) encodeur à partir du bus CAN.

## Table of Contents
- [To Do](#to-do)
- [Fonctionnement de la base roulant](#fonctionnement-de-la-base-roulant)
    - [Contrôle de vitesse (Boucle de régulation dans STM)](#contrôle-de-vitesse-boucle-de-régulation-dans-stm)
    - [Contrôle de PWM (Boucle de régulation dans Raspi)](#contrôle-de-pwm-boucle-de-régulation-dans-raspi)
    - [Contrôle de distance](#contrôle-de-distance)
- [Fonction à utiliser](#fonction-à-utiliser)
    - [BR_init(...)](#br_init)
    - [BR_startAllMotors()](#br_startallmotors)
    - [BR_stopAllMotors()](#br_stopallmotors)
    - [BR_setDirection(motor_ID, direction)](#br_setdirectionmotor_id-direction)
    - [BR_setSpeed(motor_ID, speed)](#br_setspeedmotor_id-speed)
    - [BR_setPWM(motor_ID, pwm)](#br_setpwmmotor_id-pwm)
    - [BR_setDirection(motor_ID)](#br_setdirectionmotor_id)
    - [BR_getSpeed(motor_ID)](#br_getspeedmotor_id)
    - [BR_getPWM(motor_ID)](#br_getpwmmotor_id)

## To Do
- [ ] Function to set targetSpeed
- [ ] Boucle de régulation dans STM (avec l'interruption du SysTick)   

## Fonctionnement de la base roulant
Il y a trois façons possible pour contrôler la base roulant. 
1. Contrôle de vitesse
2. Contrôle de PWM
3. Contrôle de distance

### Contrôle de vitesse (Boucle de régulation dans STM)
Dans ce cas, la Raspi donne la vitesse cible comme consigne et le STM se débrouille pour atteindre cette vitesse.
Voici un schéma bloc pour illustrer le fonctionnement :
[Schema_avec_regulation](fonctionnement_base_roulant_avec_regulation_vitesse.png)

### Contrôle de PWM (Boucle de régulation dans Raspi)
Dans ce cas, la Raspi donne une PWM pour commander le moteur, le STM fait aucune régulation. La régulation est fait par la Raspi.
Voici le schéma bloc du fonctionnement :
[Schema_sans_regulation](fonctionnement_base_roulant_sans_regulation_vitesse.png)

### Contrôle de distance
Dans ce cas, la Raspi demande la base roulant à faire une distance. Le STM se débrouille pour le faire.
Ce fonctionnement nécessite une modification de la configuration de l'encodeur.
A vérifier avec @tanghuongcam

## Fonction à utiliser
### *BR_init(...)*
Initialisation du moteur et encoder. A faire à chaque démarrage

### *BR_startAllMotors()*

### *BR_stopAllMotors()*

### *BR_setDirection(motor_ID, direction)*

### *BR_setSpeed(motor_ID, speed)*

### *BR_setPWM(motor_ID, pwm)*

### *BR_setDirection(motor_ID)*

### *BR_getSpeed(motor_ID)*

### *BR_getPWM(motor_ID)*