# Bras-robotise-stm32
Projet d'asservissement de vitesse et de position d'un bras robotisé avec STM32

Ce projet consiste à concevoir et implémenter un système de commande pour un moteur à courant continu (MCC) pilotant un axe de bras robotisé. Le système inclut une boucle de régulation de vitesse et de position, exploitant les retours d’un encodeur et d’une génératrice tachymétrique.

## 🎯 Objectifs
- Contrôler la vitesse et la position d’un moteur CC.
- Utiliser une carte STM32 pour le traitement et le pilotage.
- Lire les retours capteurs (encodeur + génératrice tachymétrique).
- Visualiser les données en temps réel (écran LCD).

## ⚙️ Technologies & Matériel utilisés
- Carte STM32 (NUCLEO-L476RG)
- Module IHM08M1 (driver moteur)
- Moteur CC avec génératrice tachymétrique
- Encodeur quadrature (ChA / ChB)
- Écran LCD 16x2 ou interface UART/USB
- IDE STM32CubeIDE + STM32CubeMX
- Langage : C

## 🧠 Fonctionnalités principales
- Génération de signaux PWM pour commande du moteur
- Acquisition de la vitesse par la génératrice tachymétrique
- Lecture de la position via l’encodeur
- Régulation PID de la vitesse et/ou de la position
- Affichage en temps réel sur LCD

## 🖼️ Schéma de principe

![Montage](schém_fonct_projet)
