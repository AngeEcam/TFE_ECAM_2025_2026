# TFE_ECAM_2025_2026

## Développement et intégration d’un système de monitoring environnemental pour un local pilote hospitalier

## Contexte du projet

Ce travail de fin d’études, réalisé au sein du Service Biomédical de l’Hôpital Militaire Reine Astrid (HMRA), porte sur le développement d’un système de monitoring environnemental dédié à la surveillance de différents paramètres (température, humidité et pression) dans des environnements hospitaliers.

Le point de départ de cette recherche est le constat des limites de la solution actuelle (JRI-MySirius), notamment :
- une communication difficile avec le fournisseur,
- une dépendance à une solution externe ne permettant pas une intervention directe ni une modification du système par le service,
- une instabilité du signal radio due à l’augmentation du bruit électromagnétique généré par la multiplication des équipements électriques au sein de l’hôpital.

L’objectif est de proposer une architecture plus robuste capable de garantir l’intégrité des données sans perte d’information, même en cas de coupure réseau ou électrique.

L’architecture proposée repose sur un découplage entre l’acquisition et la transmission.

L’acquisition locale est assurée par un module Nordic nRF communiquant avec les capteurs via un bus RS-485 (Modbus RTU).  
Ce choix se justifie par :
- une excellente immunité aux perturbations électromagnétiques,
- une grande fiabilité sur de longues distances,

ce qui le rend particulièrement adapté lorsque les unités à surveiller sont éloignées les unes des autres.

La transmission des données vers l’infrastructure hospitalière s’effectue via une passerelle ESP32 reliée en Ethernet.  
La liaison entre le module Nordic et l’ESP32 est assurée par Bluetooth Low Energy (BLE) ou en UART, permettant d’isoler la couche capteurs de la couche réseau.
