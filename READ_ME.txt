
uCQ_THU_AddOn Projekt

UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
    version 2013 -> 2018 !!!
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION

Dieses Projekt enth�lt mehrere Demos zu Add-On-Boards f�r die uCQ_2018 Platine

Die Demos sind im Prinzip eigenst�ndige Projekte, welche �ber das Konfigurations-
Men� ausgew�hlt werden. Die ausgew�hlte Konfiguration bestimmt, welche Source-
Dateien kompiliert werden.

�ber ein Makro in den Compiler Einstellungen der jeweiligen Konfiguration werden
haupts�chlich in der Header-Datei AddOnBoards.h
- die zur Demo passende Hauptfunktion ausgew�hlt, welche main() quasi ersetzt
- das _XTAL_FREQ Makro definiert (timing f�r LCD, Delay Funktionen, etc.)
- ggf. weitere #define angelegt
- und die erforderlichen Header eingebunden.

Soll eine Demo aus dem Verbund heraus gel�st werden,
+ ermittelt man �ber die ausgew�hlte Konfiguration die erforderlichen Dateien
+ erstellt aus dem aktiven Teil in AddOnBoards.h eine neue Projekt-Header-Datei
+ kopiert den Inhalt der Demofunktion test...() in AddOnBoards.c nach main()
#### WICHTIG: die Dateien AddOnBoards.c/h geh�ren nicht zum neuen Projekt! ####


                                    TODO
================================================================================
- SPI BUSMODE MPU6000 Konfiguration ?
- BMP280spi