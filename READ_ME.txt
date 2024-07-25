
uCQ_THU_AddOn Projekt

UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
    version 2013 -> 2018 !!!
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION
UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION   UNDER CONSTRUCTION

Dieses Projekt enthält mehrere Demos zu Add-On-Boards für die uCQ_2018 Platine

Die Demos sind im Prinzip eigenständige Projekte, welche über das Konfigurations-
Menü ausgewählt werden. Die ausgewählte Konfiguration bestimmt, welche Source-
Dateien kompiliert werden.

Über ein Makro in den Compiler Einstellungen der jeweiligen Konfiguration werden
hauptsächlich in der Header-Datei AddOnBoards.h
- die zur Demo passende Hauptfunktion ausgewählt, welche main() quasi ersetzt
- das _XTAL_FREQ Makro definiert (timing für LCD, Delay Funktionen, etc.)
- ggf. weitere #define angelegt
- und die erforderlichen Header eingebunden.

Soll eine Demo aus dem Verbund heraus gelöst werden,
+ ermittelt man über die ausgewählte Konfiguration die erforderlichen Dateien
+ erstellt aus dem aktiven Teil in AddOnBoards.h eine neue Projekt-Header-Datei
+ kopiert den Inhalt der Demofunktion test...() in AddOnBoards.c nach main()
#### WICHTIG: die Dateien AddOnBoards.c/h gehören nicht zum neuen Projekt! ####


                                    TODO
================================================================================
- SPI BUSMODE MPU6000 Konfiguration ?
- BMP280spi