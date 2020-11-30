# CPE301_Swampcooler
Implementation of a swamp cooler using Arduino

Code : src/main.cpp

In order to get an accurate time reading, you must run SyncArduinoClock.pde (./lib/Time-master/examples/Processing / SyncArduinoClock) in a program called Processing. It is a java program that sends the current date and time via serial. If time is not synced, it will begin from 00:00:00 1/1/1970.

Arduino Pin Connections : 

PB6 - Green LED

PB5 - Red LED

PB4 - Yellow LED

PH6 - Blue LED

PF0 - Water Sensor S

PH4 - Temp Sensor S

PK7 - Standby button

PH3 - LCD RS

PE3 - LCD E

PG5 - LCD D4

PE5 - LCD D5

PH5 - LCD D6

PE4 - LCD D7