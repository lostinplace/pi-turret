pi-turret: src/*
	gcc -std=c99 -Wall -o pi-turret src/* -lwiringPi -lm
