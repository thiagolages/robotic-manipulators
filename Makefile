# Makefile for managing the Docker containers

IMAGE = coppelia_sim_rtp
SHELL := bash
NAME := robotic-manipulators

.PHONY: build up down clean

build:  ## build the Docker image
	docker build -t thiagolages/robotic-manipulators:latest .

shell:
	docker exec -it ${NAME} bash

run:
	docker compose up -d --remove-orphans

stop:
	docker kill ${NAME} || true

clean:
	docker compose down --volumes --remove-orphans
	docker system prune -f --volumes
