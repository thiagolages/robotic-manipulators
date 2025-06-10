# Makefile for managing the Docker containers

IMAGE = coppelia_sim_rtp
SHELL := bash
NAME := robotic-manipulators

.PHONY: build up down clean help

.PHONY: help
help: ## Show usage information for this Makefile.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'


build:  ## build the Docker image
	docker build -t thiagolages/robotic-manipulators:latest .

shell: ## Open a shell in the Docker container
	docker exec -it ${NAME} bash

run: ## Run the Docker container (detached mode)
	docker compose up -d --remove-orphans

stop: ## Stop the Docker container
	docker kill ${NAME} || true

clean: ## Clean up Docker containers and images
	docker compose down --volumes --remove-orphans
	docker system prune -f --volumes
