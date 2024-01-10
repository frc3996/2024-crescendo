TARGET ?= default
ROBOT_PY := src/robot.py

# Define the default target
default: 
	python3 ${ROBOT_PY}

sync:
	robotpy sync

install:
	robotpy installer install-python

# Rule to generate a target of the same name
%: 
	python3 ${ROBOT_PY} $@

clean:
	find . -iname '*.pyc' -delete
	find . -iname '__pycache__' -delete

.PHONY: default clean download install
poetry add wpilib@latest robotpy-hal@latest robotpy-wpiutil@latest robotpy-wpimath@latest robotpy-wpinet@latest robotpy-halsim-ws@latest robotpy-halsim-gui@latest robotpy-halsim-ds-socket@latest robotpy-cscore@latest robotpy-apriltag@latest
