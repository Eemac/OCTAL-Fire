#!/bin/bash
# This is a simple shell script.
echo "Starting OCTAL CAN Library Builder!"

cd "$(dirname "$0")"

# python3 venvStart.py
echo "Activating python venv..."
source ./venv/bin/activate

echo "Building main DBC file..."
python3 can_api/dbc_generator.py x11.dbc BMS/bms_a.yml Core/core.yml Charger/charger.yml -o can_lib.dbc #BMS/bms_a.yml

#Build files for the BMS
echo "Building BMS files..."
python3 -m cantools generate_c_source can_lib.dbc -o ./BMS
python3 can_api/c_generator.py -y BMS/bms_a.yml -d can_lib.dbc -c can_api/files/c_templates/can.c -H can_api/files/c_templates/can.h -o BMS #BMS/bms_a.yml

#Build files for the BMS
echo "Building Core files..."
python3 -m cantools generate_c_source can_lib.dbc -o ./Core
python3 can_api/c_generator.py -y Core/core.yml -d can_lib.dbc -c can_api/files/c_templates/can.c -H can_api/files/c_templates/can.h -o Core #BMS/bms_a.yml