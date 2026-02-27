# OlinBAD
A BADass Firefighting Drone

## BEFORE LOADING FIRMWARE:
1. Run: `python3 venvStart.py` to create a python virtual environment
2. Start that environment with `source ./venv/bin/activate` (./ being the Firmware directory)
3. Build the dbc intermediate by running `python3 can_api/dbc_generator.py BMS/bms.yml`
3. Build the CAN C files by running `python3 can_api/c_generator.py -y BMS/bms.yml -d out.dbc -c can_api/files/c_templates/can.c -H can_api/files/c_templates/can.h -o BMS`
