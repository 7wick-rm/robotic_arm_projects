Firmware simulator (no hardware needed). Re-run it after you change any constant in rail_esp32.ino:

    cd hardware/rail_esp32/sim
    cp ../rail_esp32.ino fw.cpp
    g++ -std=c++17 -O1 -include Arduino.h -I. -o sim sim2.cpp fw.cpp && ./sim

It simulates the carriage (step pulses move it, limit switches trip, AS5600 follows) and checks: no motion at boot, HOME,
MOVE/MOVEPCT accuracy and clamping, STOP during homing and moves, stall detection, FAULT recovery, switch trip.
The mechanism constants at the top of sim2.cpp (COUNTS_PER_STEP, switch positions) are guesses; they do not need to match your rail.
