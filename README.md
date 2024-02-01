# HP_Server_PSU_DPS1200FB_HSTNS-FD19_Controller

Arduino based i2c controller with status display and USB serial link output.
Should work with any similar HP Server PSU. See Docs folder for circuit.
Functions include:
  - power on/off control depending on USB power being present
  - current trip with level selected with rotary encoder
  - rotary push button to select displayed status register (Voltages, currents etc)
  - rotatry push button to reset current trip
  - seperate switch to enable PSU (when USB powered up)

![20240201_133005](https://github.com/Rich5252/HP_Server_PSU_DPS1200FB_HSTNS-FD19_Controller/assets/144813773/7b092a99-4d16-423b-9797-aafd188c382c)

Derived from: https://github.com/raplin/DPS-1200FB
Lots of other useful info here: https://github.com/slundell/dps_charger

73 - G4AHN


************** CAUTION *********** DANGER **************************

These are small but very powerful PSUs - take great care. The 100A output will easily melt wires and cause fires and burns. If you take the case off there are VERY HIGH VOLTAGES on exposed metalwork and components that can KILL!!!
