# RomiController
Using an ESP32 to provide Wireless control to Romi 32U4 control board
Uses the PS4 library from here (these changes are needed to deal with the latest (as of this commit) ESP32 BT changes: https://github.com/AzSaSiN/PS4-esp32
The project needs a secrets.h file created with the mac address you are using for pairing the PS4 controller.  I paired it to my phone, then used that mac and forgot it from my phone.
