# Traquino

[Start]
   |
   v
[Init Serial, GPS, I2C, Si5351]
   |
   v
[Si5351 OK?] --No--> [Print "Si5351 not found"] 
   |Yes
   v
[Acquire GPS (120s)]
   |
   v
[Have GPS Time?] --No--> [Print "No GPS fix"]
   |Yes
   v
[Determine shouldTx?] --No--> [Skip TX]
   |Yes
   v
[Determine Grid Locator] 
   |
   v
[Transmit WSPR?] --No--> [Skip TX]
   |Yes
   v
[Send WSPR Packet]
   |
   v
[Mark day as transmitted]
   |
   v
[Disable Si5351 outputs]
   |
   v
[Print "Cycle complete"]
   |
   v
[Enter Loop]

------------------------------
Loop:
[Delay 60s]
   |
   v
[Acquire GPS (10s)]
   |
   v
[Have GPS Time?] --No--> [Skip TX]
   |Yes
   v
[In morning window AND not transmitted today?] --No--> [Skip TX]
   |Yes
   v
[Determine Grid Locator]
   |
   v
[Transmit WSPR]
   |
   v
[Mark day as transmitted]
   |
   v
[Delay 30s]
   |
   v
[Repeat Loop]
