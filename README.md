# azimuthalencoder
H7 code for azimuthal encoder sampling at 1kHz. Currently no PPS alignment.

#### Latest update: Friday Sept. 7, 2024

## System overview:
- STM H723ZG microcontroller
- Hohner 20-bit absolute position encoder
- Ethernet to Fibre Media Converter
- Switch + computer

## IP settings:
- Microcontroller IP: 10.20.3.2
- Microcontroller subnet mask: 255.255.0.0
- Microcontroller gateway: 10.20.1.1
- Computer IP: 10.20.1.3
- Computer subnet mask length (on lab PC): 16
- Computer gateway (on lab PC): 10.20.1.1

## Data output from microcontroller (not relevant for data analysis, see csv section below for that):
The microcontroller sends a 602-byte package every 10ms. The data is structured as follows. All are 16-bit values:

MSB 16-bits of data, LSB 4-bits of data, LSB 16-bits of clock tick, MSB 16-bits of clock tick, 16-bit spacer pt.1 (0xAB89), 16-bit spacer pt. 2 (0xEFCD)

Note that since these 16-bit values are packaged into 8-bit bytes, with the LSB first, in practice the spacer becomes 0x89ABEFCD, which is used in the python script to parse individual samples from each packet. This is also the reason the indexing in the python script for storing the data and clock tick values might seem slightly strange at first look.
<img width="1049" alt="Screenshot 2024-09-09 at 10 07 25 PM" src="https://github.com/user-attachments/assets/2527d637-e849-4490-8920-522668cc8a44">

## Csv file format:
Python script saves a .csv file with the filename given in the script in the directory in which it is run from.
### | ES1 | ... | ES100 | CT1 | ... | CT100 | PN | T1 | T2 |
- ES = Encoder sample/data
- CT = Clock tick at time of querying corresponding to each encoder sample
- PN = Packet number (increments by 1 and rolls over at 2^16-1)
- T1 = Real time (accurate to 1/10th of a second) when data has been received from UDP socket (added by computer)
- T2 = Number of nanoseconds elapsed since start of the unix epoch (Jan 1, 1970) modulo 1,000,000 (i.e. the number of nanoseconds into the current second) when data is received from UDP socket (added by computer)

## System diagram
![Azimuthal Encoder-6](https://github.com/user-attachments/assets/28ff7027-73ce-4680-9452-8b50fd2c9dd6)

## System Pinout:
<img width="929" alt="Screenshot 2024-09-05 at 4 12 59 PM" src="https://github.com/user-attachments/assets/0d22ecae-d1f9-459e-b847-99cb874eadf4">

## Instructions for installation:
1. Ensure Hut PC is set to static IP 10.20.1.3 and has all IP settings above (which are what was tested in lab)
2. Connect ethernet cable between Hut PC and switch in the hut.
3. Install encoder and rotary stack.
4. Connect 5V and corresponding GND, 12V and corresponding GND, to respective linear power supplies at the telescope base. See Image 1. **IMPORTANT: Add GND line between the two GNDs of the linear power supplies in order to have them share a ground**. Do not plug in linear power supplies yet.
5. Connect DB15 connector to power supplies (J1) to bottom rotary stack power DB15 connector (P1).
6. Connect rotary stack power DB15 connector (J2) to box DB15 connector (P2).
7. Connect encoder connector (J3) to box connector (P3).
8. Connect optic fibre cable coming from box to top of rotary stack (not sure how). Connect optic fibre cable from bottom of rotary stack to switch in hut.
9. Plug in 12V linear power supply first (this ensures that the encoder is 'on' before the microcontroller starts sending it a clock signal). Then plug in 5V linear power supply.
10. Run 'UDPsocket_Sep7_Windows.py' script on terminal/command prompt of Hut PC (the script should work on other operating systems, it was just tested on Windows computer, hence the filename). If nothing prints, try using Wireshark. There may be a problem with the IP configuration of either the Hut PC or the switch.

Image 1: 
![IMG_0527_4](https://github.com/user-attachments/assets/2cd98414-8261-4783-ab78-b3eca20e6723)
