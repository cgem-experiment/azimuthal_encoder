# azimuthalencoder
H7 code for azimuthal encoder sampling at 1kHz. Currently no PPS alignment.

Latest update: Friday Sept. 6, 2024

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

## Data output:

Python script saves a .csv file with the filename given in the script in the directory in which it is run from.

### | ES1 | ... | ES100 | CT1 | ... | CT100 | PN | T1 | T2 |

- ES = Encoder sample/data
- CT = Clock tick at time of querying corresponding to each encoder sample
- PN = Packet number (increments by 1 and rolls over at 2^16-1)
- T1 = Real time (accurate to 1/10th of a second) when data has been received from UDP socket and processed (added by computer)
- T2 = Number of microseconds elapsed since start of python script and when data is received from UDP socket and processed (added by computer)

  

## System diagram


## System Pinout:
<img width="929" alt="Screenshot 2024-09-05 at 4 12 59 PM" src="https://github.com/user-attachments/assets/0d22ecae-d1f9-459e-b847-99cb874eadf4">


## Instructions for installation:
