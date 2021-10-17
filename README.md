# AES-RTL 
**(Project taken as a part of tape out of AES Design -- Refer Future work section below)**

## AES Power Ananlysis Attack Counter Measure (PACM) Design :: A 32 bit iterative architecture designed in Verilog HDL

## **TOP MODULE:**

A byte-addressable register bank stores the plain text (data) or the whitening key (input key) loaded from the external environment. After the desired operation (encryption/ decryption) is complete, the output data is written back in the same register bank from which it can be read through the IO ports. The main objective of this module is to reduce the number of IO ports for the design for better-used case scenarios after tape-out. Since the core is a 32 – bit iterative architecture, data or key should be loaded or read in sizes of 32 bits which leads to 32 data pins. Through a byte-addressable register bank, the number of IO_ports required has been reduced to 8 ports. 

#### OUTER BLOCK DIAGRAM

<img width="405" alt="Screenshot 2021-04-17 at 1 41 59 PM" src="https://user-images.githubusercontent.com/81558273/115106547-182bfc00-9f83-11eb-8eff-7471dfdd2a14.png">

#### PORT DETAILS

<img width="502" alt="Screenshot 2021-10-02 at 5 32 44 PM" src="https://user-images.githubusercontent.com/81558273/135715239-43614e0f-05f3-4598-872c-23174005f6ab.png">

## **AESDATAPATH CORE DESIGN:**

The design supports all key variants: 128, 192, and 256 bit for encryption and decryption using the same core. It is a 32-bit architecture with an inbuilt key scheduling module and memory bank of 2KB to store the round-key values. Storing round key values help in better performance while encryption or decryption because of direct fetches from the Key memory and also, if the whitening key is maintained, a stream of data can be encrypted and decrypted with Key Scheduling Algorithm block run only once. 128 bits of data is given as input and 128 bits of data is obtained as encrypted or decrypted output. The core consists of modules that perform each operation present in a round. The data path design is operated in an iterative manner using the control path design. 

Power Analysis attacks are avoided by applying different tasks on the data along with a Pseudo-random number generator that generates a random number for every data. 

<img width="748" alt="Screenshot 2021-10-02 at 5 45 26 PM" src="https://user-images.githubusercontent.com/81558273/135715652-b5a67e28-728f-4ca3-b702-e671f730fcb6.png">

## **KEY SCHEDULING ALGORITHM DESIGN:** 

The AES Key Expansion takes the Cipher Key K (of Nk bytes) and generates round keys for Nr rounds. So, in total, Nb * (Nr+1) words will be present. 
It involves XORing w\[i-1] and w\[i-Nk] columns in the memory to produce round keys. The w\[i-1]th column has to be transformed or not based on the following conditions: 
1. The current word is in the position equal to multiple of Nk and 
2. Key is 256 bits and the current word’s position satisfies (i mod  Nk == 4). This signal is high only if any of the above is true.
3. Transformation consists of Substitution (SBox) and RotWord (cyclic shifts) followed by XOR with Round CONstant (RCON).

![ksa_colour](https://user-images.githubusercontent.com/81558273/120992843-9eeda000-c7a0-11eb-9e91-d3219ac215c6.jpg)

This module contains the data path and control path integrated with the memory to store the cipher key and the round keys. It supplies the AES core module with the required round keys for encryption/decryption. it has an in-built ‘Inverse Mix Column’ implementation along with it to perform equivalent cipher operations. To achieve decryption of any Ciphertext, the normal encryption flow followed in the AES CORE module has to be modified slightly, i.e., the data has to flow in a different path to achieve decryption. For decryption, because of using Inverse Mix Columns operation to the Round Keys, the same flow can be maintained, without even the slightest change in the logic or code. 

This module:
1.	Receives the Whitening key and start signal of key scheduling from AES TOP.
2.	Computes the Round keys as a standalone module.
4.	Upon a read request from AES TOP to read a round key, KSA returns a 32 bit round key by fetching the key information from the KSA_MEMORY using the address which is provided by the AES Control path.

## Future Work Performed with the RTL Design
The AES RTL design was taken and back-end design flow was performed on the design. The back-end design was performed under Semiconductor Laboratory's 180nm technology (https://scl.gov.in/). Starting from logic synthesis on creating the netlist using the technology files, physical design was continued. 
We performed Floorplanning, Placement, Power Planning and Power Routing, Clock Tree Synthesis, Routing and Timing sign-off and verified post timing sign-off using the standard delay file obtained from the timing sign-off tool.

#### PLACED AND ROUTED DESIGN LAYOUT OF AES

<img width="534" alt="Screenshot 2021-10-02 at 6 13 00 PM" src="https://user-images.githubusercontent.com/81558273/135716618-5aa6feab-0647-4c1e-bc6e-876e8d62c28b.png">

#### GDS IMPORT AND LAYOUT VIEW OF AES 

<img width="625" alt="Screenshot 2021-10-02 at 6 14 01 PM" src="https://user-images.githubusercontent.com/81558273/135716669-b7e64b42-2785-4809-af85-31faa7f9f3f6.png">

## AES POST-LAYOUT DESIGN SPECIFICATION

<img width="503" alt="Screenshot 2021-10-02 at 6 04 08 PM" src="https://user-images.githubusercontent.com/81558273/135716307-e56fd927-9c05-4e82-aa86-d8b20f182d98.png">

## AES DESIGN MICROGRAPH

The GDSII of the AES Desing was obtained and sent for tape out to SCL Fabrication laboratory. Below is a snapshot of the micrograph.

![112](https://user-images.githubusercontent.com/81558273/135715741-4a823ead-f3e6-4c11-92b3-90cfb8d52e74.jpeg)
