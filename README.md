# AES_RTL


AES Power Ananlysis Attack Counter Measure (PACM) Design is a 32 bit iterative architecture designed in Verilog HDL

# **TOP MODULE:**

The top module consists of a register bank which is used to store the key and plain texts which are loaded from the external environment and the encrypted data is loaded back in the same register bank from which the output can be read. The main objective of this module is to reduce the number of IO ports for the design. Since the core is a 32 – bit iterative core architecture, data or key should be loaded or read in sizes of 32 bits which leads to 32 ports. Using register bank, the number of IO_ports that are required can be reduced to 8 ports. This is created to decrease the number of IO pins during Tapeout.

<img width="405" alt="Screenshot 2021-04-17 at 1 41 59 PM" src="https://user-images.githubusercontent.com/81558273/115106547-182bfc00-9f83-11eb-8eff-7471dfdd2a14.png">


**AESDATAPATH CORE DESIGN:**

The design supports all key variants: 128, 192 and 256 bit for encryption and decryption using the same design. It is a 32-bit core architecture with inbuilt key scheduling module and memory bank of 2KB to store the round key values. 128 bits of data can be given as input and 128 bits of data is obtained as encrypted or decrypted output. Using this design, we can encrypt or decrypt data continuously for a block of data given that the key is unchanged (stream mode). The core consists of modules that perform each operation present in a round. The core is operated in an iterative manner using the control path design. 

Power Analysis attacks are avoided by applying different tasks on the data along with a Pseudo-random number generator that generates a random number for every data. 

![aes_top](https://user-images.githubusercontent.com/81558273/115106542-0c403a00-9f83-11eb-81dc-d9d5e92057ad.jpg)
