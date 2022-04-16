# VLSI_design-of-RISC
A Project by IEEE Diode on the designing of a RISC Processor with features like Branch predictors, Forwarding and implementing the physical design of the same on OpenLane.

## Introduction
Microprocessors and Microcontrollers are generally designed in the vicinity of two main computer architectures: Complex Instruction Set Computing i.e. CISC architecture and Reduced Instruction Set Computing i.e. The main idea of CISC is that a single instruction will do all loading, evaluating, and storing operations just like a multiplication command will do stuff like loading data, evaluating, and storing it, hence it’s complex.  Thus causing them to have varying execution time and lengths thereby authoritatively mandating an intricate Control Unit, which inhabits an immensely existent region on the chip. 

Compared with their CISC analogue, RISC processors typically support a minuscule set of instructions. A display that juxtaposes RISC processor with CISC processor, the number of instructions in a RISC Processor is low while the number of general purpose registers, addressing modes, fixed instruction length and load-store architecture is more this in turn facilitates the execution of instructions to be carried out in a short time thus achieving higher overall performance .Currently, the efficacy of the RISC processors is generally accepted to be greater than that of their CISC counterparts. Before their execution the instructions are translated into RISC instructions in even the most popular CISC processors. The attributes mentioned above accentuate the design strength of RISC in the market for embedded systems known as "system-on-a-chip (SoC)". The premier micro processors exhibiting reduced instruction set are SPARC, ARM, MIPS and IBM's PowerPC. RISC processors typically have load store architecture. This denotes there are two instructions for accessing memory which are a load instruction set to load data from the memory and store instruction set to Write Back (WB) the data into memory without any instructions

## Objective
To generate a full GDSII from a RTL netlist for a fully working and optimized RISC microprocessor from scratch using openLane.

### MIPS Instruction Set Architecture:
Instructions are encoded as 32-bit instruction words.The instruction set can be categorized under three classifications in the MIPS ISA, these are: Register type (R), Immediate type (I) and Jump type (J).

The following table demonstrates the three formats used for MIPS core instruction set architecture.
![image](https://user-images.githubusercontent.com/78913275/163660344-8d9c5084-5653-4534-8ee0-6425b73a4b0f.png)

### R-format Instructions:
![image](https://user-images.githubusercontent.com/78913275/163660337-be2923a9-cf5b-4373-af73-e83da4852aa4.png)


Instruction Fields
-op : operation code (opcode)
-rs : first source register number
-rt : second source register number
-rd : destination register number
-shamt : shift amount (00000 for now)
-funct : function code (extends opcode)

![image](https://user-images.githubusercontent.com/78913275/163660357-ac541ac0-5965-4f1a-a161-dec9461eb962.png)

In R-format instructions the data is read from two register operands (rs & rt) .These instructions are used to perform arithmetic and logical operations and write back the result into the register stored in rd.

### Immediate (I) format:
 ![image](https://user-images.githubusercontent.com/78913275/163660392-2eb272f8-eb2c-419a-bf43-a6489b62e232.png)


### Types of instructions: load, store, addi
![image](https://user-images.githubusercontent.com/78913275/163660394-3f06ba64-868c-477d-aa37-96e8974b2e4b.png)

 

Immediate arithmetic and load/store instructions
-rt : destination or source register number
-Constant: -2 ^15 to +2 ^15 
-Address: offset added to base address in rs

### Jump Addressing
Jump ( j and jal ) targets could be anywhere in text
segment

### Encode full address in instruction
 ![image](https://user-images.githubusercontent.com/78913275/163660400-58eb43c9-0b8f-4083-b640-659a8a76c4b2.png)

(Pseudo)Direct jump addressing
Target address = PC 31…28 : (address × 4)

### Branch Addressing
Branch instructions specify Opcode, two registers, target address. Most branch targets are near branch. 

 ![image](https://user-images.githubusercontent.com/78913275/163660437-aac78de8-0782-435d-8741-5700b401c263.png)

PC relative addressing
Target address = PC + offset × 4
PC already incremented by 4 by this time

### Instruction execution:

Microprocessor without Interlocked Pipeline Stages (MIPS) is a RISC (Reduced Instruction Set Computing) architecture.Pipelining means several operations in a single data path at the same instant. Pipelined MIPS has five stages which are IF, ID, EX, MEM and WB. 

## MIPS PIPELINE
Five stages, one step per stage 

1. IF: Instruction fetch from memory 
2. ID/RF: Instruction decode & register read 
3. EX/EA: Execute operation or calculate address 
4. MEM: Access memory operand 
5. WB: Write result back to register

Pipelining is used to enhance the capabilities of the RISC processor which is the reason for its
utilization in this type of computer architecture. A multi cycle CPU comprises countless tasks. So if one task occurs, rather than waiting for the process to finish, at the same time another task is initiated in the same data path simultaneously without interfering with the previous task.
The process is thus divided into different pipelined stages.
Following every clock a new operation is loaded into the program counter (PC) in the
pipeline stage to which the process is being fed to. The triggering is done without causing any interruptions to the past process. This makes simultaneous utilization of all stages in the data path possible. This thus can increment the throughput of MIPS.

### CPU Overview
![image](https://user-images.githubusercontent.com/78913275/163660485-13bac6ea-f4fa-4488-bbbb-99a00dcf1824.png)

The control unit tells the datapath what to do, based on the instruction that’s currently being executed.
Our processor has ten control signals that regulate the datapath.The control signals can be generated by a combinational circuit with the instruction’s 32-bit binary encoding as input.

### Datapath Design
![image](https://user-images.githubusercontent.com/78913275/163660487-cc913525-5716-4411-90a0-c08f7da7cb9d.png)

In this stage, the instruction memory receives the address of 32 bit instruction given by the Program Counter(PC). Using the address the instruction is fetched from the instruction memory.

![image](https://user-images.githubusercontent.com/78913275/163660493-6a802a7a-e9ab-4b40-a2d7-b2ae3de0db54.png)


In branch type instructions when the instructions are decoded. We need an alu unit to compare the value in both the registers(rs and rt).  for example in beq instruction if the value in both the registers are same then a zero flag in alu unit is triggered and program counter (PC) will branch to that address mentioned in executing instruction (calculated by sigh extending the 16 bit address/ constant)

![image](https://user-images.githubusercontent.com/78913275/163660503-41c94e84-8fe9-48a8-97a5-fcef6042ce24.png)

The two elements needed to implement R-format ALU operations are the register file and the ALU. The register file contains all the registers and has two read ports and one write port.
The register file always outputs the contents of the registers corresponding to the Read register inputs on the outputs; no other control inputs are needed. In contrast, a register write must be explicitly indicated by asserting the write control signal.
The RISC-V load register and store register instructions, which have the general form ld x1, offset(x2) or sd x1, offset(x2). These instructions compute a memory address by adding the base register, which is x2, to the 12-bit signed offset field contained in the instruction. If the instruction is a store, the value to be stored must also be read from the register file where it resides in x1. If the instruction is a load, the value read from memory must be written into the register file in the specified register, which is x1.

### ALU Control 
![image](https://user-images.githubusercontent.com/78913275/163660527-4e595b6f-e38a-4f61-a987-5eda5c945aee.png)

Alu control takes in function code and shift amount as input and a combinational logic derives alu control signals to the ALU unit. These alu control signals decides which alu operation has to be performed while taking in what type of instruction is given

### Full Datapath
![image](https://user-images.githubusercontent.com/78913275/163660571-6362ac83-7a6d-4d73-b6fb-4e88cdfc0bf8.png)

Here all stages discussed above are linked together.

### Datapath with Control
![image](https://user-images.githubusercontent.com/78913275/163660603-3618a57b-b770-4970-9847-9907fd0b7f27.png)


##  Processor design:

MIPS processor is executed utilizing five pipeline stages, which are Instruction Fetch (IF), Instruction Decode (ID), Execution Stage (EX), Memory access (MEM) and Write Back (WB).
This isolation of stages is achieved by special registers known as pipeline registers. The aim of 
These registers are to isolate the stages of the instructions so that there is no inadmissible information because of various directions being executed all the while. They are named in the middle of each of these: IF/ID Register, ID/EX Register, EX/MEM Register and MEM/WB Register. The data path demonstrated in above figure is that of the MIPS pipelined processor.

## Physical design preview

Physical design of the processor can be obtained using a tool called OpenLane. OpenLane is an automated RTL to GDSII flow based on several components including OpenROAD, Yosys, Magic, Netgen, CVC, SPEF-Extractor, CU-GR, Klayout and a number of custom scripts for design exploration and optimization. The flow performs full ASIC implementation steps from RTL all the way down to GDSII.
This tool is used for Macro Hardening and SoC integration
Macro Hardening is converting RTL Design to corresponding physical views such as GDS view
and SoC integration is  Adding IO pads, floor planning the chip using other hardened macros.
![image](https://user-images.githubusercontent.com/78913275/163660633-28c1df57-3a26-4316-90d2-6e08dfd0d8c4.png)

This is the OpenLane Macro Hardening basic flow
![image](https://user-images.githubusercontent.com/78913275/163660639-6722320b-4cca-49c0-82c0-ccb280ee09ea.png)

Before running the whole design we are expected to perform synthesis exploration of the design. Which is the process of getting different netists using different synthesis strategies.
![image](https://user-images.githubusercontent.com/78913275/163660647-c1b291e5-16c0-4f27-a9c2-0536768f5b13.png)


The above figure shows 7 different strategies which one can follow to choose the most suitable design.The plot gives information about the delay and area of the design. One should aim to select the strategy with least area and delay. After choosing the strategy best suited, we need to then add it in the config.tcl file.
Finally, we can run our design and obtained the GDSII view.


## Results and Discussion
Successfully implemented all the 5 pipelining stages in verilog

## Conclusions and future work
When we ran the design on OpenLane, a few modules gave errors. After sorting them, we will be implementing branch prediction and feed forwarding.

## References
http://sersc.org/journals/index.php/IJAST/article/view/22228/11141 

Openlane tutorial: https://www.youtube.com/watch?v=d0hPdkYg5QI







