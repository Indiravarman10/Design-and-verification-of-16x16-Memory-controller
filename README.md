# Design-and-verification-of-16x16-Memory-controller
**Abstract**

This project presents the design and verification of a Memory Controller for a 16×16-bit single-port RAM operating at 100 MHz. The system performs read and write operations using a request (req) signal and a read/write (rw) control line. Address decoding is performed using a 8-bit input address which is split into 4-bit row and column addresses. A Memory Controller module was implemented in Verilog and verified using simulation testbenches in Vivado. The design was synthesized and deployed on a Xilinx Nexys 4 DDR FPGA. This project emphasizes efficient data storage/retrieval, synchronization, and verification using FPGA hardware.

**Introduction**
Memory controllers serve as the critical link between memory devices and digital systems such as processors and FPGAs. They manage memory access, timing, and data integrity. In this project, a custom controller is designed for a 16×16-bit single-port RAM. The design ensures correct timing and synchronization for read/write operations through the use of a clocked control logic.
The controller accepts a 8-bit address input and splits it into 4-bit row and column addresses. Operations are enabled using a request signal, and the read/write direction is determined using a control input. Read data is output along with a valid signal.

**Literature Review**
Several prior works and textbooks highlight memory controller design principles. Notable references include:
1.	Hardware modelling using Verilog by IIT Kharagpur (NPTEL)
2.	IEEE papers discussing memory optimization techniques for SoC and FPGA environments, especially in high-performance systems.
3.	Compared to generic memory controllers, this project simplifies by targeting a fixed 16x16-bit RAM, allowing focus on address mapping, FSM control, and verification on a real FPGA.

**FPGA Board Review – Xilinx Nexys 4 DDR:**
The Xilinx Nexys 4 DDR FPGA development board features:
•	Artix-7 FPGA (XC7A100T-1CSG324C)
•	100 MHz onboard clock
•	16MB DDR2 memory (not used in this project, but relevant for real-world scaling)
•	Plenty of I/O pins for GPIO, switches, and displays
•	Vivado Design Suite support with powerful simulation/synthesis tools
Its availability, documentation, and performance make it an ideal platform for prototyping memory controller logic.

**Project Objectives:**
1.	Design a Memory Controller  using single-port RAM (16x16-bit) .
2.	Implement read/write logic using a 100 MHz clock.
3.	Validate the design through simulation.
4.	Synthesize and test the design on Nexys 4 DDR FPGA.
5.	Obtain Power, Timing and Utilization report

 **Design and Methodology:**
**RAM Description:**
•	256 locations (8-bit address space)
•	Each location stores 8-bit data
•	Row address = addr[7:4], column address = addr[3:0]
**Controller Tasks:**
•	Decode address → row & column
•	Generate control : IDLE → READ or WRITE → WAIT → DONE
•	Manage data bus direction (write to RAM or read from RAM)
•	Assert valid signal after read



**Interface Signals:**
Signal						Width	Description
clk						1-bit	100 MHz clock
reset						1-bit	Synchronous reset
req						1-bit	Request signal to enable operation
rw						1-bit	1: Read, 0: Write
addr						8-bit	Address input
Qi						8-bit	Input data (write)
Qa						8-bit	Output data (read)
valid						1-bit	Data valid indicator (read)

**Source code:**
 MemoryController_16 (
    input wire clk,
    input wire reset,
    input wire req,
    input wire rw,
    input wire [7:0] addr,
    input wire [7:0] Qi,
    output reg [7:0] Qa,
    output reg valid
);
    reg [7:0] RAM [0:15][0:15];
    reg [3:0] row_addr;
    reg [3:0] col_addr;

    always @(*) begin
        row_addr = addr[7:4];
        col_addr = addr[3:0];
    end

   wire [7:0] RAM_read = RAM[row_addr][col_addr];
    wire RAM_write_en = req && !rw;
    
    always @(posedge clk) begin
        valid <= reset ? 8'h0 : (req & rw);
        Qa    <= reset ? 8'h0 : (req & rw) ? RAM_read : Qa;
        
        if (RAM_write_en) begin
            RAM[row_addr][col_addr] <= Qi;
        end
    end endmodule

Testbench:
	```module MemoryController_16_tb;
    reg clk;
    reg reset;
    reg req;
    reg rw;
    reg [7:0] Qi;
    reg [7:0] addr;
    wire [7:0] Qa;
    wire valid;

    MemoryController_16 uut (
        .clk(clk),
        .reset(reset),
        .req(req),
        .rw(rw),
        .addr(addr),
        .Qi(Qi),
        .Qa(Qa),
        .valid(valid)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin

        reset = 1;
        req = 0;
        rw = 0;
        Qi = 8'h00;
        addr = 8'h00;

        #10 reset = 0;

        addr = 8'h00;
        Qi = 8'hA5;
        req = 1;
        rw = 0; // Write operation
        #10 req = 0;
        #10;
        
        addr = 8'h01;
        Qi = 8'h3C;
        req = 1;
        rw = 0; // Write operation
        #10 req = 0;
        #10;
        
        addr = 8'hCA;
        Qi = 8'hFF;
        req = 1;
        rw = 0; // Write operation
        #10 req = 0;
        #10;

        addr = 8'h11;
        Qi = 8'hBC;
        req = 1;
        rw = 0; // Write operation
        #10 req = 0;
        #10;

        addr = 8'h00;
        req = 1;
        rw = 1; // Read operation
        #10 req = 0;
        #10;

        addr = 8'h01;
        req = 1;
        rw = 1; // Read operation
        #10 req = 0;
        #10;

        addr = 8'hCA;
        req = 1;
        rw = 1; // Read operation
        #10 req = 0;
        #10;

        addr = 8'h11;
        req = 1;
        rw = 1; // Read operation
        #10 req = 0;
        #10;    
            
        #20 $finish;
    end
endmodule

**Constraint File:**
	```#clock signal
	```set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L12P_T1_MRCC_35 Sch=clk100mhz
	```create_clock -period 10.000 -name sys_clk [get_ports clk]
###Switches

	```set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { addr[0] }]; #IO_L24N_T3_RS0_15 Sch=sw[0]
	```set_property -dict { PACKAGE_PIN L16   IOSTANDARD LVCMOS33 } [get_ports { addr[1] }]; #IO_L3N_T0_DQS_EMCCLK_14 Sch=sw[1]
	```set_property -dict { PACKAGE_PIN M13   IOSTANDARD LVCMOS33 } [get_ports { addr[2] }]; #IO_L6N_T0_D08_VREF_14 Sch=sw[2]
	```set_property -dict { PACKAGE_PIN R15   IOSTANDARD LVCMOS33 } [get_ports { addr[3] }]; #IO_L13N_T2_MRCC_14 Sch=sw[3]
	```set_property -dict { PACKAGE_PIN R17   IOSTANDARD LVCMOS33 } [get_ports { addr[4] }]; #IO_L12N_T1_MRCC_14 Sch=sw[4]
	```set_property -dict { PACKAGE_PIN T18   IOSTANDARD LVCMOS33 } [get_ports { addr[5] }]; #IO_L7N_T1_D10_14 Sch=sw[5]
	```set_property -dict { PACKAGE_PIN U18   IOSTANDARD LVCMOS33 } [get_ports { addr[6] }]; #IO_L17N_T2_A13_D29_14 Sch=sw[6]
	```set_property -dict { PACKAGE_PIN R13   IOSTANDARD LVCMOS33 } [get_ports { addr[7] }]; #IO_L5N_T0_D07_14 Sch=sw[7]
	```set_property -dict { PACKAGE_PIN T8    IOSTANDARD LVCMOS18 } [get_ports { Qi[0] }]; #IO_L24N_T3_34 Sch=sw[8]
	```set_property -dict { PACKAGE_PIN U8    IOSTANDARD LVCMOS18 } [get_ports { Qi[1] }]; #IO_25_34 Sch=sw[9]
	```set_property -dict { PACKAGE_PIN R16   IOSTANDARD LVCMOS33 } [get_ports { Qi[2] }]; #IO_L15P_T2_DQS_RDWR_B_14 Sch=sw[10]
	```set_property -dict { PACKAGE_PIN T13   IOSTANDARD LVCMOS33 } [get_ports { Qi[3] }]; #IO_L23P_T3_A03_D19_14 Sch=sw[11]
	```set_property -dict { PACKAGE_PIN H6    IOSTANDARD LVCMOS33 } [get_ports { Qi[4] }]; #IO_L24P_T3_35 Sch=sw[12]
	```set_property -dict { PACKAGE_PIN U12   IOSTANDARD LVCMOS33 } [get_ports { Qi[5] }]; #IO_L20P_T3_A08_D24_14 Sch=sw[13]
	```set_property -dict { PACKAGE_PIN U11   IOSTANDARD LVCMOS33 } [get_ports { Qi[6] }]; #IO_L19N_T3_A09_D25_VREF_14 Sch=sw[14]
	```set_property -dict { PACKAGE_PIN V10   IOSTANDARD LVCMOS33 } [get_ports { Qi[7] }]; #IO_L21P_T3_DQS_14 Sch=sw[15]


### LEDs

	```set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 } [get_ports { Qa[0] }]; #IO_L18P_T2_A24_15 Sch=led[0]
	```set_property -dict { PACKAGE_PIN K15   IOSTANDARD LVCMOS33 } [get_ports { Qa[1] }]; #IO_L24P_T3_RS1_15 Sch=led[1]
	```set_property -dict { PACKAGE_PIN J13   IOSTANDARD LVCMOS33 } [get_ports { Qa[2] }]; #IO_L17N_T2_A25_15 Sch=led[2]
	```set_property -dict { PACKAGE_PIN N14   IOSTANDARD LVCMOS33 } [get_ports { Qa[3] }]; #IO_L8P_T1_D11_14 Sch=led[3]
	```set_property -dict { PACKAGE_PIN R18   IOSTANDARD LVCMOS33 } [get_ports { Qa[4] }]; #IO_L7P_T1_D09_14 Sch=led[4]
	```set_property -dict { PACKAGE_PIN V17   IOSTANDARD LVCMOS33 } [get_ports { Qa[5] }]; #IO_L18N_T2_A11_D27_14 Sch=led[5]
	```set_property -dict { PACKAGE_PIN U17   IOSTANDARD LVCMOS33 } [get_ports { Qa[6] }]; #IO_L17P_T2_A14_D30_14 Sch=led[6]
	```set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS33 } [get_ports { Qa[7] }]; #IO_L18P_T2_A12_D28_14 Sch=led[7]

	```set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { valid }]; #IO_L11P_T1_SRCC_14 Sch=led16_r
###Buttons

	```set_property -dict { PACKAGE_PIN C12   IOSTANDARD LVCMOS33 } [get_ports { reset }]; #IO_L3P_T0_DQS_AD1P_15 Sch=cpu_resetn
	```set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports { rw }]; #IO_L10N_T1_D15_14 Sch=btnr
	```set_property -dict { PACKAGE_PIN P18   IOSTANDARD LVCMOS33 } [get_ports { req }]; #IO_L9N_T1_DQS_D13_14 Sch=btn

**Simulation Results:**
 <img width="1161" height="632" alt="Image" src="https://github.com/user-attachments/assets/675df677-fab5-4313-bc9c-1f0a653b1ad8" />
**Power Consumption:**
 <img width="1151" height="578" alt="Image" src="https://github.com/user-attachments/assets/e6714303-03d0-41df-bfd5-23cd604f4bc4" />
**Timing Analysis:**
 <img width="1145" height="621" alt="Image" src="https://github.com/user-attachments/assets/912bd952-5655-409b-96dc-38240c08507d" />
**Hardware Utilization:**  
 <img width="1151" height="566" alt="Image" src="https://github.com/user-attachments/assets/b812c585-3e95-4c94-bd66-3307af3d46a9" />


 **Tools Used:**
•	Vivado Simulator (2023.1)
•	Xilinx Synthesis for Artix-7

 **FPGA Implementation:**
•	Mapped all inputs to switches and outputs to LEDs or 7-segment display.
•	Verified correctness using hardware debug 
•	Successfully read back correct data after writing.

**Conclusion:**
The memory controller was successfully designed and verified using Verilog, and implemented on the Xilinx Nexys 4 DDR FPGA. The controller efficiently decoded addresses and handled read/write operations with minimal latency. This project enhances understanding of memory systems and FPGA verification workflows.
Future work can include:
•	Dual-port memory
•	Pipelined controller
•	AXI interface integration
•	Dynamic memory timing adjustment
   **References:**
1.	 Hardware modelling using Verilog by IIT Kharagpur (NPTEL)
2.	Xilinx Nexys 4 DDR User Guide
3.	IEEE Xplore papers on memory controller architectures

 
