# Final-Prj-of-Advanced-computer-architecture
25fall Advanced computer architecture, Final PRJ
By Trevor, Thierry, Yang

we build a cpu in RiscV with special instruction set.

Link of Document
https://docs.google.com/document/d/1RRewOy-fuHN_yRXx5kFoFxv4HNzzx_Hq/edit


/////////////////  bug 1  ///////////////
Simulation successfully ran for 3500ns, but the Data Memory dump shows all zeros,
indicating the Bubble Sort program did not correctly write any data to memory. 
The sorting logic is sound, but the data path for store (sw) instructions 
is being corrupted by a crucial, but simple, data width mismatch warning in top module.
/////////////////////////////////////////

////////////////  Forward Unit  ////////////////// 
The core feature of this pipelined design is the Forwarding_Unit which uses two bypass paths to resolve Read-After-Write (RAW) data hazards. This avoids most pipeline stalls.Forwarding Logic:The unit monitors two downstream stages to see if they are producing a required value (register write is enabled, RD != 0, and the destination register matches the source register needed in EX).
•ForwardA/B = 2'b10 (MEM Hazard): The required data is currently in the MEM stage (from res_M). This is the fastest bypass.
•ForwardA/B = 2'b01 (WB Hazard): The required data is currently in the WB stage (from Result_W).
•ForwardA/B = 2'b00 (No Hazard): Use the data read from the Register File in the ID stage (ReadData1_E/ReadData2_E).

SrcAMux (Input A)
A 3-to-1 MUX that selects the final ALU operand A. Its selection is controlled by the ForwardA_E signal (2-bit), choosing between the Register File read, the WB stage result (Result_W), or the MEM stage result (res_M).

SrcBMux (Input B for Store)
A 3-to-1 MUX controlled by ForwardB_E that selects the second ALU operand. Crucially, this is where the sw instruction's data operand is selected and passed as WriteData_E_Final to the MEM stage.

ALUSrcMUX (Final Input B)
A 2-to-1 MUX (controlled by ALUSrc_E) that selects the final ALU operand B, choosing between the result from SrcBMux (another register, potentially forwarded) or the imm_data_E (the sign-extended immediate value).
