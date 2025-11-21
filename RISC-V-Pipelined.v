module Control_Unit(opcode , Branch, Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
input [6:0] opcode;
output reg Branch , Jump , MemWrite , ALUSrc , RegWrite;
output reg [1:0] ImmSrc , ALUOp ,  ResultSrc;
always @(*) begin 
case (opcode)
7'b0110011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_00_xx_1_0_0_10_0;
7'b0000011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_01_00_1_0_0_00_0;
7'b0100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_xx_01_0_1_0_00_0;
7'b1100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_xx_10_0_0_1_01_0;
7'b0010011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_00_00_1_0_0_10_0;
7'b1101111 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_10_11_1_0_0_xx_1;
default    : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_xx_xx_x_x_x_xx_x;
endcase
end
endmodule
module ALU_Control(ALUOp , Funct3 , Funct7 , op , Operation);
input [1:0] ALUOp;
input [2:0] Funct3;
input Funct7 , op;
output reg [2:0] Operation;

always @(*) begin
case(ALUOp)
2'b00: Operation = 3'b000;  // add for load/store
2'b01: Operation = 3'b001;  // sub for branch
2'b10: begin               // R-type/I-type
case(Funct3)
3'b000: Operation = (op && Funct7) ? 3'b001 : 3'b000; // sub if R-type and Funct7=1, add otherwise
 3'b010: Operation = 3'b101; // slt
                3'b110: Operation = 3'b011; // or
                3'b111: Operation = 3'b010; // and
                default: Operation = 3'b000;
            endcase
        end
        default: Operation = 3'b000;
    endcase
end
endmodule
module CU(opcode , Funct3 , Funct7 , ResultSrc , MemWrite , ALUSrc , ImmSrc , RegWrite , Operation , Branch , Jump);
input [6:0] opcode ,Funct7;
input [2:0] Funct3;
output  MemWrite , ALUSrc , RegWrite;
output  [1:0] ImmSrc , ResultSrc;
output  [2:0] Operation;
output Branch , Jump;
wire [1:0] ALUOp;
Control_Unit c(opcode , Branch , Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
ALU_Control  a(ALUOp , Funct3 , Funct7[5] , opcode[5] , Operation);
endmodule



module registerFile (
    input wire clk,             // Clock signal
    input wire RegWrite,        // Write enable signal
    input wire [4:0] RS1,       // Source register 1 (5 bits, selects one of 32 registers)
    input wire [4:0] RS2,       // Source register 2 (5 bits, selects another of 32 registers)
    input wire [4:0] RD,        // Destination register (5 bits, selects the register to write to)
    input wire [31:0] WriteData, // Data to be written to register RD (32 bits wide)
    output wire [31:0] ReadData1, // Data read from register RS1 (32 bits wide)
    output wire [31:0] ReadData2  // Data read from register RS2 (32 bits wide)
);

    // 32 registers, each 32 bits wide
    reg [31:0] Registers [31:0];  

    // Initialize registers with random values
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] = i;  // Initialize with random values for simulation
    end

    // Read operation (asynchronous)
    assign ReadData1 = Registers[RS1];  // Read register RS1
    assign ReadData2 = Registers[RS2];  // Read register RS2

    // Write operation (synchronous, on negative clock edge)
    always @(negedge clk) begin
        if (RegWrite) begin
            Registers[RD] <= WriteData;  
        end
    end

endmodule



module alu(a, b, op, res, zero);
input [31:0] a, b; 
input [2:0] op;
output reg zero;
output reg [31:0] res;

always @(*) begin
    case(op)
        3'b000: res = a + b;    // ADD
        3'b001: res = a - b;    // SUB
        3'b101: res = a < b;    // SLT
        3'b011: res = a | b;    // OR
        3'b010: res = a & b;    // AND
        default: res = 32'b0;
    endcase
    
    zero = (res == 32'b0) ? 1'b1 : 1'b0;
end
endmodule



module imm_data_gen(instruction , ImmSrc , imm_data);
input [31:0] instruction;
input [1:0] ImmSrc;
output reg [31:0] imm_data;
always @(*) begin
    case(ImmSrc)
        2'b00: imm_data = {{20{instruction[31]}}, instruction[31:20]};
        2'b01: imm_data = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        2'b10: imm_data = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        2'b11: imm_data = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        default: imm_data = 32'b0; 
    endcase
end
endmodule


module Data_Memory(
    input wire [31:0] Mem_Addr,    
    input wire [31:0] Write_Data,  
    input wire clk,                
    input wire MemWrite,           
    output wire [31:0] Read_Data   
);

    reg [7:0] memory [63:0];       

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1)
            memory[i] = i;
    end

    assign Read_Data = {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]};

    always @(posedge clk) begin
        if (MemWrite) begin
            {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]} = Write_Data;
        end
    end
endmodule

module adder (a , b , out);
input [31:0] a , b;
output [31:0] out;
assign out = a + b ; 
endmodule



module Instruction_Memory(Inst_Address, Instruction); 
    input wire [31:0] Inst_Address;   
    output reg [31:0] Instruction; 
    reg [7:0] memory [83:0]; 

    initial begin
        memory[0] = 8'b00010011;
        memory[1] = 8'b00001011;
        memory[2] = 8'b00000000;
        memory[3] = 8'b00000000;

        memory[4] = 8'b10010011;
        memory[5] = 8'b00001011;
        memory[6] = 8'b00000000;
        memory[7] = 8'b00000000;

        memory[8] = 8'b00010011;
        memory[9] = 8'b00000101;
        memory[10] = 8'b10100000;
        memory[11] = 8'b00000000;

        memory[12] = 8'b00010011;
        memory[13] = 8'b00011100;
        memory[14] = 8'b00101011;
        memory[15] = 8'b00000000;

        memory[16] = 8'b00100011;
        memory[17] = 8'b00100000;
        memory[18] = 8'b01101100;
        memory[19] = 8'b00000011;

        memory[20] = 8'b00010011;
        memory[21] = 8'b00001011;
        memory[22] = 8'b00011011;
        memory[23] = 8'b00000000;

        memory[24] = 8'b11100011;
        memory[25] = 8'b10011010;
        memory[26] = 8'b01010010;
        memory[27] = 8'b11111110;

        memory[28] = 8'b00010011;
        memory[29] = 8'b00001011;
        memory[30] = 8'b00000000;
        memory[31] = 8'b00000000;

        memory[32] = 8'b00010011;
        memory[33] = 8'b00011100;
        memory[34] = 8'b00101011;
        memory[35] = 8'b00000000;

        memory[36] = 8'b00110011;
        memory[37] = 8'b00000000;
        memory[38] = 8'b10110011;
        memory[39] = 8'b00000000;

        memory[40] = 8'b10010011;
        memory[41] = 8'b10011100;
        memory[42] = 8'b00101011;
        memory[43] = 8'b00000000;

        memory[44] = 8'b10000011;
        memory[45] = 8'b00100010;
        memory[46] = 8'b00011010;
        memory[47] = 8'b00000000;

        memory[48] = 8'b10000011;
        memory[49] = 8'b00110000;
        memory[50] = 8'b00011010;
        memory[51] = 8'b00000000;

        memory[52] = 8'b01100011;
        memory[53] = 8'b00000010;
        memory[54] = 8'b00010010;
        memory[55] = 8'b00000100;

        memory[56] = 8'b10110011;
        memory[57] = 8'b00101011;
        memory[58] = 8'b00101000;
        memory[59] = 8'b00000000;

        memory[60] = 8'b00100011;
        memory[61] = 8'b00100000;
        memory[62] = 8'b01101100;
        memory[63] = 8'b00000011;

        memory[64] = 8'b00100011;
        memory[65] = 8'b10101000;
        memory[66] = 8'b01010001;
        memory[67] = 8'b00000011;

        memory[68] = 8'b10010011;
        memory[69] = 8'b00001011;
        memory[70] = 8'b00011011;
        memory[71] = 8'b00000000;

        memory[72] = 8'b11100011;
        memory[73] = 8'b10011010;
        memory[74] = 8'b11000011;
        memory[75] = 8'b11111110;

        memory[76] = 8'b00010011;
        memory[77] = 8'b00001011;
        memory[78] = 8'b00011011;
        memory[79] = 8'b00000000;

        memory[80] = 8'b01100011;
        memory[81] = 8'b10001100;
        memory[82] = 8'b01010101;
        memory[83] = 8'b01111110;
    end

    always @(*) begin
        Instruction = {memory[Inst_Address + 3], memory[Inst_Address + 2], memory[Inst_Address + 1], memory[Inst_Address]};
    end 
endmodule


module mux32bit(a,b,c,sel,out);
input [31:0] a,b,c;
input [1:0] sel;
output reg [31:0] out; 
always @(*) begin 
case(sel)
2'b00 : out <= a;
2'b01 : out <= b;
2'b10 : out <= c;
default: out <=32'b0;
endcase
end
endmodule


module mux32bit2(a,b,sel,out);
input [31:0] a,b;
input  sel;
output reg [31:0] out; 
always @(*) begin 
case(sel)
1'b0 : out <= a;
1'b1 : out <= b;
default: out <=32'b0;
endcase
end
endmodule

module Program_Counter (clk , rst , PC_In , PC_Out);
input clk , rst;
input [31:0] PC_In;
output reg [31:0] PC_Out;
always @(posedge clk) begin
if (rst)
PC_Out <= 0;
else
PC_Out <= PC_In;
end
endmodule 


module Pipe_D(clk , Instruction_F , PC_Out_F  , PCPlus4_F , Instruction_D , PC_Out_D , PCPlus4_D);
input clk;
input [31:0] Instruction_F , PC_Out_F , PCPlus4_F;
output reg [31:0] Instruction_D , PC_Out_D , PCPlus4_D;
always @(posedge clk) begin
Instruction_D <= Instruction_F;
PC_Out_D <= PC_Out_F;
PCPlus4_D <= PCPlus4_F;
end
endmodule

module Pipe_E(clk , ReadData1_D , ReadData2_D , PC_Out_D , RD_D , imm_data_D , PCPlus4_D , MemWrite_D , ALUSrc_D , RegWrite_D , ImmSrc_D , ResultSrc_D , Operation_D , Branch_D , Jump_D , MemWrite_E , ALUSrc_E , RegWrite_E , ImmSrc_E , ResultSrc_E , Operation_E , Branch_E , Jump_E , ReadData1_E , ReadData2_E , PC_Out_E , RD_E , imm_data_E , PCPlus4_E);
input clk;
input [31:0] ReadData1_D , ReadData2_D , PC_Out_D , imm_data_D , PCPlus4_D;
input [4:0] RD_D;
input  MemWrite_D , ALUSrc_D , RegWrite_D;
input  [1:0] ImmSrc_D , ResultSrc_D;
input  [2:0] Operation_D;
input Branch_D , Jump_D;
output reg MemWrite_E , ALUSrc_E , RegWrite_E;
output reg [1:0] ImmSrc_E , ResultSrc_E;
output reg [2:0] Operation_E;
output reg Branch_E , Jump_E;
output reg [31:0] ReadData1_E , ReadData2_E , PC_Out_E , imm_data_E , PCPlus4_E;
output reg [4:0] RD_E;
always @(posedge clk) begin
MemWrite_E <= MemWrite_D;
ALUSrc_E <= ALUSrc_D;
RegWrite_E <= RegWrite_D; 
ImmSrc_E <= ImmSrc_D;
ResultSrc_E <= ResultSrc_D; 
Operation_E <= Operation_D; 
Branch_E <= Branch_D; 
Jump_E <= Jump_D;
ReadData1_E <= ReadData1_D;
ReadData2_E <= ReadData2_D;
PC_Out_E <= PC_Out_D;
imm_data_E <= imm_data_D;
PCPlus4_E <= PCPlus4_D;
RD_E <= RD_D;
end
endmodule

module Pipe_M(clk , RegWrite_E , ResultSrc_E , MemWrite_E , res_E , WriteData_E , RD_E , PCPlus4_E , RegWrite_M , ResultSrc_M , MemWrite_M , res_M , WriteData_M , RD_M , PCPlus4_M);
input clk;
input RegWrite_E , MemWrite_E;
input [1:0] ResultSrc_E;
input [31:0] res_E , WriteData_E , PCPlus4_E;
input [4:0] RD_E;
output reg RegWrite_M , MemWrite_M;
output reg [1:0] ResultSrc_M;
output reg [31:0] res_M , WriteData_M , PCPlus4_M;
output reg [4:0] RD_M;
always @(posedge clk) begin
RegWrite_M <= RegWrite_E;
ResultSrc_M <= ResultSrc_E;
MemWrite_M <= MemWrite_E; 
res_M <= res_E;
WriteData_M <= WriteData_E;
RD_M <= RD_E;
PCPlus4_M <= PCPlus4_E;
end
endmodule

module Pipe_W(clk , RegWrite_M , ResultSrc_M , res_M , Read_Data_M , RD_M , PCPlus4_M , RegWrite_W , ResultSrc_W , res_W , Read_Data_W , RD_W , PCPlus4_W );
input clk , RegWrite_M;
input [1:0] ResultSrc_M;
input [31:0] res_M , Read_Data_M , PCPlus4_M;
input [4:0] RD_M;
output reg RegWrite_W;
output reg [1:0] ResultSrc_W;
output reg [31:0] res_W , Read_Data_W , PCPlus4_W;
output reg [4:0] RD_W;
always @(posedge clk) begin
RegWrite_W <= RegWrite_M;
ResultSrc_W <= ResultSrc_M;
res_W <= res_M;
Read_Data_W <= Read_Data_M;
RD_W <= RD_M;
PCPlus4_W <= PCPlus4_M;
end
endmodule


module RISC_V_Processor_Pipelined(clk , rst);
input clk , rst;
wire [31:0] PCPlus4_F , PCPlus4_D , PCPlus4_E , PCPlus4_W  , PCPlus4_M , PCin_F , PCTarget_E , PC_Out_F ,  PC_Out_D , PC_Out_E , Instruction_F , Instruction_D , Result_W , ReadData1_D , ReadData2_D , ReadData1_E , ReadData2_E , ReadData2_M , res_E , res_M , res_W , Read_Data_M ,  Read_Data_W , imm_data_D , imm_data_E , SrcB_E;
wire [4:0] Instruction_W , Instruction_E , Instruction_M;
wire [2:0] Operation_D , Operation_E;
wire [1:0]  ImmSrc_D , ImmSrc_E , ResultSrc_W , ResultSrc_D , ResultSrc_E , ResultSrc_M;
wire  RegWrite_D , RegWrite_E , RegWrite_M  , RegWrite_W  ,  PCSrc_E , ALUSrc_D , ALUSrc_E , Zero_E , Branch_E , Branch_D , Jump_E , Jump_D , MemWrite_D , MemWrite_E , MemWrite_M ;

adder pcp4(PC_Out_F , 32'b100 , PCPlus4_F);

assign PCSrc_E = Jump_E | (Zero_E & Branch_E);

mux32bit2 PCSrcMUX(PCPlus4_F , PCTarget_E , PCSrc_E , PCin_F);

Program_Counter PC(clk , rst , PCin_F , PC_Out_F);

Instruction_Memory IM(PC_Out_F , Instruction_F);

Pipe_D PD(clk , Instruction_F , PC_Out_F  , PCPlus4_F , Instruction_D , PC_Out_D , PCPlus4_D);

mux32bit ResultSrcMUX(res_W , Read_Data_W , PCPlus4_W , ResultSrc_W , Result_W);

registerFile RF(clk , RegWrite_W , Instruction_D[19:15] , Instruction_D[24:20] , Instruction_W , Result_W , ReadData1_D , ReadData2_D);

imm_data_gen ID(Instruction_D , ImmSrc_D , imm_data_D);

Pipe_E PE(clk , ReadData1_D , ReadData2_D , PC_Out_D , Instruction_D[11:7] , imm_data_D , PCPlus4_D , MemWrite_D , ALUSrc_D , RegWrite_D , ImmSrc_D , ResultSrc_D , Operation_D , Branch_D , Jump_D , MemWrite_E , ALUSrc_E , RegWrite_E , ImmSrc_E , ResultSrc_E , Operation_E , Branch_E , Jump_E , ReadData1_E , ReadData2_E , PC_Out_E , Instruction_E , imm_data_E , PCPlus4_E);

adder pct(PC_Out_E , imm_data_E , PCTarget_E);

mux32bit2 ALUSrcMUX(ReadData2_E , imm_data_E , ALUSrc_E , SrcB_E);

alu lua(ReadData1_E , SrcB_E , Operation_E , res_E , Zero_E);

Pipe_M PM(clk , RegWrite_E , ResultSrc_E , MemWrite_E , res_E , ReadData2_E , Instruction_E , PCPlus4_E , RegWrite_M , ResultSrc_M , MemWrite_M , res_M , ReadData2_M , Instruction_M , PCPlus4_M);

Data_Memory DM(res_M, ReadData2_M, clk, MemWrite_M, Read_Data_M);

Pipe_W PW(clk , RegWrite_M , ResultSrc_M , res_M , Read_Data_M , Instruction_M , PCPlus4_M , RegWrite_W , ResultSrc_W , res_W , Read_Data_W , Instruction_W , PCPlus4_W );

CU controlunit(Instruction_D[6:0] , Instruction_D[14:12] , Instruction_D[31:25] , ResultSrc_D , MemWrite_D , ALUSrc_D , ImmSrc_D , RegWrite_D , Operation_D , Branch_D , Jump_D); 
endmodule