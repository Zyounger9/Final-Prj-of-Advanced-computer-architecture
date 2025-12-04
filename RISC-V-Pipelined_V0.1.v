// =================================================================
// RISC-V 5-STAGE PIPELINED PROCESSOR CORE (RV32I Subset)
// Feature: Includes Forwarding Unit to solve Data Hazards
// =================================================================

// =================================================================
// 模块 1: 主控制单元 (Control Unit)
// =================================================================
module Control_Unit(opcode , Branch, Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
    input [6:0] opcode;
    output reg Branch , Jump , MemWrite , ALUSrc , RegWrite;
    output reg [1:0] ImmSrc , ALUOp ,  ResultSrc;

    always @(*) begin 
        case (opcode)
            7'b0110011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_00_xx_1_0_0_10_0; // R-Type
            7'b0000011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_01_00_1_0_0_00_0; // lw
            7'b0100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_xx_01_0_1_0_00_0; // sw
            7'b1100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_xx_10_0_0_1_01_0; // beq
            7'b0010011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_00_00_1_0_0_10_0; // addi
            7'b1101111 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_10_11_1_0_0_xx_1; // jal
            default    : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_xx_xx_x_x_x_xx_x;
        endcase
    end
endmodule

// =================================================================
// 模块 2: ALU 控制助手 (ALU Control)
// =================================================================
module ALU_Control(ALUOp , Funct3 , Funct7 , op , Operation);
    input [1:0] ALUOp;
    input [2:0] Funct3;
    input Funct7 , op; 
    output reg [2:0] Operation; 

    always @(*) begin
        case(ALUOp)
            2'b00: Operation = 3'b000;  // add (for load/store)
            2'b01: Operation = 3'b001;  // sub (for branch)
            2'b10: begin                // R-Type / I-Type
                case(Funct3)
                    3'b000: Operation = (op && Funct7) ? 3'b001 : 3'b000; // sub / add
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

// =================================================================
// 模块 3: 控制单元包装 (CU)
// =================================================================
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

// =================================================================
// 模块 4: 寄存器堆 (Register File)
// =================================================================
module registerFile (
    input wire clk,             
    input wire RegWrite,        
    input wire [4:0] RS1,        
    input wire [4:0] RS2,        
    input wire [4:0] RD,         
    input wire [31:0] WriteData, 
    output wire [31:0] ReadData1,
    output wire [31:0] ReadData2 
);
    reg [31:0] Registers [31:0];  
    
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] = i;  
    end
    
    assign ReadData1 = (RS1 != 5'b0) ? Registers[RS1] : 32'b0; 
    assign ReadData2 = (RS2 != 5'b0) ? Registers[RS2] : 32'b0;
    
    always @(negedge clk) begin
        if (RegWrite && (RD != 5'b0)) begin 
            Registers[RD] <= WriteData;  
        end
    end
endmodule

// =================================================================
// 模块 5: 算术逻辑单元 (ALU)
// =================================================================
module alu(a, b, op, res, zero);
    input [31:0] a, b;  
    input [2:0] op;     
    output reg zero;    
    output reg [31:0] res; 

    always @(*) begin
        case(op)
            3'b000: res = a + b;    
            3'b001: res = a - b;    
            3'b101: res = a < b;    
            3'b011: res = a | b;    
            3'b010: res = a & b;    
            default: res = 32'b0;
        endcase
        zero = (res == 32'b0) ? 1'b1 : 1'b0; 
    end
endmodule

// =================================================================
// 模块 6: 立即数生成器 (Imm Gen)
// =================================================================
module imm_data_gen(instruction , ImmSrc , imm_data);
    input [31:0] instruction;
    input [1:0] ImmSrc;
    output reg [31:0] imm_data;

    always @(*) begin
        case(ImmSrc)
            2'b00: imm_data = {{20{instruction[31]}}, instruction[31:20]}; // I
            2'b01: imm_data = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S
            2'b10: imm_data = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B
            2'b11: imm_data = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0}; // J
            default: imm_data = 32'b0; 
        endcase
    end
endmodule

// =================================================================
// 模块 7: 数据内存 (Data Memory)
// =================================================================
module Data_Memory(
    input wire [31:0] Mem_Addr,    
    input wire [31:0] Write_Data,  
    input wire clk,                
    input wire MemWrite,           
    output wire [31:0] Read_Data   
);
    reg [7:0] memory [1023:0]; 

    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            memory[i] = 0; 
    end

    assign Read_Data = {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]};

    always @(posedge clk) begin
        if (MemWrite) begin
            {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]} = Write_Data;
        end
    end
endmodule

// =================================================================
// 模块 8: 加法器 (Adder)
// =================================================================
module adder (a , b , out);
    input [31:0] a , b;
    output [31:0] out;
    assign out = a + b ; 
endmodule

// =================================================================
// 模块 9: 指令内存 (Instruction Memory)
// =================================================================
module Instruction_Memory(Inst_Address, Instruction); 
    input wire [31:0] Inst_Address;    
    output reg [31:0] Instruction;     
    reg [7:0] memory [199:0]; 

    initial begin
        // --- 初始化部分 ---
        // 0x00: addi x22, x0, 0 (i=0)
        memory[0]=8'h13; memory[1]=8'h0B; memory[2]=8'h00; memory[3]=8'h00;
        // 0x04: addi x23, x0, 0 (j=0)
        memory[4]=8'h93; memory[5]=8'h0B; memory[6]=8'h00; memory[7]=8'h00;
        // 0x08: addi x10, x0, 10 (limit=10)
        memory[8]=8'h13; memory[9]=8'h05; memory[10]=8'hA0; memory[11]=8'h00;

        // --- Loop 1: 初始化数组 (地址 0x0C) ---
        // 0x0C: slli x24, x22, 2
        memory[12]=8'h13; memory[13]=8'h1C; memory[14]=8'h2B; memory[15]=8'h00;
        // 0x10: sw x22, 0x200(x24)  (Mem[512+i*4] = i)
        memory[16]=8'h23; memory[17]=8'h20; memory[18]=8'h6C; memory[19]=8'h21;
        // 0x14: addi x22, x22, 1
        memory[20]=8'h13; memory[21]=8'h0B; memory[22]=8'h1B; memory[23]=8'h00;
        // 0x18: bne x22, x10, -12 (Loop1)
        memory[24]=8'hE3; memory[25]=8'h9A; memory[26]=8'hAB; memory[27]=8'hFE;

        // --- Loop 2: 外层循环 (地址 0x1C) ---
        // 0x1C: addi x22, x0, 0 (i=0)
        memory[28]=8'h13; memory[29]=8'h0B; memory[30]=8'h00; memory[31]=8'h00;
        // 0x20: slli x24, x22, 2
        memory[32]=8'h13; memory[33]=8'h1C; memory[34]=8'h2B; memory[35]=8'h00;
        // 0x24: add x23, x22, x0 (j=i)
        memory[36]=8'h33; memory[37]=8'h0B; memory[38]=8'h0B; memory[39]=8'h00;

        // --- Loop 3: 内层循环 (地址 0x28) ---
        // 0x28: slli x25, x23, 2
        memory[40]=8'h93; memory[41]=8'h9C; memory[42]=8'h2B; memory[43]=8'h00;
        // 0x2C: lw x1, 0x200(x24) (a[i])
        memory[44]=8'h83; memory[45]=8'h20; memory[46]=8'h0C; memory[47]=8'h20;
        // 0x30: lw x2, 0x200(x25) (a[j])
        memory[48]=8'h03; memory[49]=8'h21; memory[50]=8'h0C; memory[51]=8'h20;
        // 0x34: bge x1, x2, 12 (EndIf)
        memory[52]=8'h63; memory[53]=8'hD6; memory[54]=8'h20; memory[55]=8'h00;
        
        // --- 交换 (Swap, 地址 0x38) ---
        // 0x38: add x5, x1, x0 (temp = a[i])
        memory[56]=8'hB3; memory[57]=8'h02; memory[58]=8'h10; memory[59]=8'h00;
        // 0x3C: sw x2, 0x200(x24) (a[i] = a[j])
        memory[60]=8'h23; memory[61]=8'h20; memory[62]=8'h2C; memory[63]=8'h20;
        // 0x40: sw x5, 0x200(x25) (a[j] = temp)
        memory[64]=8'h23; memory[65]=8'h20; memory[66]=8'h5C; memory[67]=8'h20;

        // --- EndIf & 循环控制 (地址 0x44) ---
        // 0x44: addi x23, x23, 1
        memory[68]=8'h93; memory[69]=8'h8B; memory[70]=8'h1B; memory[71]=8'h00;
        // 0x48: bne x23, x10, -32 (Loop3) -> 跳回 0x28
        memory[72]=8'hE3; memory[73]=8'h10; memory[74]=8'hAB; memory[75]=8'hFE;
        // 0x4C: addi x22, x22, 1
        memory[76]=8'h13; memory[77]=8'h0B; memory[78]=8'h1B; memory[79]=8'h00;
        // 0x50: bne x22, x10, -48 (Loop2) -> 跳回 0x20
        memory[80]=8'hE3; memory[81]=8'h10; memory[82]=8'hAB; memory[83]=8'hFD;
        
        // 0x54: 无限循环，程序停止
        memory[84]=8'h6F; memory[85]=8'hF0; memory[86]=8'h00; memory[87]=8'h00; 
    end

    always @(*) begin
        Instruction = {memory[Inst_Address + 3], memory[Inst_Address + 2], memory[Inst_Address + 1], memory[Inst_Address]};
    end 
endmodule

// =================================================================
// 模块 10: 多路选择器 (Mux)
// =================================================================
// 3选1 Mux (复用，也用于前递)
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

// 2选1 Mux
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

// =================================================================
// 模块 11: 程序计数器 (PC)
// =================================================================
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

// =================================================================
// 新增模块 12: 前递单元 (Forwarding Unit)
// 作用: 解决 RAW 数据冒险
// =================================================================
module Forwarding_Unit(
    input [4:0] RS1_E,    
    input [4:0] RS2_E,    
    input [4:0] RD_M,     
    input RegWrite_M,     
    input [4:0] RD_W,     
    input RegWrite_W,     
    output reg [1:0] ForwardA, 
    output reg [1:0] ForwardB 
);
    always @(*) begin
        // ForwardA (RS1)
        if (RegWrite_M && (RD_M != 0) && (RD_M == RS1_E)) 
            ForwardA = 2'b10; // 从 MEM 阶段前递
        else if (RegWrite_W && (RD_W != 0) && (RD_W == RS1_E)) 
            ForwardA = 2'b01; // 从 WB 阶段前递
        else 
            ForwardA = 2'b00; // 不前递

        // ForwardB (RS2)
        if (RegWrite_M && (RD_M != 0) && (RD_M == RS2_E)) 
            ForwardB = 2'b10;
        else if (RegWrite_W && (RD_W != 0) && (RD_W == RS2_E)) 
            ForwardB = 2'b01;
        else 
            ForwardB = 2'b00;
    end
endmodule

// =================================================================
// 流水线寄存器组 (Pipeline Registers)
// =================================================================
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

// 【修改】Pipe_E: 增加了 RS1_D, RS2_D 输入和 RS1_E, RS2_E 输出
module Pipe_E(
    input clk,
    input [31:0] ReadData1_D, ReadData2_D, PC_Out_D, imm_data_D, PCPlus4_D,
    input [4:0] RD_D, RS1_D, RS2_D, // 新增 RS1, RS2
    input MemWrite_D, ALUSrc_D, RegWrite_D,
    input [1:0] ImmSrc_D, ResultSrc_D,
    input [2:0] Operation_D,
    input Branch_D, Jump_D,
    
    output reg MemWrite_E, ALUSrc_E, RegWrite_E,
    output reg [1:0] ImmSrc_E, ResultSrc_E,
    output reg [2:0] Operation_E,
    output reg Branch_E, Jump_E,
    output reg [31:0] ReadData1_E, ReadData2_E, PC_Out_E, imm_data_E, PCPlus4_E,
    output reg [4:0] RD_E, RS1_E, RS2_E // 新增 RS1, RS2
);
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
        RS1_E <= RS1_D; // 传递源寄存器地址
        RS2_E <= RS2_D;
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

// =================================================================
// 模块 13: 顶层模块 (RISC_V_Processor_Pipelined)
// =================================================================
module RISC_V_Processor_Pipelined(clk , rst);
    input clk , rst;
    
    wire [31:0] PCPlus4_F , PCPlus4_D , PCPlus4_E , PCPlus4_W  , PCPlus4_M , PCin_F , PCTarget_E , PC_Out_F ,  PC_Out_D , PC_Out_E , Instruction_F , Instruction_D , Result_W , ReadData1_D , ReadData2_D , ReadData1_E , ReadData2_E , ReadData2_M , res_E , res_M , res_W , Read_Data_M ,  Read_Data_W , imm_data_D , imm_data_E , SrcB_E;
    wire [4:0] Instruction_W , Instruction_E , Instruction_M;
    wire [2:0] Operation_D , Operation_E;
    wire [1:0]  ImmSrc_D , ImmSrc_E , ResultSrc_W , ResultSrc_D , ResultSrc_E , ResultSrc_M;
    wire  RegWrite_D , RegWrite_E , RegWrite_M  , RegWrite_W  ,  PCSrc_E , ALUSrc_D , ALUSrc_E , Zero_E , Branch_E , Branch_D , Jump_E , Jump_D , MemWrite_D , MemWrite_E , MemWrite_M ;

    // --- 前递单元相关信号 ---
    wire [1:0] ForwardA_E, ForwardB_E;
    wire [31:0] SrcA_E_Final, WriteData_E_Final;
    wire [4:0] RS1_E, RS2_E; // 从 Pipe_E 出来的源寄存器号

    // IF Stage
    adder pcp4(PC_Out_F , 32'b100 , PCPlus4_F);
    assign PCSrc_E = Jump_E | (Zero_E & Branch_E);
    mux32bit2 PCSrcMUX(PCPlus4_F , PCTarget_E , PCSrc_E , PCin_F);
    Program_Counter PC(clk , rst , PCin_F , PC_Out_F);
    Instruction_Memory IM(PC_Out_F , Instruction_F);
    Pipe_D PD(clk , Instruction_F , PC_Out_F  , PCPlus4_F , Instruction_D , PC_Out_D , PCPlus4_D);

    // ID Stage
    mux32bit ResultSrcMUX(res_W , Read_Data_W , PCPlus4_W , ResultSrc_W , Result_W);
    registerFile RF(clk , RegWrite_W , Instruction_D[19:15] , Instruction_D[24:20] , Instruction_W , Result_W , ReadData1_D , ReadData2_D);
    imm_data_gen ID(Instruction_D , ImmSrc_D , imm_data_D);
    CU controlunit(Instruction_D[6:0] , Instruction_D[14:12] , Instruction_D[31:25] , ResultSrc_D , MemWrite_D , ALUSrc_D , ImmSrc_D , RegWrite_D , Operation_D , Branch_D , Jump_D); 

    // ID -> EX Pipeline Register (包含 RS1, RS2 传递)
    Pipe_E PE(
        .clk(clk), 
        .ReadData1_D(ReadData1_D), .ReadData2_D(ReadData2_D), .PC_Out_D(PC_Out_D), .RD_D(Instruction_D[11:7]), .imm_data_D(imm_data_D), .PCPlus4_D(PCPlus4_D), 
        .RS1_D(Instruction_D[19:15]), .RS2_D(Instruction_D[24:20]), // 输入 RS1, RS2
        .MemWrite_D(MemWrite_D), .ALUSrc_D(ALUSrc_D), .RegWrite_D(RegWrite_D), .ImmSrc_D(ImmSrc_D), .ResultSrc_D(ResultSrc_D), .Operation_D(Operation_D), .Branch_D(Branch_D), .Jump_D(Jump_D), 
        .MemWrite_E(MemWrite_E), .ALUSrc_E(ALUSrc_E), .RegWrite_E(RegWrite_E), .ImmSrc_E(ImmSrc_E), .ResultSrc_E(ResultSrc_E), .Operation_E(Operation_E), .Branch_E(Branch_E), .Jump_E(Jump_E), 
        .ReadData1_E(ReadData1_E), .ReadData2_E(ReadData2_E), .PC_Out_E(PC_Out_E), .RD_E(Instruction_E), .imm_data_E(imm_data_E), .PCPlus4_E(PCPlus4_E),
        .RS1_E(RS1_E), .RS2_E(RS2_E) // 输出 RS1, RS2
    );

    // EX Stage
    adder pct(PC_Out_E , imm_data_E , PCTarget_E);

    // --- 前递 MUX ---
    // 选择 ALU 操作数 A (ForwardA 控制)
    mux32bit SrcAMux (ReadData1_E, Result_W, res_M, ForwardA_E, SrcA_E_Final);
    // 选择 ALU 操作数 B 的预备值 (ForwardB 控制，用于 Store 数据)
    mux32bit SrcBMux (ReadData2_E, Result_W, res_M, ForwardB_E, WriteData_E_Final);
    // 最终 ALU 操作数 B (ALUSrc 控制)
    mux32bit2 ALUSrcMUX(WriteData_E_Final , imm_data_E , ALUSrc_E , SrcB_E);

    // 实例化前递单元
    Forwarding_Unit FU (
        .RS1_E(RS1_E), .RS2_E(RS2_E), 
        .RD_M(RegWrite_M ? Instruction_M : 5'b0), // 确保只有写使能时才检查 RD
        .RegWrite_M(RegWrite_M), 
        .RD_W(RegWrite_W ? Instruction_W : 5'b0), 
        .RegWrite_W(RegWrite_W), 
        .ForwardA(ForwardA_E), .ForwardB(ForwardB_E)
    );

    // ALU 运算 (使用前递后的数据)
    alu lua(SrcA_E_Final , SrcB_E , Operation_E , res_E , Zero_E);

    // EX -> MEM Pipeline Register
    Pipe_M PM(clk , RegWrite_E , ResultSrc_E , MemWrite_E , res_E , WriteData_E_Final , Instruction_E , PCPlus4_E , RegWrite_M , ResultSrc_M , MemWrite_M , res_M , WriteData_M , Instruction_M , PCPlus4_M);

    // MEM Stage
    Data_Memory DM(res_M, WriteData_M, clk, MemWrite_M, Read_Data_M);

    // MEM -> WB Pipeline Register
    Pipe_W PW(clk , RegWrite_M , ResultSrc_M , res_M , Read_Data_M , Instruction_M , PCPlus4_M , RegWrite_W , ResultSrc_W , res_W , Read_Data_W , Instruction_W , PCPlus4_W );

    // WB Stage (逻辑闭环)
endmodule
