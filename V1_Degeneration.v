`timescale 1ns / 1ps

// --- 1. ALU Module ---
module alu (
    input logic [31:0] a, b,
    input logic [3:0]  alu_control,
    output logic [31:0] result,
    output logic zero
);
    always_comb begin
        case (alu_control)
            4'b0000: result = a + b;       // ADD
            4'b1000: result = a - b;       // SUB
            4'b0111: result = a & b;       // AND
            4'b0110: result = a | b;       // OR
            4'b0010: result = (signed'(a) < signed'(b)) ? 32'd1 : 32'd0; // SLT
            4'b0001: result = b << a[4:0]; // SLL
            default: result = 0;
        endcase
        zero = (result == 0);
    end
endmodule

// --- 2. Register File ---
module reg_file (
    input logic clk,
    input logic we3,
    input logic [4:0] a1, a2, a3,
    input logic [31:0] wd3,
    output logic [31:0] rd1, rd2
);
    logic [31:0] rf [31:0];
    
    // 写逻辑
    always_ff @(posedge clk) begin
        if (we3 && a3 != 0) rf[a3] <= wd3;
    end

    // 读逻辑 (x0 恒为 0)
    assign rd1 = (a1 != 0) ? rf[a1] : 0;
    assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

// --- 3. RISC-V SoC (CPU + Memory) ---
module riscv_soc (
    input logic clk,
    input logic reset
);
    // 内存定义
    logic [31:0] imem [0:63]; // 指令内存 (ROM)
    logic [31:0] dmem [0:63]; // 数据内存 (RAM)

    // CPU 内部信号
    logic [31:0] pc, next_pc;
    logic [31:0] instr;
    logic [31:0] dmem_addr, dmem_wdata, dmem_rdata;
    
    // 流水线寄存器
    logic [31:0] if_id_pc, if_id_instr;
    logic [31:0] id_ex_pc, id_ex_rs1_val, id_ex_rs2_val, id_ex_imm;
    logic [4:0]  id_ex_rs1, id_ex_rs2, id_ex_rd;
    logic [3:0]  id_ex_alu_ctrl;
    logic        id_ex_reg_write, id_ex_mem_write, id_ex_mem_read, id_ex_alusrc, id_ex_branch;
    logic [2:0]  id_ex_funct3;
    logic [31:0] ex_mem_alu_res, ex_mem_rs2_val;
    logic [4:0]  ex_mem_rd;
    logic        ex_mem_reg_write, ex_mem_mem_write, ex_mem_mem_read;
    logic [31:0] mem_wb_read_data, mem_wb_alu_res;
    logic [4:0]  mem_wb_rd;
    logic        mem_wb_reg_write, mem_wb_mem_read;

    // 辅助信号
    logic [31:0] rd1, rd2, imm_ext;
    logic [31:0] src_a, src_b, src_b_temp, alu_result, result_wb;
    logic        zero_flag, pc_src;
    logic [31:0] branch_target;
    logic [1:0]  forward_a, forward_b;

    // ==========================================
    // 1. IF Stage (取指)
    // ==========================================
    always_ff @(posedge clk or posedge reset) begin
        if (reset) pc <= 0;
        else pc <= next_pc;
    end
    
    assign next_pc = (pc_src) ? branch_target : pc + 4;
    assign instr = imem[pc[7:2]]; 

    always_ff @(posedge clk) begin
        if (reset || pc_src) begin // Flush
            if_id_pc <= 0; if_id_instr <= 0;
        end else begin
            if_id_pc <= pc; if_id_instr <= instr;
        end
    end

    // ==========================================
    // 2. ID Stage (译码)
    // ==========================================
    logic reg_write_d, mem_write_d, mem_read_d, alusrc_d, branch_d;
    logic [3:0] alu_ctrl_d;
    logic [6:0] opcode = if_id_instr[6:0];
    logic [2:0] funct3 = if_id_instr[14:12];
    logic [6:0] funct7 = if_id_instr[31:25];

    // 控制单元
    always_comb begin
        {reg_write_d, mem_write_d, mem_read_d, alusrc_d, branch_d, alu_ctrl_d} = 0;
        case (opcode)
            7'b0110011: begin // R-type
                reg_write_d = 1; 
                alu_ctrl_d = (funct3==0 && funct7[5]) ? 4'b1000 : (funct3==2) ? 4'b0010 : 4'b0000; 
            end
            7'b0010011: begin reg_write_d = 1; alusrc_d = 1; alu_ctrl_d = 4'b0000; end // ADDI
            7'b0000011: begin reg_write_d = 1; alusrc_d = 1; mem_read_d = 1; end       // LW
            7'b0100011: begin mem_write_d = 1; alusrc_d = 1; end                       // SW
            7'b1100011: begin branch_d = 1; alu_ctrl_d = 4'b1000; end                  // BEQ (用SUB比较)
        endcase
    end

    // 立即数生成
    assign imm_ext = (opcode == 7'b0100011) ? {{20{if_id_instr[31]}}, if_id_instr[31:25], if_id_instr[11:7]} :
                     (opcode == 7'b1100011) ? {{20{if_id_instr[31]}}, if_id_instr[7], if_id_instr[30:25], if_id_instr[11:8], 1'b0} :
                                              {{20{if_id_instr[31]}}, if_id_instr[31:20]};

    reg_file rf (.clk(clk), .we3(mem_wb_reg_write), .a1(if_id_instr[19:15]), .a2(if_id_instr[24:20]), .a3(mem_wb_rd), .wd3(result_wb), .rd1(rd1), .rd2(rd2));

    always_ff @(posedge clk) begin
        if (reset || pc_src) begin
            id_ex_pc<=0; id_ex_reg_write<=0; id_ex_mem_write<=0; id_ex_mem_read<=0; id_ex_branch<=0;
        end else begin
            id_ex_pc<=if_id_pc; id_ex_rs1<=if_id_instr[19:15]; id_ex_rs2<=if_id_instr[24:20]; id_ex_rd<=if_id_instr[11:7];
            id_ex_rs1_val<=rd1; id_ex_rs2_val<=rd2; id_ex_imm<=imm_ext; id_ex_alu_ctrl<=alu_ctrl_d;
            id_ex_reg_write<=reg_write_d; id_ex_mem_write<=mem_write_d; id_ex_mem_read<=mem_read_d; 
            id_ex_alusrc<=alusrc_d; id_ex_branch<=branch_d; id_ex_funct3<=funct3;
        end
    end

    // ==========================================
    // 3. EX Stage (执行)
    // ==========================================
    // Forwarding Unit
    always_comb begin
        forward_a = 0; forward_b = 0;
        if (ex_mem_reg_write && ex_mem_rd!=0 && ex_mem_rd==id_ex_rs1) forward_a = 2'b10;
        else if (mem_wb_reg_write && mem_wb_rd!=0 && mem_wb_rd==id_ex_rs1) forward_a = 2'b01;
        
        if (ex_mem_reg_write && ex_mem_rd!=0 && ex_mem_rd==id_ex_rs2) forward_b = 2'b10;
        else if (mem_wb_reg_write && mem_wb_rd!=0 && mem_wb_rd==id_ex_rs2) forward_b = 2'b01;
    end

    assign src_a = (forward_a==2'b10) ? ex_mem_alu_res : (forward_a==2'b01) ? result_wb : id_ex_rs1_val;
    assign src_b_temp = (forward_b==2'b10) ? ex_mem_alu_res : (forward_b==2'b01) ? result_wb : id_ex_rs2_val;
    assign src_b = (id_ex_alusrc) ? id_ex_imm : src_b_temp;

    alu my_alu (.a(src_a), .b(src_b), .alu_control(id_ex_alu_ctrl), .result(alu_result), .zero(zero_flag));

    logic take_b;
    assign take_b = (id_ex_funct3==3'b000) ? zero_flag : !zero_flag; 
    assign pc_src = id_ex_branch && take_b;
    assign branch_target = id_ex_pc + id_ex_imm;

    always_ff @(posedge clk) begin
        if (reset) begin ex_mem_reg_write<=0; ex_mem_mem_write<=0; end
        else begin
            ex_mem_alu_res<=alu_result; ex_mem_rs2_val<=src_b_temp; ex_mem_rd<=id_ex_rd;
            ex_mem_reg_write<=id_ex_reg_write; ex_mem_mem_write<=id_ex_mem_write; ex_mem_mem_read<=id_ex_mem_read;
        end
    end

    // ==========================================
    // 4. MEM Stage (访存)
    // ==========================================
    assign dmem_addr = ex_mem_alu_res;
    assign dmem_wdata = ex_mem_rs2_val;
    
    // DMEM (Reset时初始化，防止多重驱动)
    always_ff @(posedge clk) begin
        if (reset) begin
            dmem[0] <= 32'd5; 
            dmem[1] <= 32'd2; 
            dmem[2] <= 32'd9; 
            dmem[3] <= 32'd1; 
            dmem[4] <= 32'd3;
        end else if (ex_mem_mem_write) begin
            dmem[dmem_addr[7:2]] <= dmem_wdata;
        end
    end
    assign dmem_rdata = dmem[dmem_addr[7:2]];

    always_ff @(posedge clk) begin
        if (reset) mem_wb_reg_write<=0;
        else begin
            mem_wb_read_data<=dmem_rdata; mem_wb_alu_res<=ex_mem_alu_res; mem_wb_rd<=ex_mem_rd;
            mem_wb_reg_write<=ex_mem_reg_write; mem_wb_mem_read<=ex_mem_mem_read;
        end
    end

    // ==========================================
    // 5. WB Stage (写回)
    // ==========================================
    assign result_wb = (mem_wb_mem_read) ? mem_wb_read_data : mem_wb_alu_res;

    // ==========================================
    // 6. 机器码初始化 (Machine Code)
    // ==========================================
    initial begin
        // 指令序列：实现冒泡排序的核心交换逻辑
        // 目标：将 dmem[0](5) 和 dmem[1](2) 交换
        
        // 0. x14 = 0 (指针1)
        imem[0] = 32'h00000713; 
        
        // 1. x15 = 4 (指针2)
        imem[1] = 32'h00400793; 
        
        // 2. x6 = dmem[0] -> 读取到 5
        imem[2] = 32'h00072303;
        
        // 3. x7 = dmem[1] -> 读取到 2
        imem[3] = 32'h0007a383;
        
        // 4. NOP (关键！解决 Load-Use Hazard)
        // 因为没有 Stall 机制，必须等待 MEM 阶段数据回来
        imem[4] = 32'h00000013; 
        
        // 5. slt x5, x7, x6 -> 2 < 5 ? x5=1 : x5=0
        imem[5] = 32'h0063a2b3;
        
        // 6. beq x5, x0, SKIP -> 如果不需交换则跳转 (offset=12)
        imem[6] = 32'h00028663; 
        
        // 7. sw x7, 0(x14) -> 将 2 写入 dmem[0]
        imem[7] = 32'h00772023;
        
        // 8. sw x6, 0(x15) -> 将 5 写入 dmem[1]
        imem[8] = 32'h0067a023;
        
        // 9. NOP (结束)
        imem[9] = 32'h00000013;

        // 填充剩余内存
        for (int k=10; k<64; k++) imem[k] = 32'd0;
    end

endmodule
