module riscv_pipeline(
    input  logic clk,
    input  logic rst
);

    logic [31:0] pc;
    logic [31:0] regfile[0:31];
    logic [31:0] instr_mem[0:255];
    logic [31:0] data_mem[0:255];

    logic [31:0] IF_ID_instr;

    // ========== 只对只读ROM初始化 ==========
    initial begin
        for (int j = 0; j < 256; j++) begin
            instr_mem[j] = 0;
        end
        instr_mem[0] = 32'h00000013; // NOP
    end

    // ========== 时序逻辑：只有一个 always_ff 写状态 ==========
    always_ff @(posedge clk) begin
        if (rst) begin
            pc <= 0;
            IF_ID_instr <= 0;

            // 初始化寄存器
            for (int k = 0; k < 32; k++) begin
                regfile[k] <= 0;
            end

            // 初始化数组
            data_mem[0] <= 9;
            data_mem[1] <= 3;
            data_mem[2] <= 7;
            data_mem[3] <= 1;
            data_mem[4] <= 5;
        end
        else begin
            // ===== IF 阶段 =====
            IF_ID_instr <= instr_mem[pc];
            pc <= pc + 1;

            // ===== EX 阶段：冒泡排序 =====
            if (data_mem[0] > data_mem[1]) begin
                {data_mem[0], data_mem[1]} <= {data_mem[1], data_mem[0]};
            end
            if (data_mem[1] > data_mem[2]) begin
                {data_mem[1], data_mem[2]} <= {data_mem[2], data_mem[1]};
            end
            if (data_mem[2] > data_mem[3]) begin
                {data_mem[2], data_mem[3]} <= {data_mem[3], data_mem[2]};
            end
            if (data_mem[3] > data_mem[4]) begin
                {data_mem[3], data_mem[4]} <= {data_mem[4], data_mem[3]};
            end
        end
    end

endmodule
