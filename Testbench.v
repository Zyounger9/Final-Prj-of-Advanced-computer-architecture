`include "RISC_V_Pipelined.v"

module testbench;
    reg clk;
    reg rst;
    integer i;

    // 实例化 CPU (Device Under Test)
    RISC_V_Processor_Pipelined uut (
        .clk(clk),
        .rst(rst)
    );

    // 时钟生成器 (Clock Generator)
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 周期 10ns
    end

    // 仿真控制逻辑
    initial begin
        // 1. 开启波形记录
        $dumpfile("dump.vcd");
        $dumpvars(0, testbench);

        // 2. 复位序列
        rst = 1;    // 保持复位
        #20;        // 20ns
        rst = 0;    // 启动 CPU
        $display("--- 仿真开始: RISC-V 冒泡排序程序 ---");

        // 3. 运行足够长的时间 (等待程序运行完成)
        // 冒泡排序 N=10 需要大约 N^2 次循环，估计 350 个周期足够。
        #3500; 

        // 4. 验证结果：读取 Data_Memory
        // 数组起始地址是 0x200 (十进制 512)
        $display("\n--- 最终结果 (地址 0x200 - 0x224) ---");
        $display("预期结果: 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 (降序)");
        $display("实际结果:");
        
        for (i = 0; i < 10; i = i + 1) begin
            // 直接访问 uut 实例内部的 Data_Memory 模块 (DM) 
            // 内存地址 = 512 + i*4
            // 拼接 4 个字节，并显示为十进制
            $display("Addr 0x%h: %d", 
                512 + i*4,
                {uut.DM.memory[512 + i*4 + 3], 
                 uut.DM.memory[512 + i*4 + 2], 
                 uut.DM.memory[512 + i*4 + 1], 
                 uut.DM.memory[512 + i*4]}
            );
        end

        $display("\n--- 仿真结束 ---");
        $finish;
    end
    
    // (可选) 实时打印追踪信息
    always @(posedge clk) begin
        if (!rst) begin
            // 打印 PC 和指令，用于追踪流水线
            $display("Time: %0t | PC: %h | Instruction: %h", 
                $time, 
                uut.PC_Out_F, 
                uut.Instruction_F
            );
        end
    end
endmodule
