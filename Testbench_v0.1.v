`include "RISC-V-Pipelined.v"

module testbench;
    // --- 信号定义 ---
    reg clk;
    reg rst;
    integer i; // 用于循环打印内存

    // --- 实例化 CPU (Device Under Test) ---
    RISC_V_Processor_Pipelined uut (
        .clk(clk),
        .rst(rst)
    );

    // --- 时钟生成 (每 5ns 翻转一次，周期 10ns) ---
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // --- 仿真主流程 ---
    initial begin
        // 1. 【关键】开启波形记录
        // 如果没有这两行，EDA Playground 的示波器就不会显示任何图像
        $dumpfile("dump.vcd");   // 指定波形文件名
        $dumpvars(0, testbench); // 记录 testbench 模块下的所有信号

        // 2. 复位序列
        rst = 1;    // 按下复位键，让 CPU 初始化
        #20;        // 保持 20ns
        rst = 0;    // 松开复位键，CPU 开始运行
        $display("--- 仿真开始: RISC-V 冒泡排序程序 ---");

        // 3. 运行仿真
        // 冒泡排序大约需要 N^2 次指令。对于 10 个数，3500ns (350个周期) 足够跑完。
        // 如果你只跑 50ns，波形会有，但排序还没开始就结束了。
        #3500; 

        // 4. 验证结果：检查数据内存 (Data Memory)
        // 数组起始地址是 0x200 (十进制 512)
        $display("\n--- 最终结果 (内存地址 0x200 - 0x224) ---");
        $display("预期结果: 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 (降序)");
        $display("实际结果:");
        
        for (i = 0; i < 10; i = i + 1) begin
            // 直接读取 uut (CPU实例) -> DM (数据内存模块) -> memory (内存数组)
            // 地址计算: 512 + i*4
            // 注意: 我们的内存是 8 位宽的小端序，所以需要把 4 个字节拼接起来显示
            $display("Addr 0x%h: %d", 
                512 + i*4,
                {uut.DM.memory[512 + i*4 + 3], 
                 uut.DM.memory[512 + i*4 + 2], 
                 uut.DM.memory[512 + i*4 + 1], 
                 uut.DM.memory[512 + i*4]}
            );
        end

        $display("\n--- 仿真结束 ---");
        
        // 5. 停止仿真
        $finish;
    end
    
    // --- (可选) 实时监视器 ---
    // 在 Log 中打印每一步的 PC 和 指令，方便没看波形时也能知道程序在跑
    always @(posedge clk) begin
        if (!rst) begin
            // 只有当 PC 发生变化时才打印，或者每隔 100ns 打印一次，防止 Log 刷屏太快
            // 这里为了简单，我们只在特定的 PC 地址打印（例如循环开头）
            // 或者你可以选择注释掉下面这行，保持 Log 清爽
            // $display("Time: %0t | PC: %h", $time, uut.PC_Out_F);
        end
    end

endmodule
