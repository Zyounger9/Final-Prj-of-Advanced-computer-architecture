// =================================================================
// RISC-V Testbench for Bubble Sort
// 功能: 验证 CPU 能否正确执行冒泡排序汇编程序
// 显示: 自动打印排序结果到控制台 Log
// =================================================================

module testbench;

    // --- 1. 参数定义 ---
    localparam CLK_PERIOD = 10;      // 10ns 周期
    localparam CLK_HALF_PERIOD = 5;  // 5ns 半周期
    // 冒泡排序耗时较长 (10个数约需几百到一千周期)，给 2000 周期足够安全
    localparam RUN_CYCLES = 2000;    

    // --- 2. 信号定义 ---
    reg clk;
    reg rst;
    integer i;
    
    // --- 3. 实例化 CPU (Unit Under Test) ---
    RISC_V_Processor_Pipelined uut (
        .clk(clk),
        .rst(rst)
    );

    // --- 4. 时钟生成 ---
    initial begin
        clk = 0;
        forever #CLK_HALF_PERIOD clk = ~clk; 
    end

    // --- 5. 仿真主流程 ---
    initial begin
        // 开启波形记录 (如果需要查看 EPWave)
        $dumpfile("dump.vcd");
        // 记录 CPU 内部的所有信号，方便调试
        $dumpvars(0, testbench); 

        // --- 步骤 1: 复位 ---
        rst = 1;
        #(CLK_PERIOD * 2); // 保持复位 2 个周期
        rst = 0;
        
        $display("==========================================================");
        $display("   RISC-V 冒泡排序仿真开始 ");
        $display("==========================================================");
        $display("说明: ");
        $display("1. CPU 指令内存中硬编码了一个程序。");
        $display("2. 程序首先将 0-9 写入内存 (输入数据)。");
        $display("3. 然后使用冒泡排序将数据改为降序排列 (9-0)。");
        $display("----------------------------------------------------------");
        $display("正在运行 CPU (预计耗时 %0d ns)...", RUN_CYCLES * CLK_PERIOD);

        // --- 步骤 2: 等待程序执行 ---
        // 我们等待足够长的时间让汇编程序跑完
        #(RUN_CYCLES * CLK_PERIOD);

        // --- 步骤 3: 打印最终结果 ---
        // 直接读取 CPU 内部 Data_Memory 模块的 memory 数组
        // 数组起始地址是 0x200 (十进制 512)
        $display("\n==========================================================");
        $display("   仿真结束: 检查内存结果 (地址 0x200 起始)");
        $display("==========================================================");
        $display("预期结果: 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 (降序)");
        $display("实际内存:");
        
        $write("[ ");
        for (i = 0; i < 10; i = i + 1) begin
            // 注意: 我们的内存是 8 位宽的小端序 (Little Endian)
            // 读取一个 32-bit 字需要拼接 4 个字节: {MSB, ..., LSB}
            // 地址计算: Base(512) + Index*4 + ByteOffset
            $write("%0d", {
                uut.DM.memory[512 + i*4 + 3], 
                uut.DM.memory[512 + i*4 + 2], 
                uut.DM.memory[512 + i*4 + 1], 
                uut.DM.memory[512 + i*4]
            });
            
            if (i < 9) $write(", ");
        end
        $write(" ]\n");
        $display("==========================================================");

        $finish;
    end

    // --- 6. (可选) 实时监控 ---
    // 监听内存写入信号，当数据发生交换时打印出来，让你更有“实感”
    always @(posedge clk) begin
        if (uut.MemWrite_M) begin
            // 当内存写使能有效时
            // 注意: 这里的 Write_Data_M 和 res_M 是 CPU 内部信号
            // 0x200 是十进制 512
            if (uut.res_M >= 512 && uut.res_M < 552) begin
                $display("Time %0t ns: CPU 正在写入内存地址 0x%h, 写入值: %0d", 
                         $time, uut.res_M, uut.WriteData_M);
            end
        end
    end

endmodule
