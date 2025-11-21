// 1. 引入你的“引擎”图纸
`include "RISC-V-Pipelined.v"

module testbench;

  // --- 变量定义 ---
  // reg: 我们(测试平台)手里的遥控器，负责发送信号
  // wire: 只有想看 CPU 内部输出时才需要 wire，这里我们主要负责驱动输入
  reg clk;
  reg rst;

  // --- 实例化 (Instantiate) ---
  // 把你的“法拉利引擎”搬到试车台上
  // .端口名(我们可以控制的信号名)
  RISC_V_Processor_Pipelined uut (
    .clk(clk),
    .rst(rst)
  );

  // --- 时钟发生器 (心脏起搏器) ---
  initial begin
    clk = 0;
    // forever 表示死循环，每隔 5ns 翻转一次电平
    // 周期 = 10ns (100MHz 频率，虽然仿真里频率不重要)
    forever #5 clk = ~clk; 
  end

  // --- 测试流程 (导演剧本) ---
  initial begin
    // 1. 开启摄像机 (记录波形)
    // 这两行是 EDA Playground 生成波形图必须的！
    $dumpfile("dump.vcd");
    $dumpvars(0, testbench); 

    // 2. 初始状态
    rst = 1; // 按下复位键 (Reset)，让 CPU 清零
    
    // 3. 启动！
    #20;     // 保持复位 20ns，确保电路稳定
    rst = 0; // 松开复位键，CPU 开始跑第一条指令！

    // 4. 观察运行
    // 你的 Instruction_Memory 里写了大概 80 行代码
    // 假设每条指令跑 5 个周期，跑 2000ns 足够看完全过程了
    #2000;   
    
    // 5. 结束测试
    $finish; 
  end

  // --- (可选) 实时打印一些关键信息到 Log 窗口 ---
  // 这就像是在试车时，每隔一会喊一声现在的转速
  always @(posedge clk) begin
    // 打印当前时间和 PC 的值
    // %d 是十进制，%h 是十六进制
    if (!rst) begin // 只有不在复位时才打印
        $display("Time: %0t | PC: %h | Result(WriteBack): %h", $time, uut.PC_Out_F, uut.Result_W);
    end
  end

endmodule
