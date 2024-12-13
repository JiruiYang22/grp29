# 大实验报告
> group 29
---

# 1. 实验目标概述
这次大实验的目标是搭建一个基于RISCV指令集，支持cache(icache和dcache分离)，页表和中断异常处理的五级流水线，虽然cache能够作为高速缓存利用程序的时间和空间的局部性一定程度上提高程序的运行效率，但是由于页表和中断异常处理的存在，虽然两者提高了cpu的可用性和健壮性，但是仍然避免不了页表的频繁多次访存操作，大大拉慢了cpu的运行速度，所以为了缓解页表等机制带来的CPI增大问题，我们实现了提前跳转指令的时机，提高时钟频率，实现TLB以减小内存访问等优化方法。

# 2. 功能模块设计
## 2.1 CPU
如图所示，我们省略了controller的绘制，实际上controller是所有模块的控制中心，与所有模块相连。图中包含IF,ID,EXE,MEM,WB五级流水线，还有ALU和Regfile作为辅助模块，这些构成了基础部分。其他组件对应了中断、页表、缓存的内容。具体描述见下。
![](https://pic.imgdb.cn/item/675bd6e4d0e0a243d4e3588d.png)

## 2.2 Pipeline
我们实现的是标准的五级流水线，即IF,ID,EXE,MEM,WB。流水线之间用段寄存器连接；每段流水线用stall_i和bubble_i控制是否停顿或气泡。
首先考虑控制信号的设计。我们用stall_o来唯一控制stall_i和bubble_i，这里的stall_o是一个5位的信号，分别对应5个段是否需要停顿。从WB段向前考虑，如果一个段需要停顿，那么这个段之前的所有段均停顿，紧随这个段之后的段需要插入气泡。例如，当stall_o[3]为1时(stall_o[4]不为1)，那么stall_i = 5'b01111, bubble_i = 5'b10000。实现如下：
```
casez (stall_o)
    5'b1????: begin
    stall_i = 5'b11111;
    bubble_i = 5'b00000;
    end
    5'b01???: begin
    stall_i = 5'b01111;
    bubble_i = 5'b10000;
    end
    5'b001??: begin
    stall_i = 5'b00111;
    bubble_i = 5'b01000;
    end
    5'b0001?: begin
    stall_i = 5'b00011;
    bubble_i = 5'b00100;
    end
    5'b00001: begin
    stall_i = 5'b00001;
    bubble_i = 5'b00010;
    end
    default: begin
    stall_i = 5'b00000;
    bubble_i = 5'b00000;
    end
endcase
```
当一个段的stall_i为1时，说明需要停顿，则该段的指令将被暂停，保持段寄存器不变，直到stall_i为0，说明该段的指令可以继续执行。当一个段的bubble_i为1时，说明需要插入气泡（表示为NOP指令）。

**IF:**
该阶段主要负责取指令。在实现icache前，只需要直接连接wishbone接口即可。实现了icache后，需要将控制信号送入icache，其他操作与直接访存一致。如果没有被stall，则给出read_enable信号开始取指令，同时设置stall_o_0为1，表示该段需要停顿等待取指。这里的pc从ID阶段传入，我们的逻辑是所有的pc更改均在ID阶段实现。如果icache命中，则可以一周期读取指令，将结果放入instruction_if_id段寄存器，更新pc_now；否则我等待icache_ack的到来。
```
if(!stall_i_0)begin
    read_enable <= 1;
    stall_o_0 <= 1;
end else begin
    if(icache_ack)begin
        stall_o_0 <= 0;
        read_enable <= 0;
        if_id_inst_instr <= data_in;
        if_id_inst_pc <= pc;
        pc_now <= pc;
    end
end
```
段寄存器如下：
```
typedef struct packed {
    logic [31:0] pc;
    logic [31:0] instr;
} instruction_if_id;
```

**ID:**
该阶段实现了指令译码，数据旁路和pc更新。
- 指令译码无需赘述，我根据不同类型的指令完成翻译，并将相关的内容放入段寄存器instruction_id_exe.
```
typedef struct packed {
    logic exe_en;
    logic mem_en;
    logic wb_en;
    logic [31:0] instr;
    logic [31:0] pc;
    op_t op;
    logic [31:0] rs1;
    logic [31:0] rs2;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd;
    logic [31:0] imm;
    logic [3:0] alu_op;
    logic imm_type;
} instruction_id_exe;
```
- 数据旁路考虑3种情况：上一条指令经过EXE阶段的结果是本条指令需要的，则用数据旁路前传；上上条指令经过MEM阶段取到的结果是本条指令需要的，则用数据旁路前传；上一条指令MEM阶段取到的结果是本条指令需要的，则**不能**用数据旁路前传，需要暂停流水线等待MEM阶段的结果。具体阐述见2.3节。
- PC更新考虑三种情况：
    - 普通指令，且不是气泡NOP：pc = pc_now + 4;
    - 气泡NOP：保持pc不变；
    - 跳转指令：根据不同的跳转指令，实现相应的跳转逻辑。这里需要考虑数据旁路传入的数据，否则可能会跳转错误，而前面说了有不能用数据旁路前传的情况。在这种情况下，需要暂停流水线，在暂停期间不更新pc，直到MEM阶段的结果到来。具体实现如下：
```
if(id_exe_inst.op == BEQ && !stall_o_1) begin // BEQ
    if(rf_rdata_a_reg == rf_rdata_b_reg)begin
        imm = $signed({id_exe_inst.instr[31],id_exe_inst.instr[7],id_exe_inst.instr[30:25],id_exe_inst.instr[11:8],1'b0});
        pc = id_exe_inst.pc + imm;
    end else begin
        pc = pc_now + 4;
    end
end else if(id_exe_inst.op == BNE && !stall_o_1) begin // BNE
    if(rf_rdata_a_reg!= rf_rdata_b_reg)begin
        imm = $signed({id_exe_inst.instr[31],id_exe_inst.instr[7],id_exe_inst.instr[30:25],id_exe_inst.instr[11:8],1'b0});
        pc = id_exe_inst.pc + imm;
    end else begin
        pc = pc_now + 4;
    end
end else if(id_exe_inst.op == JAL && !stall_o_1) begin // JAL
    imm = $signed({{11{id_exe_inst.instr[31]}},id_exe_inst.instr[31],id_exe_inst.instr[19:12],id_exe_inst.instr[20],id_exe_inst.instr[30:21],1'b0});
    pc = id_exe_inst.pc + imm;
end else if(id_exe_inst.op == JALR && !stall_o_1) begin // JALR
    imm = $signed({{20{id_exe_inst.instr[31]}},id_exe_inst.instr[31:20]});
    pc = (rf_rdata_a_reg+imm)& ~1;
end else if(id_exe_inst.op != BEQ && id_exe_inst.op != BNE && id_exe_inst.op != JAL && id_exe_inst.op != JALR && id_exe_inst.op != NOP) begin // 普通指令
    pc = pc_now + 4;
end else begin
    pc = pc_store;
end
```
- 此外还需要说明的是，我们的pc更新设计**不需要考虑读入错误的指令并flush的情况**。因为我们的pc并不总是在做+4操作，而是当有效指令（非NOP）到来时才更新，否则保持不变，而且有数据旁路保障使得pc不会更新到错误的地址。

**EXE:**
该阶段实现了ALU运算的控制。根据指令类型，选择对应的运算数，其中AUIPC，JAL，JALR需要对pc计算，而id_exe_inst.imm_type为1的情况需要对立即数计算。
```
if(id_exe_inst.exe_en) begin
    alu_a = (id_exe_inst.op==AUIPC||id_exe_inst.op==JAL||id_exe_inst.op==JALR) ? id_exe_inst.pc : rf_rdata_a_reg;
    alu_b = id_exe_inst.imm_type ? id_exe_inst.imm : rf_rdata_b_reg;
end else begin
    alu_a = 32'h00000000;
    alu_b = 32'h00000000;
end
```
运算结果在下一个时钟上升沿到来后保存到段寄存器instruction_exe_mem中。
```
typedef struct packed {
    logic mem_en;
    logic wb_en;
    op_t op;
    logic [31:0] pc;
    logic [31:0] alu_result;
    logic [31:0] mem_dat_o;
    logic [31:0] mem_addr;
    logic [31:0] imm;
    logic [31:0] rd;
} instruction_exe_mem;
```

**MEM:**
该阶段实现了访存操作。在实现了dcache后，将信号传入dcache，其他操作与直接访存一致。对于4条访存指令，LW和SW无需考虑地址对齐的问题，而LB和SB则需要根据偏移量设置不同的sel_out信号。
```
if(exe_mem_inst.op==LB)begin
    data_out <= 32'h00000000;
    write_enable <= 1'b0;
    read_enable <= 1'b1;
    if(exe_mem_inst.mem_addr%4==0)begin
        sel_out <= 4'b0001;
    end else if(exe_mem_inst.mem_addr%4==1)begin
        sel_out <= 4'b0010;
    end else if(exe_mem_inst.mem_addr%4==2)begin
        sel_out <= 4'b0100;
    end else if(exe_mem_inst.mem_addr%4==3)begin
        sel_out <= 4'b1000;
    end
end else if(exe_mem_inst.op==LW)begin
    data_out <= 32'h00000000;
    write_enable <= 1'b0;
    read_enable <= 1'b1;
    sel_out <= 4'b1111;
end else if(exe_mem_inst.op==SB)begin
    write_enable <= 1'b1;
    read_enable <= 1'b0;
    if(exe_mem_inst.mem_addr%4==0)begin
        data_out <= exe_mem_inst.mem_dat_o;
        sel_out <= 4'b0001;
    end else if(exe_mem_inst.mem_addr%4==1)begin
        data_out <= exe_mem_inst.mem_dat_o << 8;
        sel_out <= 4'b0010;
    end else if(exe_mem_inst.mem_addr%4==2)begin
        data_out <= exe_mem_inst.mem_dat_o << 16;
        sel_out <= 4'b0100;
    end else if(exe_mem_inst.mem_addr%4==3)begin
        data_out <= exe_mem_inst.mem_dat_o << 24;
        sel_out <= 4'b1000;
    end
end else if(exe_mem_inst.op==SW)begin
    data_out <= exe_mem_inst.mem_dat_o;
    write_enable <= 1'b1;
    read_enable <= 1'b0;
    sel_out <= 4'b1111; 
end
```
如果dcache读命中，则可以一周期读取数据，将结果放入段寄存器instruction_mem_wb；否则等待dcache_ack的到来。
```
typedef struct packed {
    logic wb_en;
    op_t op;
    logic [31:0] pc;
    logic [31:0] alu_result;
    logic [31:0] mem_addr;
    logic [31:0] mem_dat_i;
    logic [31:0] rf_writeback;
    logic [31:0] imm;
    logic [4:0] rd;
} instruction_mem_wb;
```

**WB:**
无需讨论指令类型，只需要根据段寄存器instruction_mem_wb的信号写回即可。
```
rf_wdata = mem_wb_inst.rf_writeback;
rf_waddr = mem_wb_inst.rd;
if(mem_wb_inst.wb_en)begin
    rf_we = 1'b1;
end else begin
    rf_we = 1'b0;
end
```

## 2.3 Datapath

CSY 简述数据前传部分的实现逻辑。并给出数据冲突的例子（假设 IF 段和 MEM 段功能可以在 1 周期内完成），以及对应关键信号的波形图。

## 2.4 Memory & ALU
SRAM实际上是静态随机存取存储器，但是由于SRAM的工作原理相比DDR更为简单，所以SRAM在本实验中作为Main Memory，并且组织为了一维数组的格式，ALU即算术计算单元，完成一些基本的运算

SRAM的时序逻辑如下:

![Untitled (Draft)-1 2](https://gitee.com/Sh1h0/photo/raw/master/img/Untitled%20(Draft)-1%202.jpg)

ALU由组合逻辑实现，无时序

## 2.5 中断异常

CSY 简要介绍功能是如何实现的，给出功能实现原理波形图，以及上板实验的截图 / 性能数据对比。

## 2.6 页表
### 2.6.0 组成单元

页表功能的所有功能模块包括MMU，TLB，wb_arbiter_2和 wb_mux_3

### 2.6.1 接入系统的位置及连接逻辑

MMU实现页表的多级访问从而实现虚拟地址到物理地址的转换，所以MMU的输入应该是wb_arbiter_2所仲裁出来的结果，并且wb_mux_3所对应的地址输入，sel，we等等信号都应该是MMU新产生的输出，而不是之前与wb_arbiter_2直接相连接的输出，同时MMU还应该与TLB相连，MMU给TLB传入最初的va进行查找，并且根据TLB的查找结果采取相应的动作(也即跳转到相应的状态)，详细说明见下一小节

### 2.6.3 TLB 实现细节

- 基本实现逻辑————所实现的TLB包含16个表项，每个表项由va,对应的pa，valid位，{G,U,X,W,R} 五个权限位以及LRU计数组成，TLB的主要功能就是查找传入的va是否之前已经缓存在了TLB中，如果存在并且valid位有效，那么MMU就可以不用进行页表的翻译工作，TLB直接返回va所对应的pa和权限位如果存在但是valid位无效，抛出page fault异常，等到需要的页从磁盘加载进来，更新TLB条目，再进行翻译，否则如果不存在，进行正常的页表翻译流程，并且更新TLB表
- TLB表的替换逻辑————如果更新TLB表的entry时表单已经满了并且没有所要更新的va的entry，所采取的替换原则就是最近最少使用LRU原则，具体的，每次使用一个va的时候，我都把该va对应的entry的LRU设置为`TLB_ENTRIES - 1`，也即15，并且把所有不小于该va原本的LRU的entry的LRU都减1，这样既保证了所有entry的LRU都大于0，也让每次所更新的entry都是接近最远使用的entry

### 2.6.4 MMU实现细节

- 基本实现逻辑————构建了一个状态机来实现MMU的功能，状态机的初始状态为IDLE，当wb_arbiter_2输出的wbs_stb_o为1的时候，若此时不处于sv32_mode，直接输出pa并且设置相应信号，进入WAIT_ACK状态即可，否则首先将va传入TLB，进入WAIT_HIT状态等待TLB完成查询，如果hit，根据TLB的查询结果检查权限位(权限位的检查逻辑在后面)，根据结果设置相应信号，否则进入LEVEL1，该状态查询一级页表获得二级页表的物理页号，状态根据所获得的物理页号和虚拟地址本身所带的虚拟页号获得最终的物理页号，然后根据虚拟地址的offset获得最后的物理地址，转入LEVEL2，LEVEL2根据获得的权限判断的结果选择访问最终物理地址的最终信号，并将获得的结果传入wb_arbiter_2；WAIT_ACK之后的状态不是IDLE而是WAIT是因为需要等一周期和wbm_ack_i同步,防止连续访问同一个地址，导致错误
- 权限判断逻辑

1. **指令取指（IF 段）：**
   - 检查 `X` 位是否被设置。
   - 若当前处于用户模式（U），还需要检查 `U` 位。
   - 不满足条件，则触发 **Instruction Page Fault** 异常。
2. **内存访问（MEM 段）：**
   - 对读操作：检查 `R` 位。
   - 对写操作：检查 `W` 位。
   - 若访问指令运行于用户模式（U），也需要检查 `U` 位的权限。
   - 不满足条件，则触发 **Load Page Fault** 或 **Store Page Fault**。

### 2.6.5 原理波形图

![](https://gitee.com/Sh1h0/photo/raw/master/img/%E6%88%AA%E5%B1%8F2024-12-12%2014.41.51.png)

### 2.6.6性能测试

![截屏2024-12-11 23.46.01](/Users/quguoli/Downloads/截屏2024-12-11 23.46.01.png)

![截屏2024-12-11 23.45.57](/Users/quguoli/Downloads/截屏2024-12-11 23.45.57.png)

![截屏2024-12-11 23.45.50](/Users/quguoli/Downloads/截屏2024-12-11 23.45.50.png)

![截屏2024-12-11 23.45.53](/Users/quguoli/Downloads/截屏2024-12-11 23.45.53.png)

![截屏2024-12-11 23.45.45](/Users/quguoli/Downloads/截屏2024-12-11 23.45.45.png)

## 2.7 缓存
我们实现了icache和dcache，都是VIVT结构，对于dcache可能会出现重名的问题。

### 2.7.1 icache：
- 实现结构：VIVT
- 地址映射：4路组相联
- 参数：
    - 总大小：1024 Byte
    - 块大小：16 Byte
    - 路数：4
    - 块总数：64
    - 组总数：16
    - 索引位数：4
    - 偏移位数：4
    - 标签位数：24
- 替换策略：随机替换

如果icache命中，则可以一周期读取指令，否则需要访存4次读取数据（因为每个块有16字节）。在状态机中我使用cnt来模拟随机数生成器，每次上升沿cnt++，然后取cnt%4作为替换路数，从而实现随机替换。

### 2.7.2 dcache:
- 实现结构：VIVT
- 地址映射：4路组相联
- 参数：
    - 总大小：512 Byte
    - 块大小：4 Byte
    - 路数：4
    - 块总数：128
    - 组总数：32
    - 索引位数：5
    - 偏移位数：2
    - 标签位数：25
- 替换策略：随机替换
- 写策略：写直达
- 有效地址：0x80400000~0x807FFFFF

在读时，如果dcache命中，则可以一周期读取数据；否则需要做一次访存。在写时，如果dcache命中，则修改对应的缓存块，并且写入内存；否则先写回内存，再读取内存替换缓存。
在状态机中我使用cnt来模拟随机数生成器，每次上升沿cnt++，然后取cnt%4作为替换路数，从而实现随机替换。
需要注意的是，在dcache中使用VIVT结构是有问题的，因为可能出现两个虚拟地址映射到同一个物理地址的情况。当其中一个地址的数据修改后，另一个地址的数据没有同步更新。修改成VIPT结构可以解决这个问题。

### 2.7.3 波形：
下图是cache未命中时的情况，可以看到访存4次，然后取随机路数替换。
![](https://pic.imgdb.cn/item/675a58a2d0e0a243d4e261a1.png)
下图是命中时的情况，可以看到一周期直接返回数据。
![](https://pic.imgdb.cn/item/675a58fad0e0a243d4e262c8.png)

### 2.7.4 性能：
没有cache的情况下，性能为：
![](https://pic.imgdb.cn/item/675a4f9bd0e0a243d4e2453a.png)
有cache的情况下，性能如下。可以看到明显的性能提升：
![](https://pic.imgdb.cn/item/675a4faed0e0a243d4e24576.png)


# 3. 思考题
## 3.1 流水线 CPU 设计与多周期 CPU 设计的异同？插入等待周期（气泡）和数据旁路在处理数据冲突的性能上有什么差异。
- 流水线和多周期的相同点：功能目标相同，步骤相同（IF,ID,EXE,MEM,WB），基本组件类似（ALU,Regfile,Wishbone等）
- 流水线和多周期的不同点：多周期按顺序执行每一条指令，分为五个步骤；而流水线将指令执行过程划分为多个流水级，每个流水级都可以独立地处理不同指令的不同阶段。最耗时的是IF阶段，如果没有实现cache，实际上流水线并不能完全发挥出效果，实践发现当下一条指令取值完毕时，本条指令剩下的阶段已经完成（除非需要访存的指令），也就是说大部分情况下，流水线相较于多周期省略的是后面的4条指令的时间。实现了cache后，流水线的性能提升明显。但在硬件设计的复杂度上多周期显然比流水线更加简单。
- 插入气泡和数据旁路的性能差异：插入气泡不需要额外的硬件支持，会降低流水线的效率，最坏的情况下，每次发生数据冲突都插入等待周期，可能会导致流水线的实际性能大打折扣。而数据旁路保持了较高的指令吞吐率，但是消耗硬件资源。

## 3.2 如何使用 Flash 作为外存，如果要求 CPU 在启动时，能够将存放在 Flash 上固定位置的监控程序读入内存，CPU 应当做什么样的改动？

- 需要增加一个Flash控制器模块，并将对特定地址区间的读写映射为对Flash的读写。
- 增加一个加载程序，CPU被reset后执行加载程序从固定位置读取监控程序到内存中。

## 3.3 如何将 DVI 作为系统的输出设备，从而在屏幕上显示文字？
- 根据屏幕的分辨率和刷新率（如 1920x1080 @ 60Hz），生成 **水平同步信号（Hsync）** 和 **垂直同步信号（Vsync）**。
- 生成数据使能信号（DE，Data Enable），指示当前像素是否有效。
- 使用 PLL（如 `pll_example`）生成合适的像素时钟（Pixel Clock），其频率由分辨率和刷新率决定。
- 从帧缓冲区中读取像素数据（包括 RGB 值），并通过 DVI 接口发送到屏幕。

## 3.4 （缓存）对于性能测试中的 4MDCT 测例，计算一下你设计的缓存在理论上的命中率和性能提升效果，和实际测试结果对比一下是否相符。
该测例的核心指令在0x80001088~0x8000109c之间。
首先考虑icache，由于我实现的是每个块16字节，所以上述指令会被缓存到两个块中。由于需要至少执行 192M 指令，所以可以近似认为缓存命中率接近100%。
然后考虑dcache，由于每次访存的地址都是0(sp)，所以可以认为命中率为100%。
icache命中时只需要一周期读，而没有缓存的情况下需要4周期，所以减少了$192M \times 3=576M$周期的访存时间。dcache读命中时只需要一周期，而没有缓存的情况下需要4周期，所以减少了$64M \times 3=192M$周期的访存时间。dcache写命中时需要写回内存，再读取内存替换缓存，所以实际上需要比没有缓存的情况下多花费$64M \times 3=192M$周期的时间。也就是说总共减少的时间只是icache带来的性能提升。
考虑没有缓存的情况下执行一条非访存的指令需要8周期；有缓存的指令需要11周期；平均需要10周期。有cache则每条指令少3周期。所以估计能提升30%。

下图分别是无缓存和有缓存情况下的指令执行时间，发现提升了约36%。这个数据比我估计的稍大，可能是因为cache的存在同时减少了因为数据冲突导致的等待时间。
<div style="text-align:center;">
    <img src="https://pic.imgdb.cn/item/675bcdbad0e0a243d4e35535.png" alt="图片1" style="display:inline - block; width:300px; margin - right:10px;">
    <img src="https://pic.imgdb.cn/item/675bcdf8d0e0a243d4e3554b.png" alt="图片2" style="display:inline - block; width:300px;">
</div>

## 3.5 （虚拟内存）考虑支持虚拟内存的监控程序。如果要初始化完成后用 G 命令运行起始物理地址 0x80100000 处的用户程序，可以输入哪些地址？分别描述一下输入这些地址时的地址翻译流程。

可以输入**`0x00000000`**和**`0x80100000`**

### **（1）输入 `0x00000000` 的地址翻译流程**

1. 虚拟地址 `0x00000000` 查找TLB失败，查找两次页表到物理地址 `0x80100000`。
2. 权限检查：
   - 检查页表中的权限，发现为 `DAGUX-RV`，即可读、可执行，不可写，且用户态下有效。
   - 地址合法，可以执行。
3. 程序执行：
   - CPU 跳转至物理地址 `0x80100000` 开始运行用户程序。

输入 `0x80100000` 的地址翻译流程与输入 `0x00000000` 的地址翻译流程一样

## 3.6 （异常与中断）假设第 a 个周期在 ID 阶段发生了 Illegal Instruction 异常，你的 CPU 会在周期 b 从中断处理函数的入口开始取指令执行，在你的设计中，b - a 的值为？

在本设计中跳转被放在ID阶段末尾，当ID阶段发生异常后在下一周期起始pc值即被设为中断处理入口地址，IF阶段随即从该地址取指，故b-a的值为1。

# 4. 心得和体会
- 杨季睿：最重要和最困难的部分就是搭基础流水线，因为后面附加功能中出现的很多问题都是由于流水线的时序造成的。这三周的工作很艰辛，但是收获很大！
- 陈诗洋：非常有挑战性。
- 瞿果李：理解了流水线分阶段执行和并发运行的核心思想，并熟悉了硬件设计中的数据依赖解决方法（如前递、暂停等）。掌握了虚拟地址到物理地址转换的具体流程，多级页表结构如何减少内存开销，以及 TLB 如何加速地址转换。

# 5. 分工
- 杨季睿：基础流水线 + cache
- 瞿果李：基础流水线 + 页表
- 陈诗洋：基础流水线 + 异常与中断