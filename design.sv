`timescale 1ns/1ps

// Alt modüller için timescale tanımlamaları
`ifndef CONTROL_UNIT_TIMESCALE
`define CONTROL_UNIT_TIMESCALE
`timescale 1ns/1ps
`endif

`ifndef EX_STAGE_TIMESCALE
`define EX_STAGE_TIMESCALE
`timescale 1ns/1ps
`endif

module MIPS_Pipeline (
    input wire clk,
    input wire rst,
    output wire [15:0] pc_out
);

    // Parameters
    parameter MEM_SIZE = 512;
    parameter REG_COUNT = 8;

    // Integer for loops
    integer i;

    // Internal signals - tümü sıfırlanacak şekilde başlatılıyor
    reg [15:0] pc = 16'h0000;
    reg [15:0] next_pc = 16'h0000;
    reg stall = 1'b0;
    reg flush = 1'b0;

    // Pipeline registers - tümü sıfırlanacak şekilde başlatılıyor
    // IF/ID Stage
    reg [15:0] if_id_pc = 16'h0000;
    reg [15:0] if_id_instr = 16'h0000;
    reg if_id_valid = 1'b0;

    // ID/EX Stage
    reg [15:0] id_ex_pc = 16'h0000;
    reg [2:0] id_ex_rs = 3'b000;
    reg [2:0] id_ex_rt = 3'b000;
    reg [2:0] id_ex_rd = 3'b000;
    reg [15:0] id_ex_reg1 = 16'h0000;
    reg [15:0] id_ex_reg2 = 16'h0000;
    reg [15:0] id_ex_imm = 16'h0000;
    reg [3:0] id_ex_alu_op = 4'b0000;
    reg id_ex_mem_read = 1'b0;
    reg id_ex_mem_write = 1'b0;
    reg id_ex_reg_write = 1'b0;
    reg id_ex_alu_src = 1'b0;
    reg id_ex_reg_dst = 1'b0;
    reg id_ex_branch = 1'b0;
    reg id_ex_valid = 1'b0;

    // EX/MEM Stage
    reg [15:0] ex_mem_alu_result = 16'h0000;
    reg [15:0] ex_mem_reg2 = 16'h0000;
    reg [2:0] ex_mem_reg_dst = 3'b000;
    reg ex_mem_mem_read = 1'b0;
    reg ex_mem_mem_write = 1'b0;
    reg ex_mem_reg_write = 1'b0;
    reg ex_mem_branch_taken = 1'b0;
    reg [15:0] ex_mem_branch_target = 16'h0000;
    reg ex_mem_valid = 1'b0;

    // MEM/WB Stage
    reg [15:0] mem_wb_data = 16'h0000;
    reg [2:0] mem_wb_reg_dst = 3'b000;
    reg mem_wb_reg_write = 1'b0;
    reg mem_wb_valid = 1'b0;

    // Memory arrays
    reg [7:0] instr_memory [0:MEM_SIZE-1];
    reg [7:0] data_memory [0:MEM_SIZE-1];
    reg [15:0] reg_file [0:REG_COUNT-1];

    // Forwarding and hazard signals
    reg [1:0] forward_a = 2'b00;
    reg [1:0] forward_b = 2'b00;
    reg [15:0] alu_in1 = 16'h0000;
    reg [15:0] alu_in2 = 16'h0000;
    reg load_use_hazard = 1'b0;
    reg control_hazard = 1'b0;

    // Instruction decode signals
    reg [3:0] opcode = 4'b0000;
    reg [2:0] rs = 3'b000;
    reg [2:0] rt = 3'b000;
    reg [2:0] rd = 3'b000;
    reg [15:0] immediate = 16'h0000;

    // Initialize memory and registers
    initial begin
        for (i = 0; i < MEM_SIZE; i = i + 1) begin
            instr_memory[i] = 8'h00;
            data_memory[i] = 8'h00;
        end
        for (i = 0; i < REG_COUNT; i = i + 1) begin
            reg_file[i] = 16'h0000;
        end
    end

    // ALU - geliştirilmiş versiyon
    reg [15:0] alu_result;

    always @(*) begin
        // ALU input selection with explicit bit ranges
        case(forward_a)
            2'b00: alu_in1 = id_ex_reg1;
            2'b10: alu_in1 = ex_mem_alu_result;
            2'b01: alu_in1 = mem_wb_data;
            default: alu_in1 = id_ex_reg1;
        endcase
        
        // Second operand selection with explicit bit ranges
        case(forward_b)
            2'b00: alu_in2 = id_ex_alu_src ? id_ex_imm : id_ex_reg2;
            2'b10: alu_in2 = ex_mem_alu_result;
            2'b01: alu_in2 = mem_wb_data;
            default: alu_in2 = id_ex_alu_src ? id_ex_imm : id_ex_reg2;
        endcase

        // ALU operation with explicit bit ranges
        case(id_ex_alu_op)
            4'b0000: alu_result = {1'b0, alu_in1} + {1'b0, alu_in2}; // ADD with overflow check
            4'b0001: alu_result = alu_in1 - alu_in2;                  // SUB
            4'b0010: alu_result = alu_in1 & alu_in2;                  // AND
            4'b0011: alu_result = alu_in1 | alu_in2;                  // OR
            4'b0100: alu_result = $signed(alu_in1) < $signed(alu_in2) ? 16'd1 : 16'd0; // SLT
            4'b0101: alu_result = alu_in2 << alu_in1[3:0];           // SLL
            4'b0110: alu_result = alu_in2 >> alu_in1[3:0];           // SRL
            default: alu_result = alu_in1 + alu_in2;
        endcase
    end

    // MEM Stage - geliştirilmiş versiyon
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mem_wb_data <= 16'h0000;
            mem_wb_reg_dst <= 3'b000;
            mem_wb_reg_write <= 1'b0;
            mem_wb_valid <= 1'b0;
        end else begin
            if (ex_mem_valid) begin
                if (ex_mem_mem_read) begin
                    // Memory read with bounds checking
                    if (ex_mem_alu_result < MEM_SIZE - 1) begin
                        mem_wb_data <= {data_memory[ex_mem_alu_result], 
                                      data_memory[ex_mem_alu_result + 1]};
                    end else begin
                        mem_wb_data <= 16'h0000; // Memory access error
                    end
                end else if (ex_mem_mem_write) begin
                    // Memory write with bounds checking
                    if (ex_mem_alu_result < MEM_SIZE - 1) begin
                        data_memory[ex_mem_alu_result] <= ex_mem_reg2[15:8];
                        data_memory[ex_mem_alu_result + 1] <= ex_mem_reg2[7:0];
                    end
                    mem_wb_data <= ex_mem_alu_result; // Pass ALU result in case of SW
                end else begin
                    // ALU result
                    mem_wb_data <= ex_mem_alu_result;
                end
                
                mem_wb_reg_dst <= ex_mem_reg_dst;
                mem_wb_reg_write <= ex_mem_reg_write;
                mem_wb_valid <= 1'b1;
            end else begin
                mem_wb_valid <= 1'b0;
            end
        end
    end

    // Instruction Format Decoder - düzeltilmiş versiyon
    always @(*) begin
        // Bit seçimleri için wire tanımlamaları
        wire [3:0] opcode_bits;
        wire [2:0] rs_bits, rt_bits, rd_bits;
        wire [7:0] imm_bits;
        
        // Instruction decode - explicit bit selections
        opcode_bits = if_id_instr[15:12];
        rs_bits = if_id_instr[11:9];
        rt_bits = if_id_instr[8:6];
        rd_bits = if_id_instr[5:3];
        imm_bits = if_id_instr[7:0];
        
        opcode = opcode_bits;
        rs = rs_bits;
        rt = rt_bits;
        rd = rd_bits;
        
        // Immediate değer düzeltmesi
        case(opcode)
            4'b0001: immediate = {{8{imm_bits[7]}}, imm_bits}; // addi
            4'b0010,
            4'b0011: immediate = {{8{imm_bits[7]}}, imm_bits}; // lw/sw
            4'b0100,
            4'b0101: immediate = {{7{imm_bits[7]}}, imm_bits, 1'b0}; // beq/bne
            default: immediate = 16'h0000;
        endcase
    end

    // Control Unit
    always @(*) begin
        // Default values
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        alu_src = 0;
        reg_dst = 0;
        branch = 0;
        alu_op = 4'b0000;

        case(opcode)
            4'b0000: begin // R-type
                reg_write = 1;
                reg_dst = 1;
                alu_op = {1'b0, if_id_instr[2:0]}; // funct field
            end
            4'b0001: begin // addi
                reg_write = 1;
                alu_src = 1;
                alu_op = 4'b0000;
            end
            4'b0010: begin // lw
                reg_write = 1;
                mem_read = 1;
                alu_src = 1;
                alu_op = 4'b0000;
            end
            4'b0011: begin // sw
                mem_write = 1;
                alu_src = 1;
                alu_op = 4'b0000;
            end
            4'b0100: begin // beq
                branch = 1;
                alu_op = 4'b0001;
            end
            4'b0101: begin // bne
                branch = 1;
                alu_op = 4'b0001;
            end
        endcase
    end

    // Hazard Detection Unit - geliştirilmiş versiyon
    always @(*) begin
        load_use_hazard = 1'b0;
        control_hazard = 1'b0;
        
        // Load-use hazard detection
        if (id_ex_mem_read && id_ex_valid) begin
            if ((id_ex_rt == rs) || (id_ex_rt == rt)) begin
                load_use_hazard = 1'b1;
            end
        end
        
        // Control hazard detection
        if ((id_ex_branch && id_ex_valid) || ex_mem_branch_taken) begin
            control_hazard = 1'b1;
        end
    end

    // Forwarding Unit - geliştirilmiş versiyon
    always @(*) begin
        forward_a = 2'b00;
        forward_b = 2'b00;
        
        // EX Hazard
        if (ex_mem_valid && ex_mem_reg_write && (ex_mem_reg_dst != 0)) begin
            // RS için forwarding
            if (ex_mem_reg_dst == id_ex_rs) begin
                forward_a = 2'b10;
            end
            // RT için forwarding (immediate kullanılmıyorsa)
            if (ex_mem_reg_dst == id_ex_rt && !id_ex_alu_src) begin
                forward_b = 2'b10;
            end
        end
        
        // MEM Hazard
        if (mem_wb_valid && mem_wb_reg_write && (mem_wb_reg_dst != 0)) begin
            // RS için forwarding (EX hazard yoksa)
            if (!(ex_mem_valid && ex_mem_reg_write && 
                  ex_mem_reg_dst != 0 && ex_mem_reg_dst == id_ex_rs) &&
                mem_wb_reg_dst == id_ex_rs) begin
                forward_a = 2'b01;
            end
            // RT için forwarding (EX hazard yoksa ve immediate kullanılmıyorsa)
            if (!(ex_mem_valid && ex_mem_reg_write && 
                  ex_mem_reg_dst != 0 && ex_mem_reg_dst == id_ex_rt) &&
                mem_wb_reg_dst == id_ex_rt && !id_ex_alu_src) begin
                forward_b = 2'b01;
            end
        end
    end

    // Pipeline stages

    // IF Stage - geliştirilmiş versiyon
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            if_id_pc <= 16'h0000;
            if_id_instr <= 16'h0000;
            if_id_valid <= 1'b0;
            pc <= 16'h0000;
        end else begin
            if (!stall) begin
                if (flush) begin
                    if_id_valid <= 1'b0;
                    if_id_instr <= 16'h0000;
                end else begin
                    if_id_pc <= pc;
                    if_id_instr <= {instr_memory[pc], instr_memory[pc + 1]};
                    if_id_valid <= 1'b1;
                end
                pc <= next_pc;
            end
        end
    end

    // ID Stage - geliştirilmiş versiyon
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            id_ex_pc <= 16'h0000;
            id_ex_rs <= 3'b000;
            id_ex_rt <= 3'b000;
            id_ex_rd <= 3'b000;
            id_ex_reg1 <= 16'h0000;
            id_ex_reg2 <= 16'h0000;
            id_ex_imm <= 16'h0000;
            id_ex_alu_op <= 4'b0000;
            id_ex_mem_read <= 1'b0;
            id_ex_mem_write <= 1'b0;
            id_ex_reg_write <= 1'b0;
            id_ex_alu_src <= 1'b0;
            id_ex_reg_dst <= 1'b0;
            id_ex_branch <= 1'b0;
            id_ex_valid <= 1'b0;
        end else if (!stall) begin
            if (flush) begin
                id_ex_valid <= 1'b0;
            end else if (if_id_valid) begin
                id_ex_pc <= if_id_pc;
                id_ex_rs <= rs;
                id_ex_rt <= rt;
                id_ex_rd <= rd;
                id_ex_reg1 <= reg_file[rs];
                id_ex_reg2 <= reg_file[rt];
                id_ex_imm <= immediate;
                id_ex_alu_op <= alu_op;
                id_ex_mem_read <= mem_read;
                id_ex_mem_write <= mem_write;
                id_ex_reg_write <= reg_write;
                id_ex_alu_src <= alu_src;
                id_ex_reg_dst <= reg_dst;
                id_ex_branch <= branch;
                id_ex_valid <= 1'b1;
            end
        end
    end

    // EX Stage - geliştirilmiş versiyon
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ex_mem_alu_result <= 16'h0000;
            ex_mem_reg2 <= 16'h0000;
            ex_mem_reg_dst <= 3'b000;
            ex_mem_mem_read <= 1'b0;
            ex_mem_mem_write <= 1'b0;
            ex_mem_reg_write <= 1'b0;
            ex_mem_branch_taken <= 1'b0;
            ex_mem_branch_target <= 16'h0000;
            ex_mem_valid <= 1'b0;
        end else begin
            if (id_ex_valid) begin
                ex_mem_alu_result <= alu_result;
                ex_mem_reg2 <= id_ex_reg2;
                ex_mem_reg_dst <= id_ex_reg_dst ? id_ex_rd : id_ex_rt;
                ex_mem_mem_read <= id_ex_mem_read;
                ex_mem_mem_write <= id_ex_mem_write;
                ex_mem_reg_write <= id_ex_reg_write;
                ex_mem_branch_taken <= id_ex_branch && 
                                     ((opcode == 4'b0100 && alu_result == 16'h0000) ||  // BEQ
                                      (opcode == 4'b0101 && alu_result != 16'h0000));   // BNE
                ex_mem_branch_target <= id_ex_pc + {id_ex_imm[14:0], 1'b0};  // Shift left by 1
                ex_mem_valid <= 1'b1;
            end else begin
                ex_mem_valid <= 1'b0;
            end
        end
    end

    // PC Update and Control Logic - geliştirilmiş versiyon
    always @(*) begin
        stall = load_use_hazard;
        flush = ex_mem_branch_taken || control_hazard;

        // Next PC logic with improved branch prediction
        if (ex_mem_branch_taken)
            next_pc = ex_mem_branch_target;
        else if (stall)
            next_pc = pc;
        else if (id_ex_branch && id_ex_valid) begin
            // Static branch prediction: predict not taken
            next_pc = pc + 2;
        end
        else
            next_pc = pc + 2;
    end

    // Output assignment
    assign pc_out = pc;

    // Pipeline register updates - geliştirilmiş versiyon
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            if_id_valid <= 0;
            id_ex_valid <= 0;
            ex_mem_valid <= 0;
            mem_wb_valid <= 0;
            pc <= 16'h0000;
        end else begin
            if (load_use_hazard) begin
                // Stall IF/ID and ID/EX stages
                if_id_valid <= if_id_valid;
                id_ex_valid <= 0;
                pc <= pc;
            end else if (control_hazard || ex_mem_branch_taken) begin
                // Flush pipeline
                if_id_valid <= 0;
                id_ex_valid <= 0;
                
                // Update PC for branch
                if (ex_mem_branch_taken)
                    pc <= ex_mem_branch_target;
                else
                    pc <= pc + 2;
            end else begin
                // Normal operation
                if_id_valid <= 1;
                id_ex_valid <= if_id_valid;
                pc <= next_pc;
            end
            
            // Update remaining stages
            ex_mem_valid <= id_ex_valid && !flush;
            mem_wb_valid <= ex_mem_valid;
        end
    end

    // Pipeline Status Monitor - debug mesajları kaldırıldı
    always @(posedge clk) begin
        if (!rst) begin
            // Debug bilgileri testbench'e taşındı
        end
    end

    // Write-Back Stage - geliştirilmiş versiyon
    always @(posedge clk) begin
        if (mem_wb_valid && mem_wb_reg_write && (mem_wb_reg_dst != 3'b000)) begin
            reg_file[mem_wb_reg_dst] <= mem_wb_data;
        end
    end

endmodule