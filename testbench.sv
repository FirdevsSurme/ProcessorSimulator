`timescale 1ns/1ps

module MIPS_Pipeline_TB;

    reg clk;

    reg rst;

    wire [15:0] pc_out;

    integer i;



    // Clock generation

    initial begin

        clk = 0;

        forever #5 clk = ~clk;

    end



    // Instantiate the processor

    MIPS_Pipeline dut (

        .clk(clk),

        .rst(rst),

        .pc_out(pc_out)

    );



    // Test program

    initial begin

        // Initialize instruction memory with test program

        // Program: Simple test with various instructions

        

        // addi R1, R0, 5    // R1 = 5

        dut.instr_memory[0] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[1] = 8'b00100101;  // rt=1, immediate=5

        

        // addi R2, R0, 3    // R2 = 3

        dut.instr_memory[2] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[3] = 8'b01000011;  // rt=2, immediate=3

        

        // add R3, R1, R2    // R3 = R1 + R2 = 8

        dut.instr_memory[4] = 8'b00000001;  // opcode=0000, rs=1

        dut.instr_memory[5] = 8'b01011000;  // rt=2, rd=3, funct=000

        

        // sw R3, 4(R0)      // Memory[4] = R3

        dut.instr_memory[6] = 8'b00110000;  // opcode=0011, rs=0

        dut.instr_memory[7] = 8'b01100100;  // rt=3, offset=4

        

        // lw R4, 4(R0)      // R4 = Memory[4]

        dut.instr_memory[8] = 8'b00100000;  // opcode=0010, rs=0

        dut.instr_memory[9] = 8'b10000100;  // rt=4, offset=4

        

        // beq R1, R2, 2     // Skip next instruction if R1 = R2

        dut.instr_memory[10] = 8'b01000001; // opcode=0100, rs=1

        dut.instr_memory[11] = 8'b01000010; // rt=2, offset=2

        

        // sub R5, R3, R2    // R5 = R3 - R2 = 5

        dut.instr_memory[12] = 8'b00000011; // opcode=0000, rs=3

        dut.instr_memory[13] = 8'b01010001; // rt=2, rd=5, funct=001

        

        // Initialize registers

        for (i = 0; i < 8; i = i + 1) begin

            dut.reg_file[i] = 16'h0000;

        end

        

        // Initialize data memory

        for (i = 0; i < 512; i = i + 1) begin

            dut.data_memory[i] = 8'h00;

        end



        // Start simulation

        rst = 1;

        #10 rst = 0;



        // Run for enough cycles to complete the program

        #200;



        // Verify results

        $display("Simulation Results:");

        $display("R1 = %0d (Expected: 5)", dut.reg_file[1]);

        $display("R2 = %0d (Expected: 3)", dut.reg_file[2]);

        $display("R3 = %0d (Expected: 8)", dut.reg_file[3]);

        $display("R4 = %0d (Expected: 8)", dut.reg_file[4]);

        $display("R5 = %0d (Expected: 5)", dut.reg_file[5]);

        $display("Memory[4] = %0d (Expected: 8)", 

                {dut.data_memory[4], dut.data_memory[5]});



        // Check for hazards

        $display("\nPipeline Hazards:");

        $display("Load-Use Hazard detected: %0d", dut.load_use_hazard);

        $display("Control Hazard detected: %0d", dut.control_hazard);



        // Check forwarding

        $display("\nForwarding Information:");

        $display("Forward A: %0b", dut.forward_a);

        $display("Forward B: %0b", dut.forward_b);



        // Test case 2: Test forwarding

        #50;

        

        // addi R1, R0, 10

        // add R2, R1, R1  // Requires forwarding from MEM

        // add R3, R2, R1  // Requires forwarding from EX

        dut.instr_memory[16] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[17] = 8'b00101010;  // rt=1, immediate=10

        

        dut.instr_memory[18] = 8'b00000001;  // opcode=0000, rs=1

        dut.instr_memory[19] = 8'b00101000;  // rt=1, rd=2, funct=000

        

        dut.instr_memory[20] = 8'b00000010;  // opcode=0000, rs=2

        dut.instr_memory[21] = 8'b00111000;  // rt=1, rd=3, funct=000



        // Reset processor

        rst = 1;

        #10 rst = 0;



        // Run test

        #150;



        // Verify results

        $display("\nTest Case 2 Results:");

        $display("R1 = %0d (Expected: 10)", dut.reg_file[1]);

        $display("R2 = %0d (Expected: 20)", dut.reg_file[2]);

        $display("R3 = %0d (Expected: 30)", dut.reg_file[3]);



        // Test case 3: Test branch prediction and control hazards

        #50;
        

        // addi R1, R0, 1

        // addi R2, R0, 1

        // beq R1, R2, 2    // Should be taken

        // addi R3, R0, 5   // Should be skipped

        // addi R4, R0, 10  // Should execute after branch

        dut.instr_memory[24] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[25] = 8'b00100001;  // rt=1, immediate=1

        

        dut.instr_memory[26] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[27] = 8'b01000001;  // rt=2, immediate=1

        

        dut.instr_memory[28] = 8'b01000001;  // opcode=0100, rs=1

        dut.instr_memory[29] = 8'b01000010;  // rt=2, offset=2

        

        dut.instr_memory[30] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[31] = 8'b01100101;  // rt=3, immediate=5

        

        dut.instr_memory[32] = 8'b00010000;  // opcode=0001, rs=0

        dut.instr_memory[33] = 8'b10001010;  // rt=4, immediate=10



        // Reset processor

        rst = 1;

        #10 rst = 0;



        // Run test

        #200;



        // Verify results

        $display("\nTest Case 3 Results:");

        $display("R1 = %0d (Expected: 1)", dut.reg_file[1]);

        $display("R2 = %0d (Expected: 1)", dut.reg_file[2]);

        $display("R3 = %0d (Expected: 0)", dut.reg_file[3]); // Should be skipped

        $display("R4 = %0d (Expected: 10)", dut.reg_file[4]);



        // Test case 4: Test forwarding with multiple dependencies

        #50;
        
        // addi R1, R0, 15
        // add R2, R1, R1  // Requires forwarding from EX
        // sub R3, R2, R1  // Requires forwarding from both EX and MEM
        dut.instr_memory[36] = 8'b00010000;  // opcode=0001, rs=0
        dut.instr_memory[37] = 8'b00101111;  // rt=1, immediate=15
        
        dut.instr_memory[38] = 8'b00000001;  // opcode=0000, rs=1
        dut.instr_memory[39] = 8'b00101000;  // rt=1, rd=2, funct=000
        
        dut.instr_memory[40] = 8'b00000010;  // opcode=0000, rs=2
        dut.instr_memory[41] = 8'b00111001;  // rt=1, rd=3, funct=001

        // Reset processor
        rst = 1;
        #10 rst = 0;

        // Run test
        #150;

        // Verify results
        $display("\nTest Case 4 Results:");
        $display("R1 = %0d (Expected: 15)", dut.reg_file[1]);
        $display("R2 = %0d (Expected: 30)", dut.reg_file[2]);
        $display("R3 = %0d (Expected: 15)", dut.reg_file[3]);

        // Test case 5: Test load-use hazard with forwarding
        #50;
        
        // lw R1, 4(R0)    // Load value from memory
        // add R2, R1, R1  // Use loaded value immediately
        // sub R3, R2, R1  // Use both previous results
        dut.instr_memory[44] = 8'b00100000;  // opcode=0010, rs=0
        dut.instr_memory[45] = 8'b00100100;  // rt=1, offset=4
        
        dut.instr_memory[46] = 8'b00000001;  // opcode=0000, rs=1
        dut.instr_memory[47] = 8'b00101000;  // rt=1, rd=2, funct=000
        
        dut.instr_memory[48] = 8'b00000010;  // opcode=0000, rs=2
        dut.instr_memory[49] = 8'b00111001;  // rt=1, rd=3, funct=001

        // Initialize memory
        dut.data_memory[4] = 8'h00;
        dut.data_memory[5] = 8'h0A;  // Value 10 at memory[4]

        // Reset processor
        rst = 1;
        #10 rst = 0;

        // Run test
        #200;

        // Verify results
        $display("\nTest Case 5 Results:");
        $display("R1 = %0d (Expected: 10)", dut.reg_file[1]);
        $display("R2 = %0d (Expected: 20)", dut.reg_file[2]);
        $display("R3 = %0d (Expected: 10)", dut.reg_file[3]);

        // Test case 6: Test all R-type instructions
        #50;

        // Test ADD, SUB, AND, OR, SLT, SLL, SRL
        // ADD R1, R2, R3
        dut.instr_memory[52] = 8'b00000010;  // opcode=0000, rs=2
        dut.instr_memory[53] = 8'b01101000;  // rt=3, rd=1, funct=000

        // SUB R4, R1, R2
        dut.instr_memory[54] = 8'b00000001;  // opcode=0000, rs=1
        dut.instr_memory[55] = 8'b01010001;  // rt=2, rd=4, funct=001

        // AND R5, R3, R4
        dut.instr_memory[56] = 8'b00000011;  // opcode=0000, rs=3
        dut.instr_memory[57] = 8'b10010010;  // rt=4, rd=5, funct=010

        // OR R6, R1, R5
        dut.instr_memory[58] = 8'b00000001;  // opcode=0000, rs=1
        dut.instr_memory[59] = 8'b10111011;  // rt=5, rd=6, funct=011

        // Initialize registers for R-type test
        dut.reg_file[2] = 16'h000A;  // R2 = 10
        dut.reg_file[3] = 16'h0005;  // R3 = 5

        // Reset processor
        rst = 1;
        #10 rst = 0;

        // Run test
        #200;

        // Verify results
        $display("\nTest Case 6 Results (R-type Instructions):");
        $display("ADD: R1 = %0d (Expected: 15)", dut.reg_file[1]);
        $display("SUB: R4 = %0d (Expected: 5)", dut.reg_file[4]);
        $display("AND: R5 = %0d (Expected: 0)", dut.reg_file[5]);
        $display("OR:  R6 = %0d (Expected: 15)", dut.reg_file[6]);

        // Check for hazards and forwarding
        $display("\nHazard and Forwarding Status:");
        $display("Load-Use Hazards: %0d", dut.load_use_hazard);
        $display("Control Hazards: %0d", dut.control_hazard);
        $display("Forward A: %0b", dut.forward_a);
        $display("Forward B: %0b", dut.forward_b);

        // Memory consistency check
        $display("\nMemory Consistency Check:");
        for (i = 0; i < 8; i = i + 2) begin
            $display("Memory[%0d] = %0h", i, 
                     {dut.data_memory[i], dut.data_memory[i+1]});
        end

        // Register file consistency check
        $display("\nRegister File Status:");
        for (i = 0; i < 8; i = i + 1) begin
            $display("R%0d = %0h (%0d signed)", i, 
                     dut.reg_file[i], $signed(dut.reg_file[i]));
        end

        // Pipeline stage validity check
        $display("\nPipeline Stage Validity:");
        $display("IF/ID Valid: %0b", dut.if_id_valid);
        $display("ID/EX Valid: %0b", dut.id_ex_valid);
        $display("EX/MEM Valid: %0b", dut.ex_mem_valid);
        $display("MEM/WB Valid: %0b", dut.mem_wb_valid);

        $finish;
    end



    // Monitor pipeline stages - geliştirilmiş versiyon

    always @(posedge clk) begin

        if (!rst) begin

            $display("\n=== Cycle %0d ===", $time/10);
            
            // Register içerikleri

            $display("\nRegister File:");

            for (i = 0; i < 8; i = i + 1)

                $display("R%0d = %h (%0d)", i, dut.reg_file[i], $signed(dut.reg_file[i]));
            
            // Pipeline stages

            $display("\nPipeline Stages:");

            if (dut.if_id_valid)

                $display("IF/ID: PC=%h, Instr=%h", dut.if_id_pc, dut.if_id_instr);
            
            if (dut.id_ex_valid)

                $display("ID/EX: rs=%d, rt=%d, rd=%d, imm=%h, alu_op=%b",

                        dut.id_ex_rs, dut.id_ex_rt, dut.id_ex_rd,

                        dut.id_ex_imm, dut.id_ex_alu_op);
            
            if (dut.ex_mem_valid)

                $display("EX/MEM: alu_result=%h, reg_dst=%d, mem_read=%b, mem_write=%b",

                        dut.ex_mem_alu_result, dut.ex_mem_reg_dst,

                        dut.ex_mem_mem_read, dut.ex_mem_mem_write);
            
            if (dut.mem_wb_valid)

                $display("MEM/WB: data=%h, reg_dst=%d, reg_write=%b",

                        dut.mem_wb_data, dut.mem_wb_reg_dst, dut.mem_wb_reg_write);
            
            // Hazard ve forwarding durumu

            $display("\nHazard Status:");

            $display("Load-Use: %b", dut.load_use_hazard);

            $display("Control: %b", dut.control_hazard);

            $display("Forward A: %b", dut.forward_a);

            $display("Forward B: %b", dut.forward_b);

        end

    end

    // Dalga formu izleme için ekleme
    initial begin
        $dumpfile("mips_pipeline.vcd");
        $dumpvars(0, MIPS_Pipeline_TB);
        
        // Pipeline aşamalarını izle
        $dumpvars(1, dut.if_id_pc);
        $dumpvars(1, dut.if_id_instr);
        $dumpvars(1, dut.id_ex_pc);
        $dumpvars(1, dut.id_ex_reg1);
        $dumpvars(1, dut.id_ex_reg2);
        $dumpvars(1, dut.ex_mem_alu_result);
        $dumpvars(1, dut.mem_wb_data);
        
        // Hazard ve forwarding sinyallerini izle
        $dumpvars(1, dut.load_use_hazard);
        $dumpvars(1, dut.control_hazard);
        $dumpvars(1, dut.forward_a);
        $dumpvars(1, dut.forward_b);
    end

    // Test case 7: Test BNE ve karmaşık forwarding
    #50;

    // Test BNE with forwarding
    // addi R1, R0, 5
    // addi R2, R0, 3
    // bne R1, R2, 2    // Should be taken
    // add R3, R1, R2   // Should be skipped
    // sub R4, R1, R2   // Should execute
    dut.instr_memory[64] = 8'b00010000;  // opcode=0001, rs=0
    dut.instr_memory[65] = 8'b00100101;  // rt=1, immediate=5

    dut.instr_memory[66] = 8'b00010000;  // opcode=0001, rs=0
    dut.instr_memory[67] = 8'b01000011;  // rt=2, immediate=3

    dut.instr_memory[68] = 8'b01010001;  // opcode=0101, rs=1
    dut.instr_memory[69] = 8'b01000010;  // rt=2, offset=2

    dut.instr_memory[70] = 8'b00000001;  // opcode=0000, rs=1
    dut.instr_memory[71] = 8'b01011000;  // rt=2, rd=3, funct=000

    dut.instr_memory[72] = 8'b00000001;  // opcode=0000, rs=1
    dut.instr_memory[73] = 8'b01010001;  // rt=2, rd=4, funct=001

    // Reset processor
    rst = 1;
    #10 rst = 0;

    // Run test
    #200;

    // Verify results
    $display("\nTest Case 7 Results (BNE and Complex Forwarding):");
    $display("R1 = %0d (Expected: 5)", dut.reg_file[1]);
    $display("R2 = %0d (Expected: 3)", dut.reg_file[2]);
    $display("R3 = %0d (Expected: 0)", dut.reg_file[3]); // Should be skipped
    $display("R4 = %0d (Expected: 2)", dut.reg_file[4]);

    // Pipeline stage validity check
    $display("\nPipeline Stage Validity After BNE:");
    $display("IF/ID Valid: %0b", dut.if_id_valid);
    $display("ID/EX Valid: %0b", dut.id_ex_valid);
    $display("EX/MEM Valid: %0b", dut.ex_mem_valid);
    $display("MEM/WB Valid: %0b", dut.mem_wb_valid);

    // Memory bounds check test
    $display("\nMemory Bounds Check Test:");
    // Try to access invalid memory address
    dut.instr_memory[76] = 8'b00100000;  // opcode=0010, rs=0
    dut.instr_memory[77] = 8'b00101111;  // rt=1, offset=511 (boundary)

    #100;
    $display("Memory Access at Boundary: %0h", dut.mem_wb_data);

endmodule