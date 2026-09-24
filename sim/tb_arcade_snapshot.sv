`timescale 1ns/1ps
module tb_arcade_snapshot;
    logic CLK = 0;
    always #5 CLK = ~CLK;
    logic RST_N = 0, SLEEP = 0, EN = 1, WR_N = 1, RD_N = 1;
    logic [20:0] A = 0;
    logic [7:0] DI = 0;
    wire [7:0] DO;
    wire SEL_N, RAM_CS_N;
    wire [20:0] RAM_A;
    logic [63:0] SaveStateBus_Din = 0;
    logic [9:0] SaveStateBus_Adr = 0;
    logic SaveStateBus_wren = 0, SaveStateBus_rst = 0, SaveStateBus_load = 0;
    wire [63:0] SaveStateBus_Dout;
    ARCADE_CARD dut (.*);

    logic [23:0] bases[4];
    logic [15:0] offsets[4], increments[4];
    logic [6:0] controls[4];
    logic [63:0] saved[5];
    logic [31:0] latch_value;
    logic [7:0] shift_value, rotate_value;

    initial begin
        #1000000;
        $fatal(1, "Arcade snapshot watchdog");
    end

    task automatic tick(input integer n = 1);
        repeat (n) begin @(posedge CLK); #1; end
    endtask
    task automatic idle;
        @(negedge CLK); WR_N = 1; RD_N = 1;
        tick(2);
    endtask
    function automatic logic [20:0] reg_addr(input integer p, input integer r);
        return 21'h1ffa00 + 21'(p * 16 + r);
    endfunction
    function automatic logic [20:0] expected_addr(input integer p);
        // Explicit modulo arithmetic, independent of the DUT's concatenations.
        integer displacement;
        displacement = controls[p][1] ? int'(offsets[p]) : 0;
        if (controls[p][1] && controls[p][3]) displacement -= 65536;
        return 21'((int'(bases[p]) + displacement) & 'h1fffff);
    endfunction
    task automatic write_reg(input logic [20:0] address, input logic [7:0] value);
        idle();
        @(negedge CLK); A = address; DI = value; WR_N = 0;
        tick(3);
        idle();
    endtask
    task automatic expect_byte(input logic [20:0] address, input logic [7:0] value);
        // Combinational readback only: must not disturb a held CPU access.
        A = address; #1;
        if (DO !== value) $fatal(1, "register %h got %h expected %h", address, DO, value);
    endtask
    task automatic check_port(input integer p);
        expect_byte(reg_addr(p, 2), bases[p][7:0]);
        expect_byte(reg_addr(p, 3), bases[p][15:8]);
        expect_byte(reg_addr(p, 4), bases[p][23:16]);
        expect_byte(reg_addr(p, 5), offsets[p][7:0]);
        expect_byte(reg_addr(p, 6), offsets[p][15:8]);
        expect_byte(reg_addr(p, 7), increments[p][7:0]);
        expect_byte(reg_addr(p, 8), increments[p][15:8]);
        expect_byte(reg_addr(p, 9), {1'b0, controls[p]});
    endtask
    task automatic check_shift;
        for (int r = 0; r < 4; r++) expect_byte(21'h1ffae0 + 21'(r), 8'(latch_value >> (r*8)));
        expect_byte(21'h1ffae4, shift_value);
        expect_byte(21'h1ffae5, rotate_value);
    endtask
    task automatic configure(input integer p, input logic [23:0] b,
                             input logic [15:0] o, input logic [15:0] inc,
                             input logic [6:0] c);
        write_reg(reg_addr(p, 9), 0);
        for (int r = 0; r < 3; r++) write_reg(reg_addr(p, r+2), 8'(b >> (r*8)));
        write_reg(reg_addr(p, 5), o[7:0]);
        write_reg(reg_addr(p, 6), o[15:8]);
        write_reg(reg_addr(p, 7), inc[7:0]);
        write_reg(reg_addr(p, 8), inc[15:8]);
        write_reg(reg_addr(p, 9), {1'b0, c});
        bases[p] = b; offsets[p] = o; increments[p] = inc; controls[p] = c;
        check_port(p);
    endtask
    task automatic baseline_model;
        bases[0] = 24'hfffffe; offsets[0] = 16'h0040; increments[0] = 5; controls[0] = 7'h11;
        bases[1] = 24'h1ffff0; offsets[1] = 16'hfffe; increments[1] = 3; controls[1] = 7'h03;
        bases[2] = 24'h000010; offsets[2] = 16'hfff0; increments[2] = 32; controls[2] = 7'h0b;
        bases[3] = 24'h3ffffc; offsets[3] = 16'h000a; increments[3] = 7; controls[3] = 7'h13;
        latch_value = 32'h84d5e6f7; shift_value = 3; rotate_value = 12;
    endtask
    task automatic bump(input integer p);
        if (controls[p][0]) begin
            if (controls[p][4]) bases[p] = bases[p] + 24'(increments[p]);
            else offsets[p] = offsets[p] + increments[p];
        end
    endtask
    task automatic check_ram(input integer p);
        if (RAM_CS_N !== 0 || RAM_A !== expected_addr(p))
            $fatal(1, "port %0d RAM selected=%b address=%h expected=%h", p, !RAM_CS_N, RAM_A, expected_addr(p));
    endtask
    task automatic access_ram(input integer p, input bit paged, input bit writing);
        idle();
        @(negedge CLK);
        A = paged ? 21'h080000 + 21'(p * 8192) : reg_addr(p, writing ? 1 : 0);
        #1; check_ram(p);
        WR_N = !writing; RD_N = writing;
        tick(3); bump(p); check_ram(p);
        tick(4); check_ram(p); // held access increments exactly once
        idle(); check_port(p);
    endtask
    task automatic save_snapshot(input bit active);
        logic [63:0] expected;
        SLEEP = 1; tick(2);
        // Post-load refetch runs while SLEEP is high: address decode must stay live.
        A = 21'h080000; #1; check_ram(0);
        for (int k = 0; k < 5; k++) begin
            SaveStateBus_Adr = 10'(48+k); #1;
            expected = k < 4 ? {1'b0, controls[k], increments[k], offsets[k], bases[k]} :
                               {14'd0, active, rotate_value, shift_value, latch_value, 1'b1};
            if (SaveStateBus_Dout !== expected) $fatal(1, "slot %0d layout/state mismatch", 48+k);
            saved[k] = SaveStateBus_Dout;
        end
        SaveStateBus_Adr = 47; #1;
        if (SaveStateBus_Dout !== 0) $fatal(1, "unselected snapshot bus not zero");
    endtask
    task automatic stage_snapshot;
        SLEEP = 1;
        @(negedge CLK); SaveStateBus_rst = 1;
        tick();
        @(negedge CLK); SaveStateBus_rst = 0;
        for (int k = 0; k < 5; k++) begin
            SaveStateBus_Adr = 10'(48+k); SaveStateBus_Din = saved[k]; SaveStateBus_wren = 1;
            tick(); @(negedge CLK);
        end
        SaveStateBus_wren = 0;
    endtask
    task automatic load_snapshot(input bit frozen = 1);
        SLEEP = frozen; SaveStateBus_load = 1; tick();
        @(negedge CLK); SaveStateBus_load = 0; SLEEP = 0;
        // No settle clock: the derived RAM address must already be usable.
        #1;
    endtask
    task automatic exercise;
        for (int p = 0; p < 4; p++) begin
            access_ram(p, 0, 0);
            access_ram(p, 1, 1);
            access_ram(p, 0, 1);
        end
        write_reg(21'h1ffae4, 1); latch_value = 32'h09abcdee; shift_value = 1; check_shift();
        write_reg(21'h1ffae4, 14); latch_value = 32'h026af37b; shift_value = 14; check_shift();
        write_reg(21'h1ffae5, 3); latch_value = 32'h13579bd8; rotate_value = 3; check_shift();
        write_reg(21'h1ffae5, 15); latch_value = 32'h09abcdec; rotate_value = 15; check_shift();
    endtask

    initial begin
        tick(3); @(negedge CLK); RST_N = 1; tick(2);
        A = 21'h080000; #1;
        if (RAM_CS_N !== 1) $fatal(1, "reset card unexpectedly enabled");
        baseline_model();
        for (int p = 0; p < 4; p++) configure(p, bases[p], offsets[p], increments[p], controls[p]);
        write_reg(21'h1ffae0, 8'hef); write_reg(21'h1ffae1, 8'hcd);
        write_reg(21'h1ffae2, 8'hab); write_reg(21'h1ffae3, 8'h89);
        write_reg(21'h1ffae4, 3); write_reg(21'h1ffae5, 12); check_shift();
        save_snapshot(0);

        // Every writable register is attacked during freeze, without a load to hide leaks.
        for (int p = 0; p < 4; p++) begin
            for (int r = 2; r <= 10; r++) write_reg(reg_addr(p, r), 8'h55);
            check_port(p);
            write_reg(reg_addr(p, 0), 8'h55);
            write_reg(21'h080000 + 21'(p*8192), 8'h55);
            check_port(p);
        end
        for (int r = 0; r < 6; r++) write_reg(21'h1ffae0 + 21'(r), 8'h55);
        check_shift();
        SLEEP = 0; tick(2);
        exercise();
        for (int p = 0; p < 4; p++) configure(p, 24'h123456 + 24'(p), 16'h3456, 16'h789a, 7'h02);
        write_reg(21'h1ffae0, 0);
        // Reset also destroys ena, proving restored page mapping is not accidental.
        RST_N = 0; tick(2); RST_N = 1; tick(2);
        stage_snapshot(); baseline_model();
        A = 21'h080000; load_snapshot(); check_ram(0);
        for (int p = 0; p < 4; p++) check_port(p);
        check_shift(); exercise();

        // Save AFTER a consumed edge. Idle levels during sleep must not erase old_acc.
        idle(); @(negedge CLK); A = reg_addr(0, 0); RD_N = 0;
        tick(3); bump(0); check_ram(0); save_snapshot(1);
        WR_N = 1; RD_N = 1; tick(3);
        @(negedge CLK); A = reg_addr(0, 0); RD_N = 0; SLEEP = 0;
        tick(4); check_ram(0); // no duplicate on ordinary sleep/resume
        idle(); access_ram(0, 0, 0); // deliberately change live state
        stage_snapshot();
        bases[0] = saved[0][23:0]; offsets[0] = saved[0][39:24];
        // Restoring old_acc=1 while the same access remains asserted must not increment.
        A = reg_addr(0, 0); RD_N = 0;
        load_snapshot(); check_ram(0); tick(4); check_ram(0);
        idle(); access_ram(0, 1, 0); // fresh edge still increments

        // Save BEFORE an edge. A pending write held over load executes once, after wake.
        idle(); save_snapshot(0);
        stage_snapshot();
        A = reg_addr(1, 1); WR_N = 0; RD_N = 1;
        tick(3); check_port(1); A = reg_addr(1, 1);
        load_snapshot(0); check_ram(1); // load wins even with SLEEP low
        tick(3); bump(1); check_ram(1);
        tick(4); check_ram(1); idle();

        // Offset-add trigger modes still work after the restored control is used.
        for (int mode = 1; mode <= 3; mode++) begin
            configure(3, 24'hfffff0, 16'h0008, 0, 7'(mode << 5));
            write_reg(reg_addr(3, mode == 1 ? 5 : mode == 2 ? 6 : 10), mode == 1 ? 8'h30 : 8'h00);
            if (mode == 1) offsets[3] = 16'h0030;
            bases[3] = bases[3] + 24'(offsets[3]);
            check_port(3);
        end
        $display("tb_arcade_snapshot PASSED: real Arcade/eRegs, four ports, wrap, shifts, freeze and held-edge restore");
        $finish;
    end
endmodule
