`timescale 1ps/1ps
module altddio_out #(
 parameter extend_oe_disable="OFF", intended_device_family="Cyclone V",
 parameter invert_output="OFF", lpm_hint="UNUSED", lpm_type="altddio_out",
 parameter oe_reg="UNREGISTERED", power_up_high="OFF", width=1
)(input [width-1:0] datain_h, datain_l, input outclock,
 output [width-1:0] dataout, input aclr, aset, oe, outclocken, sclr, sset);
 assign dataout = outclock ? datain_h : datain_l;
endmodule

module tb_sdram_restart;
 localparam integer PERIOD = 11640;
 reg clk=0;
 always #(PERIOD/2) clk=~clk;
 reg init=1, clkref=0, rd=0, we=0, we_req=0;
 reg [24:0] raddr=0, waddr=0;
 reg [15:0] din=0;
 wire [15:0] dq;
 wire [12:0] a;
 wire [1:0] bank;
 wire ras_n, cas_n, we_n, cs_n, sclk, cke, dqml, dqmh, ready, ack;
 wire [7:0] dout;
 sdram dut(.SDRAM_DQ(dq),.SDRAM_A(a),.SDRAM_BA(bank),
 .SDRAM_nRAS(ras_n),.SDRAM_nCAS(cas_n),.SDRAM_nWE(we_n),
 .SDRAM_nCS(cs_n),.SDRAM_CLK(sclk),.SDRAM_CKE(cke),
 .SDRAM_DQML(dqml),.SDRAM_DQMH(dqmh),.init(init),.clk(clk),
 .clkref(clkref),.rd(rd),.rd_rdy(ready),.raddr(raddr),.dout(dout),
 .waddr(waddr),.din(din),.we(we),.we_req(we_req),.we_ack(ack));

 reg [12:0] row[0:3];
 reg [15:0] read_data=0;
 reg drive=0;
 integer read_delay=0, drive_left=0;
 time last_refresh=0;
 integer refreshes=0, reads=0, writes=0;
 reg [15:0] memory[0:3]='{16'ha55a,16'h3cc3,16'h6996,16'hf00f};
 assign dq=drive ? read_data : 16'hzzzz;
 function automatic [15:0] memory_word(input [12:0] r,input [8:0] col);
   case ({r,col})
     22'h004000: memory_word=memory[0];
     22'h004001: memory_word=memory[1];
     22'h004002: memory_word=memory[2];
     22'h004003: memory_word=memory[3];
     default: memory_word=16'hbeef;
   endcase
 endfunction
 always @(posedge sclk) begin
   if (drive_left>0) begin
     drive_left=drive_left-1;
     if (drive_left==0) drive=0;
   end
   if (read_delay>0) begin
     read_delay=read_delay-1;
     if (read_delay==0) begin drive=1; drive_left=2; end
   end
   if (!cs_n && cke) begin
     case ({ras_n,cas_n,we_n})
       3'b001: begin
         if (last_refresh!=0 && $time-last_refresh<66000)
           $fatal(1,"SDRAM tRFC violation: REFRESH %0t ps after REFRESH",$time-last_refresh);
         last_refresh=$time; refreshes=refreshes+1;
       end
       3'b011: begin
         if (last_refresh!=0 && $time-last_refresh<66000)
           $fatal(1,"SDRAM tRFC violation: ACTIVE %0t ps after REFRESH",$time-last_refresh);
         row[bank]=a;
       end
       3'b101: begin
         read_data=memory_word(row[bank],{a[8],a[7:0]});
         read_delay=2; reads=reads+1;
       end
       3'b100: begin
         if ({row[bank],a[8:0]}<22'h004000 || {row[bank],a[8:0]}>22'h004003)
           $fatal(1,"SDRAM write address mismatch");
         if (!dqml) memory[a[1:0]][7:0]=dq[7:0];
         if (!dqmh) memory[a[1:0]][15:8]=dq[15:8];
         writes=writes+1;
       end
       default: begin end
     endcase
   end
 end

 task automatic restart_read(input integer phase,input integer byte_index);
   reg [7:0] expected;
   integer cycles;
   begin
     @(posedge sclk);
     while ({ras_n,cas_n,we_n}!=3'b001) @(posedge sclk);
     repeat(phase) @(posedge sclk);
     #(PERIOD/4);
     raddr=25'h004000+byte_index[24:0]; rd=1; clkref=1;
     repeat(2) @(posedge clk);
     #1; clkref=0; rd=0;
     cycles=0;
     while (!ready && cycles<64) begin @(posedge clk); #1; cycles=cycles+1; end
     if (!ready) $fatal(1,"SDRAM read did not finish after restart phase%0d",phase);
     expected = byte_index[0] ? memory[byte_index/2][15:8] : memory[byte_index/2][7:0];
     if (dout!==expected) $fatal(1,"SDRAM restart phase%0d returned %02x expected %02x",phase,dout,expected);
     repeat(20) @(posedge clk);
   end
 endtask

 task automatic restart_write(input integer phase,input integer byte_index,input bit byte_write);
   integer cycles, previous_writes;
   reg [15:0] previous_word;
   begin
     @(posedge sclk);
     while ({ras_n,cas_n,we_n}!=3'b001) @(posedge sclk);
     repeat(phase) @(posedge sclk);
     #(PERIOD/4);
     previous_writes=writes;
     waddr=25'h004000+byte_index[24:0];
     din=16'h2011+phase[15:0]*16'h0101;
     if (byte_write) din=~din;
     previous_word=memory[byte_index/2];
     we=byte_write;
     if (!byte_write) we_req=~we_req;
     clkref=1;
     repeat(2) @(posedge clk);
     #1; clkref=0; we=0;
     cycles=0;
     while ((writes==previous_writes || ack!=we_req) && cycles<64) begin
       @(posedge clk); #1; cycles=cycles+1;
     end
     if (writes!=previous_writes+1 || ack!=we_req)
       $fatal(1,"SDRAM write did not complete once after restart phase%0d",phase);
     repeat(20) @(posedge clk);
     if (writes!=previous_writes+1) $fatal(1,"SDRAM write duplicated");
     if (byte_write) begin
       if (byte_index[0] && memory[byte_index/2][15:8]!==din[15:8]) $fatal(1,"high-byte write lost");
       if (byte_index[0] && memory[byte_index/2][7:0]!==previous_word[7:0]) $fatal(1,"high-byte write changed low byte");
       if (!byte_index[0] && memory[byte_index/2][15:8]!==previous_word[15:8]) $fatal(1,"low-byte write changed high byte");
       if (!byte_index[0] && memory[byte_index/2][7:0]!==din[7:0]) $fatal(1,"low-byte write lost");
     end
     else if (memory[byte_index/2]!==din) $fatal(1,"word write lost");
   end
 endtask

 task automatic minimum_period_reads;
   time next_request;
   reg [7:0] expected;
   integer cycles, byte_index;
   begin
     @(posedge sclk);
     while ({ras_n,cas_n,we_n}!=3'b001) @(posedge sclk);
     #(PERIOD/4);
     next_request=$time;
     for(integer i=0;i<32;i=i+1) begin
       byte_index=i%8;
       raddr=25'h004000+byte_index[24:0]; rd=1; clkref=1;
       repeat(2) @(posedge clk);
       #1; clkref=0; rd=0;
       cycles=0;
       while (!ready && cycles<10) begin @(posedge clk); #1; cycles=cycles+1; end
       if (!ready) $fatal(1,"SDRAM read exceeded minimum CPU period");
       expected=byte_index[0] ? memory[byte_index/2][15:8] : memory[byte_index/2][7:0];
       if (dout!==expected) $fatal(1,"SDRAM consecutive read returned %02x expected %02x",dout,expected);
       next_request=next_request+12*PERIOD;
       if ($time>=next_request) $fatal(1,"SDRAM read missed next request");
       #(next_request-$time);
     end
     repeat(20) @(posedge clk);
   end
 endtask
 initial begin
   #100000; init=0;
   #10000000;
   for(integer phase=0;phase<16;phase=phase+1) restart_read(phase,phase%8);
   for(integer phase=0;phase<16;phase=phase+1) begin
     restart_write(phase,phase%8,0);
     restart_read(phase,phase%8);
     restart_read(phase,(phase%8)^1);
     restart_write(phase,phase%8,1);
     restart_read(phase,phase%8);
     restart_read(phase,(phase%8)^1);
   end
   minimum_period_reads();
   if(reads!=112 || writes!=32) $fatal(1,"SDRAM lost or duplicated requests: reads%0d writes%0d",reads,writes);
   $display("TB SDRAM RESTART PASSED: all16 phases, exact bytes, word/byte writes, minimum CPU period and tRFC");
   $finish;
 end
 initial begin #100000000; $fatal(1,"SDRAM restart watchdog"); end
endmodule
