//
// ddram.v
// Copyright (c) 2020 Sorgelig
//
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version. 
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ------------------------------------------
//


module ddram
(
	input         DDRAM_CLK,

	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	input         clkref,

	input  [27:0] wraddr,
	input  [15:0] din,
	input         we,
	output reg    we_rdy,
	input         we_req,
	output reg    we_ack,

	input  [27:0] rdaddr,
	output  [7:0] dout,
	input         rd,
	output reg    rd_rdy,

	// ch1: 64-bit savestate channel, cache-bypass (NES_MiSTer pattern).
	// Only active while the core is frozen (sleep_savestate) — ch0 quiescent.
	input  [27:1] ch1_addr,
	output [63:0] ch1_dout,
	input  [63:0] ch1_din,
	input         ch1_req,
	input         ch1_rnw,
	input   [7:0] ch1_be,
	output reg    ch1_ready
);

assign DDRAM_BURSTCNT = ram_burst;
assign DDRAM_BE       = ch1_act ? (DDRAM_RD ? 8'hFF : ch1_be_r)
                                : (DDRAM_RD ? 8'hFF : ({6'd0,~b,1'b1} << {ram_addr[2:1],ram_addr[0] & b}));
assign DDRAM_ADDR     = {4'b0011, ram_addr[27:3]}; // RAM at 0x30000000
assign DDRAM_DIN      = ram_data;
assign DDRAM_WE       = ram_write;

assign dout = data;
assign ch1_dout = ch1_q;

reg  [7:0] ram_burst;
reg [63:0] ram_data;
reg [27:0] ram_addr;
reg  [7:0] data;
reg        ram_write = 0;
reg        b;
reg        start;
reg  [2:0] state = 0;

// ch1 (savestate) regs
reg        ch1_act = 0;
reg        ch1_rq  = 0;
reg        ch1_req_d = 0;
reg  [7:0] ch1_be_r;
reg [63:0] ch1_q;
reg        ram_read = 0;

// ch1_req comes from clk_sys (2:1 related): one engine request is TWO
// clk_ram cycles wide.  Latch it on its RISING EDGE only — level-latching
// re-armed ch1_rq on the request's second cycle after the grant cleared
// it, issuing a PHANTOM transaction whose stale completion was then
// consumed by the engine's NEXT read: every back-to-back internals word K
// received word K-1's data (RAM regions survived only because their
// byte-distribution gap swallowed the phantom).  Proven on hardware by
// the MPR register file containing the CPU_2 word's bytes verbatim.
wire ch1_new = ch1_req & ~ch1_req_d;

reg [27:0] addr;

always @(posedge DDRAM_CLK) begin
	reg        old_ref;
	reg[127:0] ram_q;

	old_ref <= clkref;
	start <= ~old_ref & clkref;

	ch1_req_d <= ch1_req;
	ch1_rq    <= ch1_rq | ch1_new;

	// ch1_ready is STICKY: set on completion, cleared only by the next
	// request.  ddram runs on clk_ram while the savestate engine samples
	// bus_out_done on clk_sys (same PLL, 2:1): a single-clk_ram ready pulse
	// lands on the invisible phase ~half the time — reads (whose completion
	// phase depends on DDR latency) then hang the LOAD forever.  A level
	// held until the next request is phase-immune; the engine only checks
	// done in its per-transaction wait states, so a stale '1' from the
	// previous transaction is cleared by the new request before it is ever
	// sampled.
	if(ch1_new) ch1_ready <= 0;

	if(start) begin
		if(we) we_rdy <= 0;
		else if(rd) rd_rdy <= 0;
	end

	ram_burst <= 1;
	addr <= rdaddr;

	// ch1 read-data return.  DDRAM_DOUT_READY (avalon readdatavalid) is a
	// single-cycle pulse INDEPENDENT of DDRAM_BUSY (waitrequest): with the
	// scaler hammering the DDR port, READY frequently lands on a BUSY cycle,
	// and sampling it only under !DDRAM_BUSY (as the FSM below does) loses
	// the word and hangs the savestate LOAD forever.  Capture it here,
	// unconditionally.  (Cache fills have their own capture inside
	// cache_2way, fed by the ~ch1_act-masked ack below.)
	if(state == 3 && DDRAM_DOUT_READY) begin
		ch1_q     <= DDRAM_DOUT;
		ch1_ready <= 1;
		state     <= 0;
	end

	if(!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;
		case(state)
			0: begin
					we_rdy <= 1;
					rd_rdy <= 1;
					cache_cs <= 0;
					ch1_act  <= 0;
					if(ch1_rq || ch1_new) begin
						// savestate channel: 64-bit direct, cache bypassed.
						// One grant per request (edge-latched) — see ch1_new above.
						ch1_rq      <= 0;
						ch1_act     <= 1;
						ram_data    <= ch1_din;
						ch1_be_r    <= ch1_be;
						ram_addr    <= {ch1_addr, 1'b0};
						if(~ch1_rnw) begin
							ram_write <= 1;
							ch1_ready <= 1;
						end
						else begin
							ram_read  <= 1;
							state     <= 3;
						end
					end
					else if(we_ack != we_req) begin
						we_ack     <= we_req;
						ram_data   <= {4{din}};
						ram_addr   <= wraddr;
						ram_write  <= 1;
						b          <= 0;
					end
					else if(start) begin
						if(we) begin
							we_rdy    <= 0;
							ram_data  <= {8{din[7:0]}};
							ram_addr  <= addr;
							ram_write <= 1;
							b         <= 1;
							cache_cs  <= 1;
							cache_we  <= 1;
							state     <= 1;
						end
						else if(rd) begin
							ram_addr  <= addr;
							rd_rdy    <= 0;
							cache_cs  <= 1;
							cache_we  <= 0;
							state     <= 2;
						end
					end
				end

			1: if(cache_wrack) begin
					cache_cs <= 0;
					we_rdy <= 1;
					state  <= 0;
				end

			2: if(cache_rdack) begin
					cache_cs <= 0;
					data <= ram_addr[0] ? cache_do[15:8] : cache_do[7:0];
					rd_rdy <= 1;
					state  <= 0;
				end

			// state 3 (ch1 read pending) is handled above, outside the
			// !DDRAM_BUSY gate — see comment there.
		endcase
	end
end

wire [15:0] cache_do;
wire        cache_rdack;
wire        cache_wrack;
reg         cache_cs;
reg         cache_we;
wire        cache_rd_req;

// ch1 reads drive DDRAM_RD directly; cache reads keep their own request line.
// The cache's read-ack MUST be masked while a ch1 read is in flight, or the
// spurious DDRAM_DOUT_READY would corrupt a cache line fill.
assign DDRAM_RD = ram_read | cache_rd_req;

cache_2way cache
(
	.clk(DDRAM_CLK),
	.rst(we_ack != we_req),

	.cache_enable(1),

	.cpu_cs(cache_cs),
	.cpu_adr(addr[27:1]),
	.cpu_bs({addr[0],~addr[0]}),
	.cpu_we(cache_we),
	.cpu_rd(~cache_we),
	.cpu_dat_w(ram_data[15:0]),
	.cpu_dat_r(cache_do),
	.cpu_ack(cache_rdack),
	.wb_en(cache_wrack),

	.mem_dat_r(DDRAM_DOUT),
	.mem_read_req(cache_rd_req),
	.mem_read_ack(DDRAM_DOUT_READY & ~ch1_act)
);

endmodule
