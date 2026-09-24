module savestate_ui #(parameter INFO_TIMEOUT_BITS)
(
	input            clk,    
   input     [10:0] ps2_key,     
	input            allow_ss,    
	input            joySS   ,    
	input            joyRight,    
	input            joyLeft ,    
	input            joyDown ,    
	input            joyUp   ,    
	input      [1:0] status_slot, 
	input      [1:0] OSD_saveload,
	output reg       ss_save,
	output reg       ss_load,
	output reg       ss_info_req,
	output reg [7:0] ss_info,
	input            ss_busy,
	input            ss_failed,
	input      [1:0] ss_active_slot,
	input            ss_active_load,
	output reg       statusUpdate,
	output     [1:0] selected_slot
);

reg [1:0] ss_base = 0;

reg lastRight  = 1'b0;
reg lastLeft   = 1'b0;
reg lastDown   = 1'b0;
reg lastUp     = 1'b0;

reg [(INFO_TIMEOUT_BITS-1):0] InfoWaitcnt = 0;

reg        slotswitched   = 1'b0;
reg [1:0]  lastOSDsetting = 2'b0;

reg        busy_d       = 1'b0;
reg        req_was_load = 1'b0;
reg [1:0]  completed_slot = 0;

assign selected_slot = ss_base;

wire pressed = ps2_key[9];

always @(posedge clk) begin
	reg old_state;
	reg alt = 0;
	reg [1:0] old_st;

	old_state <= ps2_key[10];
	if (old_state != ps2_key[10] && ps2_key[7:0] == 8'h11)
		alt <= pressed;
	
	lastRight <= joyRight;
	lastLeft  <= joyLeft; 
	lastDown  <= joyDown; 
	lastUp    <= joyUp;   
	
	slotswitched <= 1'b0;
	
	ss_save      <= 1'b0;
	ss_load      <= 1'b0;
	ss_info_req  <= 1'b0;
	statusUpdate <= 1'b0;
	
	lastOSDsetting <= status_slot;

	// HPS restores the selected slot before a game is mounted.
	if (lastOSDsetting != status_slot) begin
		ss_base      <= status_slot;
		statusUpdate <= 1'b1;
	end

	if(allow_ss) begin

		// keyboard
		if(old_state != ps2_key[10]) begin
			case(ps2_key[7:0])
				'h05: begin ss_save <= pressed & alt; ss_load <= pressed & ~alt; ss_base <= 0; statusUpdate <= 1'b1; end // F1
				'h06: begin ss_save <= pressed & alt; ss_load <= pressed & ~alt; ss_base <= 1; statusUpdate <= 1'b1; end // F2
				'h04: begin ss_save <= pressed & alt; ss_load <= pressed & ~alt; ss_base <= 2; statusUpdate <= 1'b1; end // F3
				'h0C: begin ss_save <= pressed & alt; ss_load <= pressed & ~alt; ss_base <= 3; statusUpdate <= 1'b1; end // F4
			endcase
		end
		
		// gamepad
		if (joySS) begin
			// timeout with no button pressed -> help text
			InfoWaitcnt <= InfoWaitcnt + 1'b1;
			if (InfoWaitcnt[(INFO_TIMEOUT_BITS-1)]) begin
				ss_info     <= 7'd5;
				ss_info_req <= 1'b1;
				InfoWaitcnt <= 25'b0;
			end
			// switch slot
			if (joyRight & ~lastRight & ss_base < 3) begin
				ss_base      <= ss_base + 1'd1;
				statusUpdate <= 1'b1;
				slotswitched <= 1'b1;
				InfoWaitcnt  <= 25'b0;
			end
			if (joyLeft & ~lastLeft & ss_base > 0) begin
				ss_base      <= ss_base - 1'd1;
				statusUpdate <= 1'b1;
				slotswitched <= 1'b1;
				InfoWaitcnt  <= 25'b0;
			end
			// joySS is the modifier for all four directions.
			if (joyDown & ~lastDown) begin
				ss_save     <= 1'b1;
				InfoWaitcnt <= 25'b0;
			end
			if (joyUp & ~lastUp) begin
				ss_load     <= 1'b1;
				InfoWaitcnt <= 25'b0;
			end
		end else begin
			InfoWaitcnt <= 25'b0;
		end
		
		// OSD
		old_st <= OSD_saveload;
		if(old_st[0] ^ OSD_saveload[0]) ss_save <= OSD_saveload[0];
		if(old_st[1] ^ OSD_saveload[1]) ss_load <= OSD_saveload[1];

		// infotexts
		if (slotswitched) begin
			ss_info     <= 7'd6 + ss_base;
			ss_info_req <= 1'b1;
		end

	end
	// Capture the dispatched operation, not input queued while it runs.
	// Report successful completion even if commands have since been inhibited.
	busy_d <= ss_busy;
	if (ss_busy & ~busy_d) begin
		req_was_load <= ss_active_load;
		completed_slot <= ss_active_slot;
	end
	if (busy_d & ~ss_busy & ~ss_failed) begin
		ss_info     <= 7'd10 + {completed_slot, req_was_load};
		ss_info_req <= 1'b1;
	end

end

endmodule

