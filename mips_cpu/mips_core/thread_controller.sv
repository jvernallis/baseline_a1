`include "mips_core.svh"

// `ifdef SIMULATION
// import "DPI-C" function void stats_event (input string e);
// `endif

module thread_controller (
    input clk,    // Clock
	input rst_n,  // Synchronous reset active low

    thread_control_ifc i_tc, //Interface directly manipulated (without modports) - needed to change thread_id

	input mem_busy,
    pc_ifc.in e2m_pc,
    pc_ifc.out if_pc_current,
);

	//Thread switch condition handlers
    always_comb
    begin
		i_tc.thread_switch = 1'b0;
        if(i_tc.current_thread_done) 
            i_tc.thread_switch = 1'b1;
		
		if(mem_busy & i_tc.thread_switch_available)
			i_tc.thread_switch = 1'b1;
	end

	always_comb
	begin
		if(~mem_busy & ~i_tc.thread_done[0]) 
			i_tc.thread_ready[0] = 1'b1;
		if(~mem_busy & ~i_tc.thread_done[1]) 
			i_tc.thread_ready[1] = 1'b1;
	end

	always_comb
	begin
		i_tc.thread_switch_available = (~i_tc.thread_done[0] & ~i_tc.thread_done[1]) & (i_tc.thread_ready[0] & i_tc.thread_ready[1]);
	end

	always_comb
	begin
		
	end

    always_ff @(posedge clk)
	begin
		if(~rst_n) begin
			i_tc.thread_id <= 1'b0;
			i_tc.thread_done[0] <= 1'b0;
			i_tc.thread_done[1] <= 1'b0;
			i_tc.thread_resume_pc[0] <= {1'b0, {`ADDR_WIDTH - 1{1'b0}}};
			i_tc.thread_resume_pc[1] <= {1'b1, {`ADDR_WIDTH - 1{1'b0}}};
		end

		if(i_tc.current_thread_done) begin
			i_tc.thread_done[i_tc.thread_id] <= 1'b1;
			i_tc.thread_ready[i_tc.thread_id] <= 1'b0;
		end
		
		if(i_tc.thread_switch) begin
			i_tc.thread_ready[i_tc.thread_id] <= 1'b0;

			i_tc.thread_id <= ~i_tc.thread_id; 
			i_tc.thread_resume_pc[i_tc.thread_id] <= e2m_pc.pc;
		end
	end

endmodule