`include "mips_core.svh"

// `ifdef SIMULATION
// import "DPI-C" function void stats_event (input string e);
// `endif

module thread_controller (
    input clk,    // Clock
	input rst_n,  // Synchronous reset active low

    thread_control_ifc i_tc, //Interface directly manipulated (without modports) - needed to change thread_id

    pc_ifc.in e2m_pc,
    pc_ifc.out if_pc_current,
);

    always_comb
    begin
        if(i_tc.current_thread_done) 
            i_tc.thread_switch = 1'b1;
        else
            i_tc.thread_switch = 1'b0;
    end

    always_ff @(posedge clk)
	begin
		if(~rst_n) begin
			i_tc.thread_id <= 1'b0;
			i_tc.thread_0_done <= 1'b0;
			i_tc.thread_1_done <= 1'b0; 
			i_tc.thread_resume_pc[0] <= {1'b0, {`ADDR_WIDTH - 1{1'b0}}};
			i_tc.thread_resume_pc[1] <= {1'b1, {`ADDR_WIDTH - 1{1'b0}}};
		end

		//The test: Switch from nqueens to coin on a lw, let coin run to completion.
		if(i_tc.current_thread_done) begin
			if(i_tc.thread_id == 1'b0)
				i_tc.thread_0_done <= 1'b1;
			else if(i_tc.thread_id == 1'b1)
				i_tc.thread_1_done <= 1'b1;

			i_tc.thread_id <= ~i_tc.thread_id; 
			if_pc_current.pc <= i_tc.thread_resume_pc[~i_tc.thread_id];
		end
	end

endmodule