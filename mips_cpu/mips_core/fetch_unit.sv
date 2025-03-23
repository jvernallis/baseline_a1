/*
 * d_cache.sv
 * Author: Pravin P. Prabhu, Zinsser Zhang
 * Last Revision: 04/08/2018
 *
 * This module provides pc to i_cache to fetch the next instruction. Two outputs
 * exist. o_pc_current.pc is registered and represent the current pc, i.e. the
 * address of instruction needed to be fetched during the current cycle.
 * o_pc_next.pc is not registered and represent the next pc.
 *
 * All addresses in mips_core are byte addresses (26-bit), so all pc are also
 * byte addresses. Thus, it increases by 4 every cycle (without hazards).
 *
 * See wiki page "Synchronous Caches" for details.
 */
`include "mips_core.svh"

module fetch_unit (
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	thread_control_ifc.in i_tc,

	// Stall
	hazard_control_ifc.in i_hc,

	// Load pc
	load_pc_ifc.in i_load_pc,
	branch_prediction_ifc.in i_branch_prediction,

	// Output pc
	pc_ifc.out o_pc_current,
	pc_ifc.out o_pc_next
);

	always_comb
	begin
		if(!i_tc.thread_switch) begin
			if (!i_hc.stall) begin
				if(i_load_pc.we) begin
					o_pc_next.pc = {i_tc.thread_id, i_load_pc.new_pc[`ADDR_WIDTH - 2 : 0]};
				end
				else
				begin
					if(i_branch_prediction.is_branch & i_branch_prediction.prediction)
						o_pc_next.pc = {i_tc.thread_id, i_branch_prediction.target[`ADDR_WIDTH - 2 : 0]};
					else
						o_pc_next.pc = o_pc_current.pc + `ADDR_WIDTH'd4;
				end
			end else begin
				o_pc_next.pc = {i_tc.thread_id, o_pc_current.pc[`ADDR_WIDTH - 2 : 0]};
			end
		end 
		else begin
			o_pc_next.pc = i_tc.thread_resume_pc[~i_tc.thread_id];
		end
	end

	always_ff @(posedge clk)
	begin
		if(~rst_n)
			o_pc_current.pc <= {i_tc.thread_id, {`ADDR_WIDTH - 1{1'b0}}};	// Start point of programs are always 0x0
		else
		begin
			if(i_tc.thread_switch)
				o_pc_current.pc <= i_tc.thread_resume_pc[~i_tc.thread_id];
			else
				o_pc_current.pc <= {i_tc.thread_id, o_pc_next.pc[`ADDR_WIDTH - 2 : 0]};
		end
	end

endmodule
