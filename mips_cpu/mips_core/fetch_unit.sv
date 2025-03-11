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
`ifdef SIMULATION
import "DPI-C" function void thread_event (input int aThreadId);
`endif

module fetch_unit (
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Stall
	hazard_control_ifc.in i_hc,

	// Load pc
	load_pc_ifc.in i_load_pc,
	branch_prediction_ifc.in i_branch_prediction,

	// Output pc
	pc_ifc.out o_pc_current,
	pc_ifc.out o_pc_next
);
	logic thread_id;

	always_comb
	begin
		if (!i_hc.stall) begin
			o_pc_next.thread_id = ~o_pc_current.thread_id;//dummy
			if(i_load_pc.we) begin
				o_pc_next.pc = i_load_pc.new_pc;
				// $display("Fetch unit detected overload, loading next pc %h", o_pc_next.pc);
			end
			else
			begin
				if(i_branch_prediction.valid & i_branch_prediction.prediction)begin
					o_pc_next.pc = i_branch_prediction.target;
					
				end
				else
					o_pc_next.pc = o_pc_current.pc + `ADDR_WIDTH'd4;
			end

			// if(o_pc_next.pc == 12'h0fc)
			// 	// $display("We have a problem here, next PC is fc. Current PC is %h", o_pc_current.pc);
			// if(o_pc_next.pc == 12'h100)
			// 	// $display("We have a problem here, next PC is 100. Current PC is %h", o_pc_current.pc);
		end else begin
			o_pc_next.pc = o_pc_current.pc;
			
		end
	end

	always_ff @(posedge clk)
	begin
		if(~rst_n) begin
			o_pc_current.pc <= '0;	// Start point of programs are always 0x0
			o_pc_current.thread_id <= '0; //dummy
			thread_id <= '0;
		end
		else
		begin
			o_pc_current.pc <= o_pc_next.pc;
			o_pc_current.thread_id <= o_pc_next.thread_id;
		end
	end

	`ifdef SIMULATION
	always_ff @(posedge clk)
	begin
		if(o_pc_current.thread_id != o_pc_next.thread_id) thread_event(o_pc_next.thread_id);
	end
	`endif
endmodule
