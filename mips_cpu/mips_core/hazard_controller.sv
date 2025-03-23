/*
 * hazard_controller.sv
 * Author: Zinsser Zhang
 * Last Revision: 03/13/2022
 *
 * hazard_controller collects feedbacks from each stage and detect whether there
 * are hazards in the pipeline. If so, it generate control signals to stall or
 * flush each stage. It also contains a branch_controller, which talks to
 * a branch predictor to make a prediction when a branch instruction is decoded.
 *
 * It also contains simulation only logic to report hazard conditions to C++
 * code for execution statistics collection.
 *
 * See wiki page "Hazards" for details.
 * See wiki page "Branch and Jump" for details of branch and jump instructions.
 */
`include "mips_core.svh"

`ifdef SIMULATION
import "DPI-C" function void stats_event (input string e);
`endif

module hazard_controller (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	thread_control_ifc.in i_tc,

	// Feedback from IF
	cache_output_ifc.in if_i_cache_output,
	branch_prediction_ifc.in if_branch_prediction, 
	
	// Feedback from DEC
	pc_ifc.in dec_pc,
	branch_prediction_ifc.in dec_branch_prediction, 
	branch_resolution_ifc.in dec_branch_resolved,
	alu_pass_through_ifc.in dec_pass_through,

	branch_prediction_ifc.out if_branch_prediction_pass_through,

	input lw_hazard,
	input mem_done,
	input mem_busy,


	// Hazard control output
	hazard_control_ifc.out i2i_hc,
	hazard_control_ifc.out i2d_hc,
	hazard_control_ifc.out d2e_hc,
	hazard_control_ifc.out e2m_hc,
	hazard_control_ifc.out m2w_hc,

	// Load pc output
	load_pc_ifc.out load_pc
);
	// We have total 6 potential hazards
	logic ic_miss;			// I cache miss
	// Gone without delay slots:
	//logic ds_miss;			// Delay slot miss
	
	logic dec_overload;		// Branch prediction wrong
	//    lw_hazard;		// Load word hazard (input from forward unit)
	logic dc_busy_multithread;		// D cache access when other (active) thread is being serviced
	logic dc_miss_single_thread;	// D cache miss running in single-thread mode

	logic thread_switch;

	logic branch_correct;
	logic branch_missed; 			// Branch was not identified as a branch from BTB
	logic branch_misidentified; 	// Non-branch was identified as a branch from BTB
	logic branch_target_wrong;		// Branch target was incorrect
	logic branch_mispredicted;		// Branch decision was incorrect

	// Determine if we have these hazards
	always_comb
	begin
		ic_miss = ~if_i_cache_output.valid;

		dec_overload =
			(!dec_branch_prediction.is_branch & dec_branch_resolved.is_branch)
			| (dec_branch_prediction.is_branch & !dec_branch_resolved.is_branch)
			| (dec_branch_resolved.is_branch & 
			((dec_branch_prediction.target != dec_branch_resolved.target)
			| (dec_branch_prediction.prediction != dec_branch_resolved.outcome)));

		dc_busy_multithread = (dec_pass_through.is_mem_access) & (mem_busy & ~i_tc.thread_switch);
		dc_miss_single_thread = ~mem_done & ~i_tc.thread_switch_available;

		thread_switch = i_tc.thread_switch;
	end

	// Logging
	always_comb begin
		// Overloaded needed only if the wrong path was taken
		branch_correct = (dec_branch_prediction.is_branch & dec_branch_resolved.is_branch) & 
						 (dec_branch_prediction.prediction == dec_branch_resolved.outcome) &
						 (dec_branch_prediction.target == dec_branch_resolved.target);

		branch_missed = 0;
		branch_misidentified = 0;
		branch_target_wrong = 0;
		branch_mispredicted = 0;
		if(dec_overload) begin
			if (!dec_branch_prediction.is_branch & dec_branch_resolved.is_branch) 
				branch_missed = 1;
			else if (dec_branch_prediction.is_branch & !dec_branch_resolved.is_branch) 
				branch_misidentified = 1;
			else if (dec_branch_prediction.target != dec_branch_resolved.target)
				branch_target_wrong = 1;
			else if (dec_branch_prediction.prediction != dec_branch_resolved.outcome)
				branch_mispredicted = 1;
		end
	end

	always_comb
	begin
		if(!if_flush) 
		begin
			i2d_pred_pass_through.is_branch = i2d_pred.is_branch;
			i2d_pred_pass_through.target = i2d_pred.target;
			i2d_pred_pass_through.prediction = i2d_pred.prediction;
		end
		else
		begin
			i2d_pred_pass_through.is_branch = '0;
			i2d_pred_pass_through.target = '0;
			i2d_pred_pass_through.prediction = NOT_TAKEN;
		end
	end

	// Control signals
	logic if_stall, if_flush;
	logic dec_stall, dec_flush;
	logic ex_stall, ex_flush;
	logic mem_stall, mem_flush;
	// wb doesn't need to be stalled or flushed
	// i.e. any data goes to wb is finalized and waiting to be commited

	/*
	 * Now let's go over the solution of all hazards
	 * ic_miss:
	 *     if_stall, if_flush
	 * ds_miss:
	 *     dec_stall, dec_flush (if_stall and if_flush handled by ic_miss)
	 * dec_overload:
	 *     load_pc
	 * ex_overload:
	 *     load_pc, ~if_stall, if_flush
	 * lw_hazard:
	 *     dec_stall, dec_flush
	 * dc_miss:
	 *     mem_stall, mem_flush
	 *
	 * The only conflict here is between ic_miss and ex_overload.
	 * ex_overload should have higher priority than ic_miss. Because i cache
	 * does not register missed request, it's totally fine to directly overload
	 * the pc value.
	 *
	 * In addition to above hazards, each stage should also stall if its
	 * downstream stage stalls (e.g., when mem stalls, if & dec & ex should all
	 * stall). This has the highest priority.
	 */

	always_comb
	begin : handle_if
		if_stall = 1'b0;
		if_flush = 1'b0;

		if (ic_miss)
		begin
			if_stall = 1'b1;
			if_flush = 1'b1;
		end

		if(dec_overload)  
		begin
			if_stall = 1'b0;
			if_flush = 1'b1;
		end

		if (dec_stall)
			if_stall = 1'b1;

		if (thread_switch)
			if_flush = 1'b1;
	end

	always_comb
	begin : handle_dec
		dec_stall = 1'b0;
		dec_flush = 1'b0;

		if (lw_hazard)
		begin
			dec_stall = 1'b1;
			dec_flush = 1'b1;
		end

		if (ex_stall)
			dec_stall = 1'b1;

		if (thread_switch)
			dec_flush = 1'b1;
	end

	always_comb
	begin : handle_ex
		ex_stall = mem_stall;
		ex_flush = 1'b0;

		if (thread_switch)
			ex_flush = 1'b1;

		if (dc_busy_multithread) begin
			ex_stall = 1'b1;
			ex_flush = 1'b1;
		end
	end

	always_comb
	begin : handle_mem
		mem_stall = 1'b0;
		mem_flush = 1'b0;

		if (thread_switch)
			mem_flush = 1'b1;	

		if (dc_miss_single_thread) begin
			mem_stall = 1'b1;
			mem_flush = 1'b1;
		end
	end

	// Now distribute the control signals to each pipeline registers
	always_comb
	begin
		i2i_hc.flush = 1'b0;
		i2i_hc.stall = if_stall;
		i2d_hc.flush = if_flush;
		i2d_hc.stall = dec_stall;
		d2e_hc.flush = dec_flush;
		d2e_hc.stall = ex_stall;
		e2m_hc.flush = ex_flush;
		e2m_hc.stall = mem_stall;
		m2w_hc.flush = mem_flush;
		m2w_hc.stall = 1'b0;
	end

	// Derive the load_pc
	always_comb
	begin
		load_pc.we = dec_overload;
		if(dec_branch_resolved.is_branch & dec_branch_resolved.outcome == TAKEN)
			load_pc.new_pc = dec_branch_resolved.target;
		else
			load_pc.new_pc = dec_pc.pc + `ADDR_WIDTH'd8;
	end

`ifdef SIMULATION
	always_ff @(posedge clk)
	begin
		if (ic_miss) stats_event("ic_miss");
		// if (ds_miss) stats_event("ds_miss");
		if (dec_overload) stats_event("dec_overload");
		// if (ex_overload) stats_event("ex_overload");
		if (lw_hazard) stats_event("lw_hazard");
		if (dc_miss_single_thread) stats_event("dc_miss_single_thread");
		if (dc_busy_multithread) stats_event("dc_busy_multithread");
		if (thread_switch) stats_event("thread_switch");
		if (if_stall) stats_event("if_stall");
		if (if_flush) stats_event("if_flush");
		if (dec_stall) stats_event("dec_stall");
		if (dec_flush) stats_event("dec_flush");
		if (ex_stall) stats_event("ex_stall");
		if (ex_flush) stats_event("ex_flush");
		if (mem_stall) stats_event("mem_stall");
		if (mem_flush) stats_event("mem_flush");

		if (branch_missed) stats_event("branch_missed");
		if (branch_misidentified) stats_event("branch_misidentified");
		if (branch_target_wrong) stats_event("branch_target_wrong");
		if (branch_mispredicted) stats_event("branch_mispredicted");
		if (branch_correct) stats_event("branch_correct");
	end
`endif

endmodule
