/*
 * branch_controller.sv
 * Author: Zinsser Zhang
 * Last Revision: 04/08/2018
 *
 * branch_controller is a bridge between branch predictor to hazard controller.
 * Two simple predictors are also provided as examples.
 *
 * See wiki page "Branch and Jump" for details.
 */
`include "mips_core.svh"

module branch_controller (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	pc_ifc.in dec_pc,
	branch_decoded_ifc.hazard dec_branch_decoded,

	// Feedback
	pc_ifc.in ex_pc,
	branch_result_ifc.in ex_branch_result
);
	logic request_prediction;

	// Change the following line to switch predictor
	branch_predictor_gshare PREDICTOR (
		.clk, .rst_n,

		.i_req_valid     (request_prediction),
		.i_req_pc        (dec_pc.pc),
		.i_req_target    (dec_branch_decoded.target),
		.o_req_prediction(dec_branch_decoded.prediction),

		.i_fb_valid      (ex_branch_result.valid),
		.i_fb_pc         (ex_pc.pc),
		.i_fb_prediction (ex_branch_result.prediction),
		.i_fb_outcome    (ex_branch_result.outcome)
	);

	always_comb
	begin
		request_prediction = dec_branch_decoded.valid & ~dec_branch_decoded.is_jump;
		dec_branch_decoded.recovery_target =
			(dec_branch_decoded.prediction == TAKEN)
			? dec_pc.pc + `ADDR_WIDTH'd8
			: dec_branch_decoded.target;
	end

endmodule

module branch_predictor_always_not_taken (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);

	always_comb
	begin
		o_req_prediction = NOT_TAKEN;
	end

endmodule

module branch_predictor_2bit (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);

	logic [1:0] counter;

	task incr;
		begin
			if (counter != 2'b11)
				counter <= counter + 2'b01;
		end
	endtask

	task decr;
		begin
			if (counter != 2'b00)
				counter <= counter - 2'b01;
		end
	endtask

	always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			counter <= 2'b01;	// Weakly not taken
		end
		else
		begin
			if (i_fb_valid)
			begin
				case (i_fb_outcome)
					NOT_TAKEN: decr();
					TAKEN:     incr();
				endcase
			end
		end
	end

	always_comb
	begin
		o_req_prediction = counter[1] ? TAKEN : NOT_TAKEN;
	end

endmodule

module branch_predictor_gshare (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);
localparam COUNTER_WIDTH = 2;
localparam INDEX_WIDTH = 16;

logic [INDEX_WIDTH-1:0] index_read; 
logic [INDEX_WIDTH-1:0] index_write; 
logic [INDEX_WIDTH-1:0] ghr;
// logic [INDEX_WIDTH-1:0] old_ghr;
logic [COUNTER_WIDTH-1:0] current_counter;

// logic [1:0] counter;

logic [COUNTER_WIDTH-1:0] counter_regs[2**INDEX_WIDTH];

	task incr;
		begin
			if (counter_regs[index_write] != 2'b11)
				counter_regs[index_write] <= counter_regs[index_write] + 2'b01;
		end
	endtask

	task decr;
		begin
			if (counter_regs[index_write] != 2'b00)
				counter_regs[index_write] <= counter_regs[index_write] - 2'b01;
		end
	endtask


	always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			for (int i=0; i<2**INDEX_WIDTH; i++) 
			begin
			counter_regs[i] <= 2'b10; //Default: weakly taken
			end
		end
		else
		begin
		index_write <= index_read;

		if (i_fb_valid)
		begin

				case (i_fb_outcome)
					NOT_TAKEN: decr();
					TAKEN:     incr();
				endcase

				for (int i=1; i<INDEX_WIDTH; i++)
				begin
					ghr[i] <= ghr[i-1];
				end
			ghr[0] <= i_fb_outcome;
		end
		else begin
			ghr <= ghr;
		end
		end
	end

	always_comb
	begin

		index_read = i_req_pc[INDEX_WIDTH-1:0] ^ ghr;
		o_req_prediction = counter_regs[index_read][1] ? TAKEN : NOT_TAKEN;
	end
endmodule