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
	/*
	 // Change the following line to switch predictor
	 branch_predictor_2bit PREDICTOR (
		 .clk, .rst_n,
 
		 .i_req_valid     (request_prediction),
		 .i_req_pc        (dec_pc.pc),
		 .i_req_target    (dec_branch_decoded.target),
		 .o_req_prediction(dec_branch_decoded.prediction),
 
		 .i_fb_valid      (ex_branch_result.valid),
		 .i_fb_pc         (ex_pc.pc),
		 .i_fb_prediction (ex_branch_result.prediction),
		 .i_fb_outcome    (ex_branch_result.outcome)
	 );*/
	 logic correct_branch;
	 assign correct_branch = (ex_branch_result.outcome ==ex_branch_result.prediction);
 
	 g_share#(
		.INDEX_WIDTH(10),
	) PREDICTOR(
		 .clk,
		 .rst_n,
		 .we_bp(dec_branch_decoded.valid),
		 .correct_branch(correct_branch),
		 .fb_pred(ex_branch_result.outcome),
		 .write_pc(ex_pc.pc),
		 .read_pc(dec_pc.pc),
		 .pred(dec_branch_decoded.prediction)
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
 
 `include "mips_core.svh"

 module g_share#(
	 parameter INDEX_WIDTH = 4,
	 parameter ADDR_WIDTH = 26
 )(
	 input  clk,         // Clock
	 input  rst_n,       // Synchronous reset active low
	 input we_bp,        //when decode resoves that instr was a branch
	 input correct_branch, //from decode; if the pranch prediction was correct
	 input fb_pred,      //from decode : prediction that was made
	 input logic[ADDR_WIDTH-1:0] write_pc,//from decode, current_pc
	 input logic[ADDR_WIDTH-1:0] read_pc,
	 output mips_core_pkg::BranchOutcome pred
 
	 //pc_ifc.in i_pc_next
	 
	 //counter signals
 );
 logic global_history [INDEX_WIDTH-1:0];
 
 ghr #(
	 .GHR_DEPTH(INDEX_WIDTH)
 )GHR (
	 .clk,
	 .rst_n,
	 .we_ghr(we_bp),
	 .branch_taken(fb_pred),
	 .global_history_out(global_history)
 );
 
 localparam COUNTER_WIDTH =2;
 
 logic [INDEX_WIDTH-1:0] index; 
 logic [INDEX_WIDTH-1:0] index_write; 
 logic [COUNTER_WIDTH-1:0] current_counter;
 logic counter_dir;
 logic default_encounter;
 
 assign current_counter = counter_regs[index];
 assign index_write = write_pc[INDEX_WIDTH-1:0];
 
 always_comb begin
	 //xor to fold down history and pc to index length
	 /*for (int i=0; i<INDEX_WIDTH;i++)begin
		 index[i] = read_pc[i] ;//^ global_history[i];
	 end*/
	 index = read_pc[INDEX_WIDTH-1:0];
	 //prediction is reader form msb of counter
	  if(counter_regs[index][1]) begin
		 pred = TAKEN;
	  end
	 else begin
		 pred = NOT_TAKEN;
	  end
	 //direction of increment for the counter(based on the branch prediction and the valifity of prediction)
	 //from decode
	//  counter_dir =fb_pred;
 end
 
 logic [COUNTER_WIDTH-1:0] counter_regs[INDEX_WIDTH-1:0];
 
 always_ff@(posedge clk)
 begin
	 if(~rst_n) begin
		 for (int i=0; i<64;i++)begin
			 counter_regs[i] <= 'b01;
		 end
	 end
	  else begin
		 if(we_bp) begin
		 unique case({current_counter,fb_pred})
			 'b000: begin
				 counter_regs[index_write] <= 'b00;
				 default_encounter <= 'b0;
			 end
			 'b001:begin
				 counter_regs[index_write] <= 'b01;
				 default_encounter <= 'b0;
			 end
			 'b010: begin
				 counter_regs[index_write] <= 'b00;
				 default_encounter <= 'b0;
			 end
			 'b011:begin
				 counter_regs[index_write] <= 'b10;
				 default_encounter <= 'b0;
			 end
			 'b100: begin
				 counter_regs[index_write] <= 'b01;
				 default_encounter <= 'b1;
			 end
			 'b101:begin
				 counter_regs[index_write] <= 'b11;
				 default_encounter <= 'b1;
			 end
			 'b110: begin
				 counter_regs[index_write] <= 'b10;
				 default_encounter <= 'b0;
			 end
			 'b111:begin
				 counter_regs[index_write] <= 'b11;
				 default_encounter <= 'b0;
			 end
		 endcase 
	 end
	 else begin
		counter_regs <=  counter_regs;
	 end
	 end
 end
 
 `ifdef SIMULATION
 always_ff @(posedge clk)
 begin
	 if (we_bp) stats_event("we_ghr");
	 if (we_bp & default_encounter) stats_event("default_encounter");
	 //if (counter_dir) stats_event("dir");
 end
`endif

 endmodule
 
 module ghr#(
	 parameter GHR_DEPTH = 4
 )(
	 input  clk,                   // Clock
	 input  rst_n,                 // Synchronous reset active low
	 input logic we_ghr,           //we from ghr will be the same as the we_bp
	 input logic branch_taken,     //if the branch was taken(from decode)
	 output logic global_history_out[GHR_DEPTH-1:0]
 );
 logic global_history_reg [GHR_DEPTH-1:0];
 
 always_ff@(posedge clk)begin
	 if(~rst_n) begin
		 for (int i=0; i<GHR_DEPTH;i++)begin
			 global_history_reg[i] <= 'b0;
		 end
	 end
	 else begin
		 if(we_ghr)begin
			 for (int i=1; i<GHR_DEPTH;i++)begin
				 global_history_reg[i] <= global_history_reg[i-1];
			 end
			 global_history_reg[0] <= branch_taken;
		 end
		 else begin
			 global_history_reg <= global_history_reg;
		 end
	 end
 end
 
 assign global_history_out = global_history_reg;
 
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
 