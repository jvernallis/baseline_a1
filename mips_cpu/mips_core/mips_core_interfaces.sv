/*
 * mips_core_interfaces.sv
 * Author: Zinsser Zhang
 * Last Revision: 04/09/2018
 *
 * These are interfaces that are not the input or output of one specific unit.
 *
 * See wiki page "Systemverilog Primer" section interfaces for details.
 */
`include "mips_core.svh"

interface load_pc_ifc ();
	logic we;	// Write Enable
	logic [`ADDR_WIDTH - 1 : 0] new_pc;
	logic thread_id;

	modport in  (input we, new_pc,thread_id);
	modport out (output we, new_pc,thread_id);
endinterface

interface pc_ifc ();
	logic [`ADDR_WIDTH - 1 : 0] pc;
	logic thread_id;

	modport in  (input pc,thread_id);
	modport out (output pc,thread_id);
endinterface

interface cache_output_ifc ();
	logic valid;	// Output Valid
	logic [`DATA_WIDTH - 1 : 0] data;
	logic thread_id;

	modport in  (input valid, data,thread_id);
	modport out (output valid, data,thread_id);
endinterface

// Note: Similar to branch_decoded_ifc
// This interface predicts a branch - but now, its validity and target are predicted,
// instead of decoded.
interface branch_prediction_ifc ();
	logic valid;	// High means the instruction is a branch or a jump
	// For simplicity: no jump prediction for now
	// Jumps predicted as branches.
	//logic is_jump;	// High means the instruction is a jump
	logic [`ADDR_WIDTH - 1 : 0] target;

	mips_core_pkg::BranchOutcome prediction;

	logic thread_id;

	modport in (input valid, target, prediction,thread_id);
	modport out (output valid, target, prediction,thread_id);
endinterface

// Note: Replacement for branch_result_ifc.
// This interface is the resolution of a branch - filling in information about its validity,
// target, and decision.
interface branch_resolution_ifc ();
	logic valid; // High means the instruction was decoded to be a branch or a jump
	logic [`ADDR_WIDTH - 1 : 0] target; // The decoded branch target
	mips_core_pkg::BranchOutcome prediction; // The predicted branch decision
	mips_core_pkg::BranchOutcome outcome; // The evaluated branch decision

	modport in  (input valid, target, prediction, outcome);
	modport out  (output valid, target, prediction, outcome);
endinterface

interface alu_pass_through_ifc ();
	// Branch logic moved to decode.
	// logic is_branch;
	// mips_core_pkg::BranchOutcome prediction;
	// logic [`ADDR_WIDTH - 1 : 0] recovery_target;

	logic is_mem_access;
	mips_core_pkg::MemAccessType mem_action;
	logic [`DATA_WIDTH - 1 : 0] sw_data;

	logic uses_rw;
	mips_core_pkg::MipsReg rw_addr;

	logic thread_id;

	modport in  (input is_mem_access, mem_action, sw_data, uses_rw, rw_addr,thread_id);
	modport out (output is_mem_access, mem_action, sw_data, uses_rw, rw_addr,thread_id);
endinterface

// This interface is no longer needed as branch evaluation happens in decode.
// interface branch_result_ifc ();
// 	logic valid;
// 	mips_core_pkg::BranchOutcome prediction;
// 	mips_core_pkg::BranchOutcome outcome;
// 	logic [`ADDR_WIDTH - 1 : 0] recovery_target;

// 	modport in  (input valid, prediction, outcome, recovery_target);
// 	modport out (output valid, prediction, outcome, recovery_target);
// endinterface

interface d_cache_pass_through_ifc ();
	logic is_mem_access;
	logic [`DATA_WIDTH - 1 : 0] alu_result;

	logic uses_rw;
	mips_core_pkg::MipsReg rw_addr;

	logic thread_id;

	modport in  (input is_mem_access, alu_result, uses_rw, rw_addr,thread_id);
	modport out (output is_mem_access, alu_result, uses_rw, rw_addr,thread_id);
endinterface

interface write_back_ifc ();
	logic uses_rw;	// Write Enable
	mips_core_pkg::MipsReg rw_addr;
	logic [`DATA_WIDTH - 1 : 0] rw_data;
	logic thread_id;

	modport in  (input uses_rw, rw_addr, rw_data,thread_id);
	modport out (output uses_rw, rw_addr, rw_data,thread_id);
endinterface

interface hazard_control_ifc ();
	// Stall signal has higher priority
	logic flush;	// Flush signal of the previous stage
	logic stall;	// Stall signal of the next stage
	logic thread_id;

	modport in  (input flush, stall,thread_id);
	modport out (output flush, stall,thread_id);
endinterface
