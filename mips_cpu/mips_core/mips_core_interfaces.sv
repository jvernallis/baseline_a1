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

	modport in  (input we, new_pc);
	modport out (output we, new_pc);
endinterface

interface pc_ifc ();
	logic [`ADDR_WIDTH - 1 : 0] pc;

	modport in  (input pc);
	modport out (output pc);
endinterface

interface cache_output_ifc ();
	logic valid;	// Output Valid
	logic [`DATA_WIDTH - 1 : 0] data;

	modport in  (input valid, data);
	modport out (output valid, data);
endinterface

// Note: Similar to branch_decoded_ifc
// This interface predicts a branch - but now, its validity and target are predicted,
// instead of decoded.
interface branch_prediction_ifc ();
	logic is_branch; // High means the instruction is a branch or a jump
	// For simplicity: no jump prediction for now
	// Jumps predicted as branches.
	//logic is_jump;	// High means the instruction is a jump
	logic [`ADDR_WIDTH - 1 : 0] target;

	mips_core_pkg::BranchOutcome prediction;

	modport in (input is_branch, target, prediction);
	modport out (output is_branch, target, prediction);
endinterface

// Note: Replacement for branch_result_ifc.
// This interface is the resolution of a branch - filling in information about its validity,
// target, and decision.
interface branch_resolution_ifc ();
	logic is_branch; // High means the instruction was decoded to be a branch or a jump
	logic [`ADDR_WIDTH - 1 : 0] target; // The decoded branch target
	mips_core_pkg::BranchOutcome prediction; // The predicted branch decision
	mips_core_pkg::BranchOutcome outcome; // The evaluated branch decision

	modport in  (input is_branch, target, prediction, outcome);
	modport out  (output is_branch, target, prediction, outcome);
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

	modport in  (input is_mem_access, mem_action, sw_data, uses_rw, rw_addr);
	modport out (output is_mem_access, mem_action, sw_data, uses_rw, rw_addr);
endinterface

interface d_cache_pass_through_ifc ();
	logic is_mem_access;
	logic [`DATA_WIDTH - 1 : 0] alu_result;

	logic uses_rw;
	mips_core_pkg::MipsReg rw_addr;

	modport in  (input is_mem_access, alu_result, uses_rw, rw_addr);
	modport out (output is_mem_access, alu_result, uses_rw, rw_addr);
endinterface

interface write_back_ifc ();
	logic uses_rw;	// Write Enable
	mips_core_pkg::MipsReg rw_addr;
	logic [`DATA_WIDTH - 1 : 0] rw_data;

	modport in  (input uses_rw, rw_addr, rw_data);
	modport out (output uses_rw, rw_addr, rw_data);
endinterface

interface hazard_control_ifc ();
	// Stall signal has higher priority
	logic flush;	// Flush signal of the previous stage
	logic stall;	// Stall signal of the next stage

	modport in  (input flush, stall);
	modport out (output flush, stall);
endinterface

interface thread_control_ifc ();
	logic thread_id; 			// ID of the active thread
	logic thread_switch; 		// True on the cycle a thread switches
	logic thread_switch_available; 	// True whenever a thread switch is available 
	logic current_thread_done; // Done status of the active thread

	//logic temp_thread_switch; //Keeping this around for when I'm doing testing in both mips_core and the hazard controller

	logic thread_done[2]; // Threads complete
	logic thread_ready[2]; // Threads can be switched to
	logic [`ADDR_WIDTH - 1 : 0] thread_resume_pc[2]; // PC to resume threads from

	modport in (input thread_id, thread_switch, thread_switch_available, current_thread_done, thread_done, thread_ready, thread_resume_pc);
	modport out (output thread_id, thread_switch, thread_switch_available, current_thread_done, thread_done, thread_ready, thread_resume_pc);

	// modport thread_control (output thread_id, )
endinterface