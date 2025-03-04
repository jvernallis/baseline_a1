/*
 * reg_file.sv
 * Author: Zinsser Zhang
 * Last Revision: 04/09/2018
 *
 * A 32-bit wide, 32-word deep register file with two asynchronous read port
 * and one synchronous write port.
 *
 * Register file needs to output '0 if uses_r* signal is low. In this case,
 * either reg zero is requested for read or the register is unused.
 *
 * See wiki page "Branch and Jump" for details.
 */
`include "mips_core.svh"

interface reg_file_output_ifc ();
	logic [`DATA_WIDTH - 1 : 0] rs_data;
	logic [`DATA_WIDTH - 1 : 0] rt_data;

	modport in  (input rs_data, rt_data);
	modport out (output rs_data, rt_data);
endinterface

module reg_file (
	input clk,    // Clock

	// Input from decoder
	decoder_output_ifc.in i_decoded,

	// Input from write back stage
	write_back_ifc.in i_wb,

	// Output data
	reg_file_output_ifc.out out
);
	/*dummy thread in i_decoded.thread_id*/
	/*Cannot insure consitency when fine swaping threads in one program, so thread 1 is always slected*/
	logic [`DATA_WIDTH - 1 : 0] regs [64];

	assign out.rs_data = i_decoded.uses_rs ? regs[{1'b1,i_decoded.rs_addr}] : '0;
	assign out.rt_data = i_decoded.uses_rt ? regs[{1'b1,i_decoded.rt_addr}] : '0;

	always_ff @(posedge clk) begin
		if(i_wb.uses_rw)
		begin
			regs[{1'b1,i_wb.rw_addr}] = i_wb.rw_data;
		end
	end

endmodule
