module sb_group#(
    parameter BLOCK_OFFSET_WIDTH = 2,
    parameter LINE_SIZE = 4,
    parameter BUFF_DATA_WIDTH = `ADDR_WIDTH,
    parameter MEMID = 0,
    parameter BUFFER_LEN = 4

    
)(
    input logic clk,
    input rst_n,
    input logic [`ADDR_WIDTH-3:0] current_addr,  //cache access address
    input logic cache_miss,
    input logic miss_valid,
    output logic hit_out,                       //if there is a hit in the stream buffer    
    output logic [`DATA_WIDTH-1:0] sb_rdata[LINE_SIZE],

    axi_read_address.master mem_read_address,
     axi_read_data.master mem_read_data
);
stream_buffer#(
		  .BLOCK_OFFSET_WIDTH,
		  .LINE_SIZE,
		  .BUFF_DATA_WIDTH,
		  .MEMID
	  )SB_D(
		  .clk,
		  .rst_n,
		  .current_addr,
		  .cache_miss,
		  .miss_valid,
		  .hit_out,
		  .sb_rdata,
		  .mem_read_address,
		  .mem_read_data
	  );
endmodule