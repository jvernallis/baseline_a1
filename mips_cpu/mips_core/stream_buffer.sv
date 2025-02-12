module stream_buffer#(
    parameter BLOCK_OFFSET_WIDTH = 2,
    parameter LINE_SIZE = 4,
    parameter BUFF_DATA_WIDTH = `ADDR_WIDTH,
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
    // States
    enum logic[1:0] {
        STATE_READY,            // Ready for incoming requests
        STATE_REFILL_REQUEST,   // Sending out a memory read request
        STATE_REFILL_DATA       // Missing on a read
    } state, next_state;

// available bits
logic [BUFFER_LEN - 1 : 0] available_bits;
// Intermediate signals
logic hit, miss;
logic last_refill_word;
//buffer pointers for FIFO managmnet
logic[`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] buff_tail,buff_head;
//logic that indicates that the stream buffer and cache both missed
logic all_miss;
//regeter to hold the value of the address to write from
logic [`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0]current_addr_reg; 

logic [LINE_SIZE - 1 : 0] databank_select;
logic [LINE_SIZE - 1 : 0] databank_we;
logic [`DATA_WIDTH - 1 : 0] databank_wdata;
logic [BUFFER_LEN - 1 : 0] databank_waddr;
logic [BUFFER_LEN - 1 : 0] databank_raddr;
logic [`DATA_WIDTH - 1 : 0] databank_rdata [LINE_SIZE];

genvar g;
    generate
        for (g = 0; g < LINE_SIZE; g++)
        begin : databanks
            cache_bank #(
                .DATA_WIDTH (`DATA_WIDTH),
                .ADDR_WIDTH ($clog2(BUFFER_LEN))
            ) databank (
                .clk,
                .i_we (databank_we[g]),
                .i_wdata(databank_wdata),
                .i_waddr(databank_waddr),
                .i_raddr(databank_raddr),

                .o_rdata(databank_rdata[g])
            );
        end
    endgenerate

// tagbank signals
logic tagbank_we;
logic [`ADDR_WIDTH - 3 : 0] tagbank_wdata;
logic [BUFFER_LEN - 1 : 0] tagbank_waddr;
logic [BUFFER_LEN - 1 : 0] tagbank_raddr;
logic [`ADDR_WIDTH - 3 : 0] tagbank_rdata;

cache_bank #(
    .DATA_WIDTH (`DATA_WIDTH),
    .ADDR_WIDTH ($clog2(BUFFER_LEN))
) tagbank (
    .clk,
    .i_we    (tagbank_we),
    .i_wdata (tagbank_wdata),
    .i_waddr (tagbank_waddr),
    .i_raddr (tagbank_raddr),

    .o_rdata (tagbank_rdata)
);

always_comb
begin
    hit = available_bits[buff_head[$clog2(BUFFER_LEN)-1:0]]
        & (current_addr[`ADDR_WIDTH-3:2] == tagbank_rdata[`ADDR_WIDTH-3:2]);

    miss = ~hit;
    all_miss = miss & cache_miss_reg & miss_valid;
    last_refill_word = databank_select[LINE_SIZE - 1]
        & mem_read_data.RVALID;
end

logic[`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] mem_addr;
always_comb
    begin
        mem_addr = (current_addr_reg + (buff_tail));
        mem_read_address.ARADDR = {mem_addr,
            {BLOCK_OFFSET_WIDTH + 2{1'b0}}};
        mem_read_address.ARLEN = LINE_SIZE;
        mem_read_address.ARVALID = state == STATE_REFILL_REQUEST;
        mem_read_address.ARID = 4'd0;

        // Always ready to consume data
        mem_read_data.RREADY = 1'b1;
    end

    always_comb
    begin
        if (mem_read_data.RVALID)
            databank_we = databank_select;
        else
            databank_we = '0;

        databank_wdata = mem_read_data.RDATA;
        databank_waddr = buff_tail[$clog2(BUFFER_LEN)-1:0];
        databank_raddr = buff_head[$clog2(BUFFER_LEN)-1:0];
    end

    logic [`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] buffer_tag;
    always_comb
    begin
       //offset from the head value for the tag
        buffer_tag = current_addr_reg + (buff_head);

        tagbank_we = last_refill_word;
        tagbank_wdata = {buffer_tag,2'b0};
        tagbank_waddr = buff_tail[$clog2(BUFFER_LEN)-1:0];
        tagbank_raddr = buff_head[$clog2(BUFFER_LEN)-1:0];
    end

//output logic
always_comb
    begin
        hit_out = hit;
        sb_rdata = databank_rdata;
    end
//state logic
    logic tail_at_end;
always_comb
    begin
        next_state = state;
        tail_at_end = ((buff_tail - buff_head) == BUFFER_LEN);
        unique case (state)
            //only come out of READY when both cache and stream buffer miss
            STATE_READY:
                next_state =  (all_miss | ~tail_at_end) ? STATE_REFILL_REQUEST : STATE_READY; 
            STATE_REFILL_REQUEST:
               if (mem_read_address.ARREADY)
                    next_state = STATE_REFILL_DATA;
            STATE_REFILL_DATA: begin
                if(last_refill_word)
                    next_state = STATE_READY;
                end
        endcase
    end
    logic hit_prev,cache_miss_reg;
    always_ff @(posedge clk)
    begin
        if(~rst_n)
        begin
            state <= STATE_READY;
            databank_select <= 1;
            available_bits <= '0;
            buff_head <= 'b0;
            buff_tail <= 'b0;
            cache_miss_reg <= 'b0;
        end
        else
        begin
            state <= next_state;
            hit_prev <= hit;
            cache_miss_reg <= cache_miss;
            
               
            case (state)
                STATE_READY:
                begin
                   buff_head <= buff_head + hit;
                   if (all_miss)
                       begin
                           current_addr_reg <= {current_addr[`ADDR_WIDTH - 3 : BLOCK_OFFSET_WIDTH]};
                           available_bits <= '0;
                           buff_head <= 'b0;
                           buff_tail <= 'b0;
                       end
                       else 
                       begin
                           current_addr_reg <= current_addr_reg;
                       end
                end
                STATE_REFILL_REQUEST:
                begin
                end
                STATE_REFILL_DATA:
                begin
                    if (mem_read_data.RVALID)
                    begin
                        databank_select <= {databank_select[LINE_SIZE - 2 : 0],
                            databank_select[LINE_SIZE - 1]};
                            available_bits[buff_tail[$clog2(BUFFER_LEN)-1:0]] <= last_refill_word;
                            buff_tail <= buff_tail + last_refill_word;
                    end
                end
            endcase
        end
    end
endmodule