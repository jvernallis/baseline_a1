
module stream_buffer#(
    parameter BLOCK_OFFSET_WIDTH = 2,
    parameter LINE_SIZE = 4,
    parameter BUFF_DATA_WIDTH = `ADDR_WIDTH,
    parameter MEMID = 0,
    parameter BUFFER_LEN = 2

    
)(
    input logic clk,
    input rst_n,
    input logic [`ADDR_WIDTH-3:0] current_addr,  //cache access address
    input logic cache_miss,
    input logic miss_valid,
    output logic hit_out,                       //if there is a hit in the stream buffer    
    output logic [`DATA_WIDTH-1:0] sb_rdata[LINE_SIZE],

    axi_read_address.master mem_read_address[BUFFER_LEN],
     axi_read_data.master mem_read_data[BUFFER_LEN]
);
    // States
    enum logic[1:0] {
        STATE_READY,            // Ready for incoming requests
        STATE_REFILL_REQUEST,       // Missing on a read
        FETCH_CHECK
    } state, next_state;

// available bits
logic [BUFFER_LEN - 1 : 0] available_bits;
// Intermediate signals
logic hit, miss;
//buffer pointers for FIFO managmnet
logic[`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] buff_tail,buff_head;
//logic that indicates that the stream buffer and cache both missed
logic all_miss;
//regeter to hold the value of the address to write from
logic [`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0]current_addr_reg; 

logic [BUFFER_LEN - 1 : 0] databank_raddr;
logic [`DATA_WIDTH - 1 : 0] databank_rdata [BUFFER_LEN][LINE_SIZE];
logic [BUFFER_LEN-1:0] databank_available;
logic [BUFFER_LEN-1:0] ar_valid;

genvar g;
    generate
        for (g = 0; g < BUFFER_LEN; g++)
        begin : sb_cells
            sb_cell #(
                .BUFFER_LEN(BUFFER_LEN),
                .ID(g)
            ) databank (
                .clk,
                .rst_n,
                .cell_addr(current_addr_reg + buff_tail),
                .cell_enable((state != STATE_READY)&&(buff_tail[$clog2(BUFFER_LEN)-1:0] == g)),
                .cell_rdata(databank_rdata[g]),
                .available(databank_available[g]),
                .ar_valid(ar_valid[g]),
                .mem_read_address(mem_read_address[g]),
                .mem_read_data(mem_read_data[g])
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
end


    always_comb
    begin
        //databank_waddr = buff_tail[$clog2(BUFFER_LEN)-1:0];
        databank_raddr = buff_head[$clog2(BUFFER_LEN)-1:0];
    end

    logic [`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] buffer_tag;
    always_comb
    begin
       //offset from the tail value for the tag
        buffer_tag = current_addr_reg + (buff_tail);

        tagbank_we = ar_valid[buff_tail[$clog2(BUFFER_LEN)-1:0]];
        tagbank_wdata = {buffer_tag,2'b0};
        tagbank_waddr = buff_tail[$clog2(BUFFER_LEN)-1:0];
        tagbank_raddr = buff_head[$clog2(BUFFER_LEN)-1:0];
    end

//output logic
always_comb
    begin
        hit_out = hit;
        sb_rdata = databank_rdata[databank_raddr];
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
                next_state =  (all_miss) ? STATE_REFILL_REQUEST : STATE_READY; 
            STATE_REFILL_REQUEST: begin
                if(ar_valid[buff_tail[$clog2(BUFFER_LEN)-1:0]])
                    next_state = FETCH_CHECK;
                end
            FETCH_CHECK:begin
                //keep requesting from mem if stream buffer is not full
                if(~tail_at_end) begin
                    next_state = STATE_REFILL_REQUEST; 
                end
                //if the head has valid data return to ready
                else if(available_bits[buff_head[$clog2(BUFFER_LEN)-1:0]]) begin
                    next_state = STATE_READY;
                end
            end
        endcase
    end
    logic cache_miss_reg;
    always_ff @(posedge clk)
    begin
        if(~rst_n)
        begin
            state <= STATE_READY;
            available_bits <= '0;
            buff_head <= 'b0;
            buff_tail <= 'b0;
            cache_miss_reg <= 'b0;
        end
        else
        begin
            state <= next_state;
            cache_miss_reg <= cache_miss;

            if(databank_available[buff_tail[$clog2(BUFFER_LEN)-1:0]])begin
                available_bits[buff_tail[$clog2(BUFFER_LEN)-1:0]] <= 'b1;
            end
               
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
                    if ((ar_valid[buff_tail[$clog2(BUFFER_LEN)-1:0]]) & (~tail_at_end))
                    begin
                        buff_tail <= buff_tail+'b1;
                    end
                end
            endcase
        end
    end
endmodule