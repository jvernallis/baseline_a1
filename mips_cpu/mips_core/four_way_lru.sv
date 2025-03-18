module four_way_lru #(
    parameter ASSOCIATIVITY = 4
)(
    input clk,
    input rst_n,
    input lru_en,
    input logic [$clog2(ASSOCIATIVITY)-1:0] select_way,
    output logic [$clog2(ASSOCIATIVITY)-1:0] lru_rp
);
logic[ASSOCIATIVITY-1:0] way_column[ASSOCIATIVITY];
logic en_last;
always_ff@(posedge clk)begin
    if(~rst_n) begin
        for(int i=0;i<ASSOCIATIVITY;i++)
            way_column[i] <= 'b0;
        for(int i=0;i<ASSOCIATIVITY;i++)
            way_column[i][i] <= 'b1;
    end
    else begin
        en_last <= lru_en;
        if(lru_en &(en_last!=lru_en))begin
           way_column[select_way] <= 'b1111;
           for(int i=0;i<ASSOCIATIVITY;i++)begin
                if(i != select_way)
                    way_column[i][select_way] <= 'b0;
           end
        end
        else begin

        end
    end
end

always_comb begin
    lru_rp = 'b0;
    for(int i=0;i<ASSOCIATIVITY;i++)begin
        if((way_column[i] & (way_column[i] -1)) == 0)
            lru_rp = i;
    end

end
endmodule