`include "bram11.v"
module fir 
#(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXI4-Lite Write Address Channel
    output wire                     awready,
    output wire                     wready,
    input  wire                     awvalid,
    input  wire [pADDR_WIDTH-1:0]   awaddr,
    input  wire                     wvalid,
    input  wire [pDATA_WIDTH-1:0]   wdata,
    
    // AXI4-Lite Read Address Channel
    output wire                     arready,
    input  wire                     rready,
    input  wire                     arvalid,
    input  wire [pADDR_WIDTH-1:0]   araddr,
    output wire                     rvalid,
    output wire [pDATA_WIDTH-1:0]   rdata,    
    
    // AXI-Stream Input
    input  wire                     ss_tvalid, 
    input  wire [pDATA_WIDTH-1:0]   ss_tdata, 
    input  wire                     ss_tlast, 
    output wire                     ss_tready, 
    
    // AXI-Stream Output
    input  wire                     sm_tready, 
    output wire                     sm_tvalid, 
    output wire [pDATA_WIDTH-1:0]   sm_tdata, 
    output wire                     sm_tlast, 
    
    // Tap BRAM Interface
    output wire [3:0]               tap_WE,
    output wire                     tap_EN,
    output wire [pDATA_WIDTH-1:0]   tap_Di,
    output wire [pADDR_WIDTH-1:0]   tap_A,
    inout  wire [pDATA_WIDTH-1:0]   tap_Do,

    // Data BRAM Interface
    output wire [3:0]               data_WE,
    output wire                     data_EN,
    output wire [pDATA_WIDTH-1:0]   data_Di,
    output wire [pADDR_WIDTH-1:0]   data_A,
    inout  wire [pDATA_WIDTH-1:0]   data_Do,

    input  wire                     axis_clk,
    input  wire                     axis_rst_n
);

    // AXI4-Lite Control Logic
    reg axi_awready;
    reg axi_wready;
    reg axi_arready;
    reg [pDATA_WIDTH-1:0] axi_rdata_reg;
    reg axi_rvalid;

    // Internal Registers
    reg [pADDR_WIDTH-1:0] write_ptr = 0;
    reg [pADDR_WIDTH-1:0] read_ptr = 0;
    reg [pDATA_WIDTH-1:0] acc = 0;
    reg [4:0] tap_cnt = 0;
    reg calc_done = 0;
    reg [2:0] state = 0;

    // AXI-Stream Control
    assign ss_tready = (write_ptr < (2**pADDR_WIDTH - 1));
    assign sm_tvalid = calc_done;
    assign sm_tlast = calc_done;
    assign sm_tdata = acc;

    // AXI4-Lite Write Channel
    assign awready = axi_awready;
    assign wready = axi_wready;

    always @(posedge axis_clk) begin
        if (!axis_rst_n) begin
            axi_awready <= 0;
            axi_wready <= 0;
        end else begin
            axi_awready <= awvalid && !axi_awready;
            axi_wready <= wvalid && !axi_wready;
        end
    end

    // AXI4-Lite Read Channel
    assign arready = axi_arready;
    assign rvalid = axi_rvalid;
    assign rdata = axi_rdata_reg;

    always @(posedge axis_clk) begin
        if (!axis_rst_n) begin
            axi_arready <= 0;
            axi_rvalid <= 0;
        end else if (arvalid && !axi_arready) begin
            axi_arready <= 1;
            axi_rvalid <= 1;
            if (araddr < Tape_Num)
                axi_rdata_reg <= tap_Do;  // Read from coefficient BRAM
            else
                axi_rdata_reg <= 0;
        end else begin
            axi_arready <= 0;
            if (rready) axi_rvalid <= 0;
        end
    end

    // Data BRAM Write Control
    assign data_WE = (ss_tvalid && ss_tready) ? 4'b1111 : 4'b0000;
    assign data_EN = 1'b1;
    assign data_Di = ss_tdata;
    assign data_A = (state == 1 && tap_cnt < Tape_Num) ? (read_ptr - tap_cnt) : 0;

    // Tap BRAM Write Control
    assign tap_WE = (awvalid && awready) ? 4'b1111 : 4'b0000;
    assign tap_EN = 1'b1;
    assign tap_Di = wdata;
    assign tap_A = (state == 1 && tap_cnt < Tape_Num) ? tap_cnt : 0;

    // Data Write Process
    always @(posedge axis_clk) begin
        if (!axis_rst_n) begin
            write_ptr <= 0;
        end else if (ss_tvalid && ss_tready) begin
            write_ptr <= write_ptr + 1;
        end
    end

    // FIR Calculation FSM
    always @(posedge axis_clk) begin
        if (!axis_rst_n) begin
            state <= 0;
            acc <= 0;
            tap_cnt <= 0;
            calc_done <= 0;
        end else begin
            case(state)
                0: begin  // Idle
                    if (sm_tready) begin
                        read_ptr <= write_ptr - Tape_Num;
                        state <= 1;
                    end
                end
                1: begin  // Multiply-Accumulate
                    if (tap_cnt < Tape_Num) begin
                        acc <= acc + (data_Do * tap_Do);
                        tap_cnt <= tap_cnt + 1;
                    end else begin
                        calc_done <= 1;
                        state <= 2;
                    end
                end
                2: begin  // Wait for output ready
                    if (sm_tready) begin
                        calc_done <= 0;
                        acc <= 0;
                        tap_cnt <= 0;
                        state <= 0;
                    end
                end
            endcase
        end
    end

    // Instantiation of BRAM modules
    bram11 tap_bram (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .Do(tap_Do),
        .A(tap_A)
    );

    bram11 data_bram (
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .Do(data_Do),
        .A(data_A)
    );

endmodule