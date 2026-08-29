`timescale 1ns / 1ps

module tb_sobel;

    // Parameters - match the Python script
    parameter WIDTH = 735;
    parameter HEIGHT = 511;

    reg clk;
    reg reset;
    reg start;
    wire done;

    // Instantiate the module
    sobel_edge_detector #(
        .IMG_WIDTH(WIDTH),
        .IMG_HEIGHT(HEIGHT),
        .THRESHOLD(30)
    ) uut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .done(done)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;
        start = 0;

        #20;
        reset = 0;
        
        #10;
        start = 1;
        #10;
        start = 0;

        wait(done == 1'b1);
        
        #20;
        $display("Edge detection complete. Output written to edge_output.bin");
        $finish;
    end

endmodule