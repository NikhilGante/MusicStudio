/*
*   Displays a pattern, which is read from a small memory, at (x,y) on the VGA output.
*   To set coordinates, first place the desired value of y onto SW[6:0] and press KEY[1].
*   Next, place the desired value of x onto SW[7:0] and then press KEY[2]. The (x,y)
*   coordinates are displayed (in hexadecimal) on (HEX3-2,HEX1-0). Finally, press KEY[3]
*   to draw the pattern at location (x,y).
*/


//module PianoKey (clk, KEY, SW, X, Y, plot, keyState, outX, outY, outClr, HEX0, HEX1, HEX2, HEX3);
module PianoKey(CLOCK_50, KEY, X, Y, keyState, vga_x, vga_y, vga_color);
				
	input CLOCK_50;	
//	input [9:0] SW;
	input [3:0] KEY;
	
	parameter WIDTH_N = 5; // how many bits are required to hold width
	parameter HEIGHT_N = 7; // how many bits are required to hold height
	parameter WIDTH = 5'd15;	// WARNING: make this 1 less than actual width
	parameter HEIGHT = 7'd32;
	
	input [7:0] X;           // starting x location of object
	input [6:0] Y;           // starting y location of object
	input keyState;
	
	wire [WIDTH_N - 1:0] XC;	// used to access object memory
	wire [HEIGHT_N - 1:0] YC;
	
    wire Ex, Ey;
	output [7:0] vga_x;       // x location of each object pixel
	output  [6:0] vga_y;       // y location of each object pixel
	output [2:0] vga_color;   // color of each object pixel
	
    count U3 (CLOCK_50, KEY[0], Ex, XC);    // column counter
        defparam U3.n = WIDTH_N;
		  defparam U3.MAX_COUNT = WIDTH;
    
    regn U5 (1'b1, KEY[0], ~KEY[3], CLOCK_50, Ex);		// enable XC when VGA plotting starts
        defparam U5.n = 1;
    count U4 (CLOCK_50, KEY[0], Ey, YC);    // row counter
        defparam U4.n = HEIGHT_N;
		  defparam U4.MAX_COUNT = HEIGHT;
   
    assign Ey = (XC == WIDTH - 1);		 // enable YC at the end of each object row

    // read a pixel color from object memory
	 wire [2:0] clrOff, clrOn;
	 
	 
    key_off U6 ({YC,XC}, CLOCK_50, clrOff);
	 key_on U9 ({YC,XC}, CLOCK_50, clrOn);
	 
	 reg [2:0] tmp;
	 always @(*) begin
		 if(keyState) tmp = clrOn;
		 else tmp = clrOff;
	 end
	 assign vga_color = tmp;
	 
    // the object memory takes one clock cycle to provide data, so store
    // the current values of (x,y) addresses to remain synchronized
    regn U7 (X + XC, KEY[0], 1'b1, CLOCK_50, vga_x);
        defparam U7.n = 8;
    regn U8 (Y + YC, KEY[0], 1'b1, CLOCK_50, vga_y);
        defparam U8.n = 7;

//    assign plot = ~KEY[3];

endmodule

module vga_demo(CLOCK_50, SW, KEY, HEX3, HEX2, HEX1, HEX0,
				VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, keyNum);
	
	input CLOCK_50;	
	input [9:0] SW;
	input [3:0] KEY;
    output [6:0] HEX3, HEX2, HEX1, HEX0;
	output [7:0] VGA_R;
	output [7:0] VGA_G;
	output [7:0] VGA_B;
	output VGA_HS;
	output VGA_VS;
	output VGA_BLANK_N;
	output VGA_SYNC_N;
	output VGA_CLK;	
	
	input [6:0] keyNum;	// I added this
	
	parameter WIDTH_N = 5; // how many bits are required to hold width
	parameter HEIGHT_N = 8; // how many bits are required to hold height
	parameter WIDTH = 5'd15;	// WARNING: make this 1 less than actual width
	parameter HEIGHT = 7'd32;
	
	wire [7:0] X;           // starting x location of object
	wire [6:0] Y;           // starting y location of object
	wire [WIDTH_N - 1:0] XC;	// used to access object memory
	wire [HEIGHT_N - 1:0] YC;
	
    wire Ex, Ey;
	reg [7:0] VGA_X;       // x location of each object pixel
	reg [6:0] VGA_Y;       // y location of each object pixel
	reg [2:0] VGA_COLOR;   // color of each object pixel
	
	wire [7:0] VGA_X1, VGA_X2, VGA_X3, VGA_X4, VGA_X5, VGA_X6, VGA_X7;
    wire [6:0] VGA_Y1, VGA_Y2, VGA_Y3, VGA_Y4, VGA_Y5, VGA_Y6, VGA_Y7;
    wire [2:0] VGA_COLOR1, VGA_COLOR2, VGA_COLOR3, VGA_COLOR4, VGA_COLOR5, VGA_COLOR6, VGA_COLOR7;

    // Instantiate PianoKey modules

	PianoKey myKey1(CLOCK_50, KEY, 8'd24, 7'd42, keyNum[0], VGA_X1, VGA_Y1, VGA_COLOR1);
	PianoKey myKey2(CLOCK_50, KEY, 8'd40, 7'd42, keyNum[1], VGA_X2, VGA_Y2, VGA_COLOR2);
	PianoKey myKey3(CLOCK_50, KEY, 8'd56, 7'd42, keyNum[2], VGA_X3, VGA_Y3, VGA_COLOR3);
	PianoKey myKey4(CLOCK_50, KEY, 8'd72, 7'd42, keyNum[3], VGA_X4, VGA_Y4, VGA_COLOR4);
	PianoKey myKey5(CLOCK_50, KEY, 8'd88, 7'd42, keyNum[4], VGA_X5, VGA_Y5, VGA_COLOR5);
	PianoKey myKey6(CLOCK_50, KEY, 8'd104, 7'd42, keyNum[5], VGA_X6, VGA_Y6, VGA_COLOR6);
	PianoKey myKey7(CLOCK_50, KEY, 8'd120, 7'd42, keyNum[6], VGA_X7, VGA_Y7, VGA_COLOR7);


	always @(*) begin
		 // Default values in case none of the conditions match
		 VGA_X = 8'd0;
		 VGA_Y = 7'd0;
		 VGA_COLOR = 3'd0;  // Default color (black or off)
		 
		 // Select the key and its corresponding VGA_X, VGA_Y, and color based on toggle
		 case (keyNum)
			  7'b0000001: begin  // If toggle[0] is set
					VGA_X = VGA_X1;
					VGA_Y = VGA_Y1;
					VGA_COLOR = VGA_COLOR1;
			  end
			  7'b0000010: begin  // If toggle[1] is set
					VGA_X = VGA_X2;
					VGA_Y = VGA_Y2;
					VGA_COLOR = VGA_COLOR2;
			  end
			  7'b0000100: begin  // If toggle[2] is set
					VGA_X = VGA_X3;
					VGA_Y = VGA_Y3;
					VGA_COLOR = VGA_COLOR3;
			  end
			  7'b0001000: begin  // If toggle[3] is set
					VGA_X = VGA_X4;
					VGA_Y = VGA_Y4;
					VGA_COLOR = VGA_COLOR4;
			  end
			  7'b0010000: begin  // If toggle[4] is set
					VGA_X = VGA_X5;
					VGA_Y = VGA_Y5;
					VGA_COLOR = VGA_COLOR5;
			  end
			  7'b0100000: begin  // If toggle[5] is set
					VGA_X = VGA_X6;
					VGA_Y = VGA_Y6;
					VGA_COLOR = VGA_COLOR6;
			  end
			  7'b1000000: begin  // If toggle[6] is set
					VGA_X = VGA_X7;
					VGA_Y = VGA_Y7;
					VGA_COLOR = VGA_COLOR7;
			  end
			  default: begin  // If no bits are set or an invalid toggle value
					VGA_X = 8'd0;
					VGA_Y = 7'd0;
					VGA_COLOR = 3'd0;  // Default color (off)
			  end
		 endcase
	end
   

	 	 
    // connect to VGA controller
    vga_adapter VGA (
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(VGA_COLOR),
			.x(VGA_X),
			.y(VGA_Y),
			.plot(1'b1),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK_N(VGA_BLANK_N),
			.VGA_SYNC_N(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

module count (Clock, Resetn, E, Q);
    parameter n = 8;
	 parameter MAX_COUNT = 100;  // Desired max count

    input Clock, Resetn, E;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
		if (Resetn == 0)
			Q <= 0;
		else if (E) begin
			if(Q == MAX_COUNT)    Q <= 0;
			else    Q <= Q + 1;	
		end			
endmodule

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule

