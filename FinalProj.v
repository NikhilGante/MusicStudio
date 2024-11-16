
module FinalProj (

// KEYBOARd
// Bidirectionals
	PS2_CLK,
	PS2_DAT,
	
	// Outputs
	LEDR, // I added
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,
	HEX6,
	HEX7,

	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	SW
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
 //KEYBOARD
 // Inputs
//input				CLOCK_50;
//input		[3:0]	KEY;

// Bidirectionals
inout				PS2_CLK;
inout				PS2_DAT;

// Outputs
output		[9:0] LEDR;	// I added
output		[6:0]	HEX0;
output		[6:0]	HEX1;
output		[6:0]	HEX2;
output		[6:0]	HEX3;
output		[6:0]	HEX4;
output		[6:0]	HEX5;
output		[6:0]	HEX6;
output		[6:0]	HEX7;
 
 
 
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
input		[3:0]	SW;

input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
 // KEYBOARD
 // Internal Wires
wire		[7:0]	ps2_key_data;
wire				ps2_key_pressed;

// Internal Registers
reg			[7:0]	last_data_received;

reg [6:0] toggle; // I added

 
 
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;


// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/
 
 //KEYBOARD
 // I added
parameter A = 8'h16;
parameter B = 8'h1E;
parameter C = 8'h26;
parameter D = 8'h25;
parameter E = 8'h2E;
parameter F = 8'h36;
parameter G = 8'h3D;
parameter Ao = 8'hF016;
parameter Bo = 8'hF01E;
parameter Co = 8'hF026;
parameter Do = 8'hF025;
parameter Eo = 8'hF02E;
parameter Fo = 8'hF036;
parameter Go = 8'hF03D;
parameter upArrow = 8'hE075;
parameter downArrow = 8'hE072;
parameter leftArrow = 8'hE06B;
parameter rightArrow = 8'hE074;

//base freq is the 4th octave - 5th octave, A - G
parameter baseFreqA = 19'd440;
parameter baseFreqB = 19'd494;
parameter baseFreqC = 19'd523;
parameter baseFreqD = 19'd587;
parameter baseFreqE = 19'd659;
parameter baseFreqF = 19'd698;
parameter baseFreqG = 19'd784;

// Registers for each note's state, delay, and volume
reg [18:0] delay_cnt_A, delay_cnt_B, delay_cnt_C, delay_cnt_D, delay_cnt_E, delay_cnt_F, delay_cnt_G;
reg [18:0] delay_A, delay_B, delay_C, delay_D, delay_E, delay_F, delay_G;
reg snd_A, snd_B, snd_C, snd_D, snd_E, snd_F, snd_G;
reg [31:0] volume_A, volume_B, volume_C, volume_D, volume_E, volume_F, volume_G;

// n should range from -4 <= n <= 4
// maybe -5 <= n <= 5 idk
reg signed [3:0] n;  // 4-bit signed register for n
reg [31:0] scaledFreq;  // Scaling factor
reg flat;
reg sharp;

reg lastWasBreak;
reg def;

// Initialize delays and other signals
initial begin
	lastWasBreak = 0;
	def = 0;
	n = 0;
	flat = 0;
	sharp = 0;
	snd_A = 0; snd_B = 0; snd_C = 0; snd_D = 0; snd_E = 0; snd_F = 0; snd_G = 0;
	delay_A = baseFreqA; delay_B = baseFreqB; delay_C = baseFreqC;
	delay_D = baseFreqD; delay_E = baseFreqE; delay_F = baseFreqF; delay_G = baseFreqG;
end

//sharp is right, flat is left

//this is what im editing from during nov 10 (IT WORKS)
always @(posedge CLOCK_50) begin
    // SPEAKER: Handle delay logic
    
    // Note A
    if (delay_cnt_A == delay_A) begin
        delay_cnt_A <= 0;
        snd_A <= !snd_A;
    end else begin
        delay_cnt_A <= delay_cnt_A + 1;
    end

    // Note B
    if (delay_cnt_B == delay_B) begin
        delay_cnt_B <= 0;
        snd_B <= !snd_B;
    end else begin
        delay_cnt_B <= delay_cnt_B + 1;
    end

    // Note C
    if (delay_cnt_C == delay_C) begin
        delay_cnt_C <= 0;
        snd_C <= !snd_C;
    end else begin
        delay_cnt_C <= delay_cnt_C + 1;
    end

    // Note D
    if (delay_cnt_D == delay_D) begin
        delay_cnt_D <= 0;
        snd_D <= !snd_D;
    end else begin
        delay_cnt_D <= delay_cnt_D + 1;
    end

    // Note E
    if (delay_cnt_E == delay_E) begin
        delay_cnt_E <= 0;
        snd_E <= !snd_E;
    end else begin
        delay_cnt_E <= delay_cnt_E + 1;
    end

    // Note F
    if (delay_cnt_F == delay_F) begin
        delay_cnt_F <= 0;
        snd_F <= !snd_F;
    end else begin
        delay_cnt_F <= delay_cnt_F + 1;
    end

    // Note G
    if (delay_cnt_G == delay_G) begin
        delay_cnt_G <= 0;
        snd_G <= !snd_G;
    end else begin
        delay_cnt_G <= delay_cnt_G + 1;
    end

    // KEYBOARD: Handle key presses
    if (KEY[0] == 1'b0) begin
        last_data_received <= 8'h00;  // Reset on KEY[0] press
    end else if (ps2_key_pressed) begin
        last_data_received <= ps2_key_data;  // Store the key data
			
			if(lastWasBreak) begin
				 case(last_data_received)
						A: toggle[6] <= 1'b0;
						B: toggle[5] <= 1'b0;
						C: toggle[4] <= 1'b0;
						D: toggle[3] <= 1'b0; 
						E: toggle[2] <= 1'b0; 
						F: toggle[1] <= 1'b0; 
						G: toggle[0] <= 1'b0;		
				endcase
				lastWasBreak <= 1'b0;
			end else begin
			  // Toggle LED for specific keys (A, B, C, D, E, F, G)
			  case(last_data_received)
					A: toggle[6] <= 1'b1;
					B: toggle[5] <= 1'b1;
					C: toggle[4] <= 1'b1;
					D: toggle[3] <= 1'b1; 
					E: toggle[2] <= 1'b1; 
					F: toggle[1] <= 1'b1; 
					G: toggle[0] <= 1'b1;
//					Ao: toggle[0] <= 1'b0;
//					Bo: toggle[1] <= 1'b0;
//					Co: toggle[2] <= 1'b0;
//					Do: toggle[3] <= 1'b0; 
//					Eo: toggle[4] <= 1'b0; 
//					Fo: toggle[5] <= 1'b0; 
//					Go: toggle[6] <= 1'b0;
					// Set LED toggle if keys A-G are pressed
					upArrow: n <= n + 1;
					downArrow: n <= n - 1;
					leftArrow: flat <= 1'b1;
					rightArrow: sharp <= 1'b1;
					8'hF0: lastWasBreak <= 1'b1;
					default: begin
					toggle <= 7'b0000000;  // Reset LED for other keys
					flat <= 1'b0;
					sharp <= 1'b0;
					end
				endcase
			end
    end
	 
	 volume_A <= toggle[6]? (snd_A ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_B <= toggle[5]? (snd_B ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_C <= toggle[4]? (snd_C ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_D <= toggle[3]? (snd_D ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_E <= toggle[2]? (snd_E ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_F <= toggle[1]? (snd_F ? 32'd10000000 : -32'd10000000) : 32'b0;
	 volume_G <= toggle[0]? (snd_G ? 32'd10000000 : -32'd10000000) : 32'b0;
	 
	 def <= 1'b0;
	 if(toggle != 7'b0) begin
	 
		 if (n > 0) begin
			  scaledFreq <= 50000000 / (1 << n);  
		 end else if (n < 0) begin
			  scaledFreq <= 50000000 * (1 << -n); 
		 end else begin
			  scaledFreq <= 50000000;            
		 end
//		 
//		 if (flat) scaledFreq <= scaledFreq / 1.0595;
//		 if (sharp) scaledFreq <= scaledFreq * 1.0595;
//		 
//		 
		 case (last_data_received)
					  A: delay_A <= scaledFreq / baseFreqA;
					  B: delay_B <= scaledFreq / baseFreqB;
					  C: delay_C <= scaledFreq / baseFreqC;
					  D: delay_D <= scaledFreq / baseFreqD;
					  E: delay_E <= scaledFreq / baseFreqE;
					  F: delay_F <= scaledFreq / baseFreqF;
					  G: delay_G <= scaledFreq / baseFreqG;
					  default: begin
							def <= 1'b1;
							//delay_A <= baseFreqA; delay_B <= baseFreqB; delay_C <= baseFreqC;
							//delay_D <= baseFreqD; delay_E <= baseFreqE; delay_F <= baseFreqF; delay_G <= baseFreqG;
							
							
					  end
		 endcase
	 
	 end else begin
		volume_A <= 32'd0;
		volume_B <= 32'd0;
		volume_C <= 32'd0;
		volume_D <= 32'd0;
		volume_E <= 32'd0;
		volume_F <= 32'd0;
		volume_G <= 32'd0;
		
	 
	 end
	 
end


/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

 //KEYBOARD
 assign LEDR[6:0] = toggle;
 assign LEDR[9] =lastWasBreak;
 assign LEDR[8] = def;
assign HEX2 = 7'h7F;
assign HEX3 = 7'h7F;
assign HEX4 = 7'h7F;
//assign HEX5 = 7'h7F;
//assign HEX6 = 7'h7F;
//assign HEX7 = 7'h7F;
 
// Combine volumes for simultaneous playback
assign read_audio_in			= audio_in_available & audio_out_allowed;
assign left_channel_audio_out = left_channel_audio_in +
                                (toggle[6] ? volume_A : 0) +
                                (toggle[5] ? volume_B : 0) +
                                (toggle[4] ? volume_C : 0) +
                                (toggle[3] ? volume_D : 0) +
                                (toggle[2] ? volume_E : 0) +
                                (toggle[1] ? volume_F : 0) +
                                (toggle[0] ? volume_G : 0);

assign right_channel_audio_out = left_channel_audio_out;  // Mirror left channel
assign write_audio_out			= audio_in_available & audio_out_allowed;


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/
//KEYBOARD
 PS2_Controller PS2 (
	// Inputs
	.CLOCK_50				(CLOCK_50),
	.reset				(~KEY[0]),

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
	.received_data		(ps2_key_data),
	.received_data_en	(ps2_key_pressed)
);

Hexadecimal_To_Seven_Segment Segment0 (
	// Inputs
	.hex_number			(last_data_received[3:0]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX0)
);

Hexadecimal_To_Seven_Segment Segment1 (
	// Inputs
	.hex_number			(last_data_received[7:4]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX1)
);

Hexadecimal_To_Seven_Segment Segmentn (
	// Inputs
	.hex_number			(n),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX5)
);

//SPEAKERS
 
 
Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);

endmodule
