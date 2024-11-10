
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

reg ledToggle; // I added
 
 
 
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

reg [18:0] delay_cnt;
reg [18:0] delay;
reg [31:0] volume;

reg snd;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/
 
 //KEYBOARD
 // I added
parameter A = 8'h1C;
parameter B = 8'h32;
parameter C = 8'h21;
parameter D = 8'h23;
parameter E = 8'h24;
parameter F = 8'h2B;
parameter G = 8'h34;
parameter upArrow = 8'hE075;
parameter downArrow = 8'hE072;

/* stuff before nov 10 (works)
always @(posedge CLOCK_50) begin
    // SPEAKER: Handle delay logic
    if (delay_cnt == delay) begin
        delay_cnt <= 0;
        snd <= !snd;  // Toggle sound
    end else begin
        delay_cnt <= delay_cnt + 1;
    end

    // KEYBOARD: Handle key presses
    if (KEY[0] == 1'b0) begin
        last_data_received <= 8'h00;  // Reset on KEY[0] press
    end else if (ps2_key_pressed == 1'b1) begin
        last_data_received <= ps2_key_data;  // Store the key data

        // Toggle LED for specific keys (A, B, C, D, E, F, G)
        case(last_data_received)
            A, B, C, D, E, F, G: ledToggle <= 1'b1;  // Set LED toggle if keys A-G are pressed
            default: ledToggle <= 1'b0;  // Reset LED for other keys
        endcase
    end

    // Handle delay and volume value based on ledToggle
    if (ledToggle) begin
        volume <= snd ? 32'd10000000 : -32'd10000000;  // Set volume based on sound toggle
        case(last_data_received)
            A: delay <= 19'd113636;
            B: delay <= 19'd101214;
            C: delay <= 19'd95555;
				D: delay <= 19'd85132;
				E: delay <= 19'd75842;
				F: delay <= 19'd71586;
				G: delay <= 19'd63776;				
            default: delay <= 19'd1000;  // Default delay if no specific key
        endcase
    end else begin
        delay <= 19'd10000;  // Default delay if ledToggle is not active
        volume <= 32'd0;      // Reset volume if ledToggle is not active
    end
end

*/
//base freq is the 4th octave - 5th octave, A - G
parameter baseFreqA = 19'd440;
parameter baseFreqB = 19'd494;
parameter baseFreqC = 19'd523;
parameter baseFreqD = 19'd587;
parameter baseFreqE = 19'd659;
parameter baseFreqF = 19'd698;
parameter baseFreqG = 19'd784;

// n should range from -4 <= n <= 4
// maybe -5 <= n <= 5 idk
reg signed [3:0] n;  // 4-bit signed register for n
reg [31:0] scaledFreq;  // Scaling factor

initial begin
    n = 0;
end

//this is what im editing from during nov 10 (IT WORKS)
always @(posedge CLOCK_50) begin
    // SPEAKER: Handle delay logic
    if (delay_cnt == delay) begin
        delay_cnt <= 0;
        snd <= !snd;  // Toggle sound
    end else begin
        delay_cnt <= delay_cnt + 1;
    end

    // KEYBOARD: Handle key presses
    if (KEY[0] == 1'b0) begin
        last_data_received <= 8'h00;  // Reset on KEY[0] press
    end else if (ps2_key_pressed == 1'b1) begin
        last_data_received <= ps2_key_data;  // Store the key data

        // Toggle LED for specific keys (A, B, C, D, E, F, G)
        case(last_data_received)
            A, B, C, D, E, F, G: ledToggle <= 1'b1;  // Set LED toggle if keys A-G are pressed
				upArrow: n <= n + 1;
				downArrow: n <= n - 1;
            default: ledToggle <= 1'b0;  // Reset LED for other keys
        endcase
    end

    // Handle delay and volume value based on ledToggle
    if (ledToggle) begin
        volume <= snd ? 32'd10000000 : -32'd10000000;  // Set volume based on sound toggle
        
				/*
				so, if n > 0, delay = 50,000,000 / (baseFreq << n);
				if n < 0, delay = 50000000 / (baseFreq >> n);
				else (n=0), delay = 50000000 / baseFreq;
				
				*/
				 if (n > 0) begin
					  scaledFreq = 50000000 / (1 << n);  // Equivalent to dividing by 2^n
				 end else if (n < 0) begin
					  scaledFreq = 50000000 * (1 << -n); // Equivalent to multiplying by 2^(-n)
				 end else begin
					  scaledFreq = 50000000;             // No scaling if n == 0
				 end

				 // Use a single case statement with the precomputed scaled frequency
				 case (last_data_received)
					  A: delay <= scaledFreq / baseFreqA; // octave 4 A
					  B: delay <= scaledFreq / baseFreqB;
					  C: delay <= scaledFreq / baseFreqC;
					  D: delay <= scaledFreq / baseFreqD;
					  E: delay <= scaledFreq / baseFreqE;
					  F: delay <= scaledFreq / baseFreqF;
					  G: delay <= scaledFreq / baseFreqG; // octave 5 G
					  default: delay <= 19'd1000;         // Default delay if no specific key
				 endcase
    end else begin
        delay <= 19'd10000;  // Default delay if ledToggle is not active
        volume <= 32'd0;      // Reset volume if ledToggle is not active
    end
end
/*
always @(posedge CLOCK_50) begin
    // SPEAKER: Handle delay logic
    if (delay_cnt == delay) begin
        delay_cnt <= 0;
        snd <= !snd;  // Toggle sound
    end else begin
        delay_cnt <= delay_cnt + 1;
    end

    // KEYBOARD: Handle key presses
    if (KEY[0] == 1'b0) begin
        last_data_received <= 8'h00;  // Reset on KEY[0] press
    end else if (ps2_key_pressed == 1'b1) begin
        last_data_received <= ps2_key_data;  // Store the key data

        // Toggle LED for specific keys (A, B, C, D, E, F, G)
        case(last_data_received)
            A, B, C, D, E, F, G: ledToggle <= 1'b1;  // Set LED toggle if keys A-G are pressed
            default: ledToggle <= 1'b0;  // Reset LED for other keys
        endcase
    end

    // Handle delay and volume value based on ledToggle
    if (ledToggle) begin
        volume <= snd ? 32'd10000000 : -32'd10000000;  // Set volume based on sound toggle
        case(last_data_received)
            A: delay <= 19'd113636;
            B: delay <= 19'd101214;
            C: delay <= 19'd95555;
				D: delay <= 19'd85132;
				E: delay <= 19'd75842;
				F: delay <= 19'd71586;
				G: delay <= 19'd63776;				
            default: delay <= 19'd1000;  // Default delay if no specific key
        endcase
    end else begin
        delay <= 19'd10000;  // Default delay if ledToggle is not active
        volume <= 32'd0;      // Reset volume if ledToggle is not active
    end
end
*/

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

 //KEYBOARD
 assign LEDR[0] = ledToggle;
 
assign HEX2 = 7'h7F;
assign HEX3 = 7'h7F;
assign HEX4 = 7'h7F;
assign HEX5 = 7'h7F;
assign HEX6 = 7'h7F;
assign HEX7 = 7'h7F;
 
 
 
 //SPEAKER
 
 
//assign delay = {SW[3:0], 15'd3000};

//wire [31:0] volume = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;


assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in+volume;
assign right_channel_audio_out	= right_channel_audio_in+volume;
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


