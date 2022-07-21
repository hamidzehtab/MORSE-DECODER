
// Copyright (C) Hamid Reza Zehtab, FUM ,  Winter 2022 (1400 Hijri Shamsi)
// kod daneshjoiee : 9912762541
// Designed and Created : Hamid Reza Zehtab

module morse_decoder(
    input CLOCK_50,
    input [3:0] KEY,
    input [17:0] SW,
    output [6:0]    HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,
    output [8:0] LEDG,
    output [17:0] LEDR,
    inout [35:0] GPIO_0,GPIO_1,
    output LCD_ON,
    output LCD_BLON,
    output LCD_RW,
    output LCD_EN,
    output LCD_RS,
    inout [7:0] LCD_DATA,
    input CLOCK_27,
    output TD_RESET,
    inout I2C_SDAT,
    output I2C_SCLK,
    output AUD_ADCLRCK,
    input  AUD_ADCDAT,
    output AUD_DACLRCK,
    output AUD_DACDAT,
    inout AUD_BCLK, 
    output AUD_XCK
);
wire DLY_RST;
Reset_Delay r0(     .iCLK(CLOCK_50) ,.oRESET(DLY_RST) );

assign  LCD_ON = 1'b1;
assign LCD_BLON = 1'b1;

assign GIPO_0 = 36'hzzzzzzzzz;
assign GIPO_1 = 36'hzzzzzzzzz;

wire [6:0] myclock;
assign RST = KEY[0];

wire clk;
unit_counter U (CLOCK_50, SW[3:0], clk);

wire [1:0] beep;
wire clk2;
assign LEDG[0] = clk2;
assign LEDR[1:0] = beep;
keyer K (clk , KEY[0] , beep, clk2);

wire newletter;
wire [6:0] letter;
fsm F (clk2, beep, letter , LEDR[17:12] , newsletter);

reg [7:0] rhex0 = 8'h20;
reg [7:0] rhex1 = 8'h20;
reg [7:0] rhex2 = 8'h20;
reg [7:0] rhex3 = 8'h20;
reg [7:0] rhex4 = 8'h20;
reg [7:0] rhex5 = 8'h20;
reg [7:0] rhex6 = 8'h20;
reg [7:0] rhex7 = 8'h20;
reg [7:0] rhex8 = 8'h20;

wire [7:0] new_letter;

wire rst;
assign rst = KEY[1];

mux8bit9tol MUX1 (letter , now_letter);

assign LEDG[3] = new_letter;

always @(posedge newletter or negedge rst ) 
begin
    if (~rst) begin
        rhex8 = 8'h20;
        rhex7 = 8'h20;
        rhex6 = 8'h20;
        rhex5 = 8'h20;
        rhex4 = 8'h20;
        rhex3 = 8'h20;
        rhex2 = 8'h20;
        rhex1 = 8'h20;
        rhex0 = 8'h20;
    end else begin
        rhex8 = rhex7;
        rhex7 = rhex6;
        rhex6 = rhex5;
        rhex5 = rhex4;
        rhex4 = rhex3;
        rhex3 = rhex2;
        rhex2 = rhex1;
        rhex1 = rhex0;
        rhex0 = new_letter;
    end
end

LCD_Display u1(
     .iCLK_50MHZ(CLOCK_50),
     .iRST_N(DLY_RST),
     .hex0(rhex0),
    .hex1(rhex1),
    .hex2(rhex2),
    .hex3(rhex3),
    .hex4(rhex4),
    .hex5(rhex5),
    .hex6(rhex6),
    .hex7(rhex7),
    .hex8(rhex8),
    .DATA_BUS(LCD_DATA),
    .LCD_RW(LCD_RW),
    .LCD_E(LCD_EN),
    .LCD_RS(LCD_RS)
);

endmodule




















module fsm(clk2, in, letter , LEDR , newletter);
    input clk2;
    input [1:0] in;
    output [5:0] LEDR;
    output reg [5:0] letter = 6'b000000;
    output reg newletter;
    reg [5:0] state = 6'b000000;

    parameter [1:0] DOT = 2'b01;
    parameter [1:0] DASH = 2'b11;
    parameter [1:0] GAP = 2'b00;

    parameter [5:0] BLANK = 6'b111111;
    parameter [5:0] A = 6'b011010;
    parameter [5:0] B = 6'b000001;
    parameter [5:0] C = 6'b000010;
    parameter [5:0] D = 6'b000011;
    parameter [5:0] E = 6'b000100;
    parameter [5:0] F = 6'b000101;
    parameter [5:0] G = 6'b000110;
    parameter [5:0] H = 6'b000111;
    parameter [5:0] I = 6'b001000;
    parameter [5:0] J = 6'b001001;
    parameter [5:0] K = 6'b001010;
    parameter [5:0] L = 6'b001011;
    parameter [5:0] M = 6'b001100;
    parameter [5:0] N = 6'b001101;
    parameter [5:0] O = 6'b001110;
    parameter [5:0] P = 6'b001111;
    parameter [5:0] Q = 6'b010000;
    parameter [5:0] R = 6'b010001;
    parameter [5:0] S = 6'b010010;
    parameter [5:0] T = 6'b010011;
    parameter [5:0] U = 6'b010100;
    parameter [5:0] V = 6'b010101;
    parameter [5:0] W = 6'b010110;
    parameter [5:0] X = 6'b010111;
    parameter [5:0] Y = 6'b011000;
    parameter [5:0] Z = 6'b011001;
    parameter [5:0] ONE  = 6'b011010;
    parameter [5:0] TWO = 6'b011011;
    parameter [5:0] THREE = 6'b011100;
    parameter [5:0] FOUR = 6'b011101;
    parameter [5:0] FIVE = 6'b011110;
    parameter [5:0] SIX = 6'b011111;
    parameter [5:0] SEVEN = 6'b100000;
    parameter [5:0] EIGHT = 6'b100001;
    parameter [5:0] NINE = 6'b100010;
    parameter [5:0] ZERO = 6'b100011;
    parameter [5:0] BEFORETWO = 6'b100100;
    parameter [5:0] BEFOREEIGHT = 6'b100101;
    parameter [5:0] BEFORENINE = 6'b100110;

  always @(posedge clk2 ) 
  begin
  newletter = 0;
      case (state)
          BLANK: case(in)
                 DOT: state = E;
                 DASH: state = T;
                 default :      begin newletter = 1;
                          letter = BLANK;
                          end
                endcase

          E : case(in)
                 DOT: state = I;
                 DASH: state = A;
                 default :      begin newletter = 1;
                          letter = E;
                          state = BLANK;
                          end
                endcase
          T : case(in)
                 DOT: state = N;
                 DASH: state = M;
                 default :      begin newletter = 1;
                          letter = T;
                          state = BLANK;
                          end
                endcase
          I : case(in)
                 DOT: state = S;
                 DASH: state = U;
                 default :      begin newletter = 1;
                          letter = I;
                          state = BLANK;
                          end
                endcase
          A : case(in)
                 DOT: state = R;
                 DASH: state = W;
                 default :      begin newletter = 1;
                          letter = A;
                          state = BLANK;
                          end
                endcase
          N : case(in)
                 DOT: state = D;
                 DASH: state = K;
                 default :      begin newletter = 1;
                          letter = N;
                          state = BLANK;
                          end
                endcase
          M : case(in)
                 DOT: state = G;
                 DASH: state = O;
                 default :      begin newletter = 1;
                          letter = M;
                          state = BLANK;
                          end
                endcase
          S : case(in)
                 DOT: state = H;
                 DASH: state = V;
                 default :      begin newletter = 1;
                          letter = S;
                          state = BLANK;
                          end
                endcase
          R : case(in)
                 DOT: state = L;
                 default :      begin newletter = 1;
                          letter = R;
                          state = BLANK;
                          end
                endcase
          U : case(in)
                 DOT: state = P;
                 DASH: state = BEFORETWO;
                 default :      begin newletter = 1;
                          letter = U;
                          state = BLANK;
                          end
                endcase
          W : case(in)
                 DOT: state = P;
                 DASH: state = J;
                 default :      begin newletter = 1;
                          letter = W;
                          state = BLANK;
                          end
                endcase
          D : case(in)
                 DOT: state = B;
                 DASH: state = X;
                 default :      begin newletter = 1;
                          letter = D;
                          state = BLANK;
                          end
                endcase
          K : case(in)
                 DOT: state = C;
                 DASH: state = Y;
                 default :      begin newletter = 1;
                          letter = K;
                          state = BLANK;
                          end
                endcase
          G : case(in)
                 DOT: state = Z;
                 DASH: state = Q;
                 default :      begin newletter = 1;
                          letter = G;
                          state = BLANK;
                          end
                endcase
          J : case(in)
                 DASH: state = ONE;
                 default :      begin newletter = 1;
                          letter = J;
                          state = BLANK;
                          end
                endcase
          V : case(in)
                 DASH: state = THREE;
                 default :      begin newletter = 1;
                          letter = V;
                          state = BLANK;
                          end
                endcase
          H : case(in)
                 DOT: state = FIVE;
                 DASH: state = FOUR;
                 default :      begin newletter = 1;
                          letter = H;
                          state = BLANK;
                          end
                endcase
          B : case(in)
                 DOT: state = SEVEN;
                 default :      begin newletter = 1;
                          letter = Z;
                          state = BLANK;
                          end
                endcase
          O : case(in)
                 default :      begin newletter = 1;
                          letter = C;
                          state = BLANK;
                          end
                endcase
          BEFORETWO : case(in)
                 DASH: state = TWO;
                 default :        state = state;
                endcase
          BEFOREEIGHT : case(in)
                 DASH: state = EIGHT;
                 default : state = state;
                endcase
          BEFORENINE : case(in)
                 DOT: state = NINE;
                 DASH: state = ZERO;
                 default :    state = state;
                endcase
          TWO : case(in)
                GAP: begin
                    letter = TWO;
                    state = BLANK;
                    end
                 default :   state = state;
                endcase
           default :      begin 
                          state = BLANK;
                          newletter = 1;
                          end
                endcase
            end
endmodule
          




















          
module unit_conter(clk, unit_freq , out_clk);
    input clk;
    input [3:0] unit_freq;
    output reg out_clk = 0;

    reg [31:0] counter = 0;

  always @(posedge clk ) begin
      case (counter)
          15000000 :            begin
                            out_clk = 1;
                            counter = 0;  
                                end
          7500000 :            begin
                            out_clk = 0;
                            counter = counter + 1;  
                            end
            default : counter = counter + 1;

      endcase
  end
endmodule























module mux8bit9tol(select , q);
input [8:0] select;
output [7:0] q;

reg [7:0] q;

parameter [5:0] BLANK = 6'b000000;
parameter [5:0] A = 6'b011010;
parameter [5:0] B = 6'b000001;
parameter [5:0] C = 6'b000010;
parameter [5:0] D = 6'b000011;
parameter [5:0] E = 6'b000100;
parameter [5:0] F = 6'b000101;
parameter [5:0] G = 6'b000110;
parameter [5:0] H = 6'b000111;
parameter [5:0] I = 6'b001000;
parameter [5:0] J = 6'b001001;
parameter [5:0] K = 6'b001010;
parameter [5:0] L = 6'b001011;
parameter [5:0] M = 6'b001100;
parameter [5:0] N = 6'b001101;
parameter [5:0] O = 6'b001110;
parameter [5:0] P = 6'b001111;
parameter [5:0] Q = 6'b010000;
parameter [5:0] R = 6'b010001;
parameter [5:0] S = 6'b010010;
parameter [5:0] T = 6'b010011;
parameter [5:0] U = 6'b010100;
parameter [5:0] V = 6'b010101;
parameter [5:0] W = 6'b010110;
parameter [5:0] X = 6'b010111;
parameter [5:0] Y = 6'b011000;
parameter [5:0] Z = 6'b011001;
parameter [5:0] ONE  = 6'b011010;
parameter [5:0] TWO = 6'b011011;
parameter [5:0] THREE = 6'b011100;
parameter [5:0] FOUR = 6'b011101;
parameter [5:0] FIVE = 6'b011110;
parameter [5:0] SIX = 6'b011111;
parameter [5:0] SEVEN = 6'b100000;
parameter [5:0] EIGHT = 6'b100001;
parameter [5:0] NINE = 6'b100010;
parameter [5:0] ZERO = 6'b100011;
parameter [5:0] BEFORETWO = 6'b100100;
parameter [5:0] BEFOREEIGHT = 6'b100101;
parameter [5:0] BEFORENINE = 6'b100110;

always @ ( select ) 
begin
    case ( select )
     BLANK :  q = 8'h20;
     A :  q = 8'h41;
     B :  q = 8'h42;
     C :  q = 8'h43;
     D :  q = 8'h44; 
     E :  q = 8'h45;
     F :  q = 8'h46;
     G :  q = 8'h47;
     H :  q = 8'h48;
     I :  q = 8'h49;
     J :  q = 8'h4A;
     K :  q = 8'h4B;
     L :  q = 8'h4C;
     M :  q = 8'h4D;
     N :  q = 8'h4E;
     O :  q = 8'h4F;
     P :  q = 8'h50;
     Q :  q = 8'h51;
     R :  q = 8'h52;
     S :  q = 8'h53;
     T :  q = 8'h54;
     U :  q = 8'h55;
     V :  q = 8'h56;
     W :  q = 8'h57;
     X :  q = 8'h58;
     Y :  q = 8'h59;
     Z :  q = 8'h5A;
     ONE :  q = 8'h31;
     TWO :  q = 8'h32;
     THREE :  q = 8'h33;
     FOUR :  q = 8'h34;
     FIVE :  q = 8'h35;
     SIX :  q = 8'h36;
     SEVEN :  q = 8'h37;
     EIGHT :  q = 8'h38;
     NINE :  q = 8'h39;
     ZERO :  q = 8'h30;
     default : q = 8'h20;
    endcase
end
endmodule
















module lcdlab3(
  input CLOCK_50,    //    50 MHz clock
  input [3:0] KEY,      //    Pushbutton[3:0]
  input [17:0] SW,    //    Toggle Switch[17:0]
  output [6:0]    HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,  // Seven Segment Digits
  output [8:0] LEDG,  //    LED Green
  output [17:0] LEDR,  //    LED Red
  inout [35:0] GPIO_0,GPIO_1,    //    GPIO Connections
//    LCD Module 16X2
  output LCD_ON,    // LCD Power ON/OFF
  output LCD_BLON,    // LCD Back Light ON/OFF
  output LCD_RW,    // LCD Read/Write Select, 0 = Write, 1 = Read
  output LCD_EN,    // LCD Enable
  output LCD_RS,    // LCD Command/Data Select, 0 = Command, 1 = Data
  inout [7:0] LCD_DATA    // LCD Data bus 8 bits
);

//    All inout port turn to tri-state
assign    GPIO_0        =    36'hzzzzzzzzz;
assign    GPIO_1        =    36'hzzzzzzzzz;

wire [6:0] myclock;
wire RST;
assign RST = KEY[0];

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(    .iCLK(CLOCK_50),.oRESET(DLY_RST) );

// Send switches to red leds 
assign LEDR = SW;

// turn LCD ON
assign    LCD_ON        =    1'b1;
assign    LCD_BLON    =    1'b1;

wire [3:0] hex1, hex0;
assign hex1 = SW[7:4];
assign hex0 = SW[3:0];


LCD_Display u1(
// Host Side
   .iCLK_50MHZ(CLOCK_50),
   .iRST_N(DLY_RST),
   .hex0(hex0),
   .hex1(hex1),
// LCD Side
   .DATA_BUS(LCD_DATA),
   .LCD_RW(LCD_RW),
   .LCD_E(LCD_EN),
   .LCD_RS(LCD_RS)
);


// blank unused 7-segment digits
assign HEX0 = 7'b111_1111;
assign HEX1 = 7'b111_1111;
assign HEX2 = 7'b111_1111;
assign HEX3 = 7'b111_1111;
assign HEX4 = 7'b111_1111;
assign HEX5 = 7'b111_1111;
assign HEX6 = 7'b111_1111;
assign HEX7 = 7'b111_1111;

endmodule

























/*
 SW8 (GLOBAL RESET) resets LCD
ENTITY LCD_Display IS
-- Enter number of live Hex hardware data values to display
-- (do not count ASCII character constants)
    GENERIC(Num_Hex_Digits: Integer:= 2); 
-----------------------------------------------------------------------
-- LCD Displays 16 Characters on 2 lines
-- LCD_display string is an ASCII character string entered in hex for 
-- the two lines of the  LCD Display   (See ASCII to hex table below)
-- Edit LCD_Display_String entries above to modify display
-- Enter the ASCII character's 2 hex digit equivalent value
-- (see table below for ASCII hex values)
-- To display character assign ASCII value to LCD_display_string(x)
-- To skip a character use 8'h20" (ASCII space)
-- To dislay "live" hex values from hardware on LCD use the following: 
--   make array element for that character location 8'h0" & 4-bit field from Hex_Display_Data
--   state machine sees 8'h0" in high 4-bits & grabs the next lower 4-bits from Hex_Display_Data input
--   and performs 4-bit binary to ASCII conversion needed to print a hex digit
--   Num_Hex_Digits must be set to the count of hex data characters (ie. "00"s) in the display
--   Connect hardware bits to display to Hex_Display_Data input
-- To display less than 32 characters, terminate string with an entry of 8'hFE"
--  (fewer characters may slightly increase the LCD's data update rate)
------------------------------------------------------------------- 
--                        ASCII HEX TABLE
--  Hex                        Low Hex Digit
-- Value  0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
------\----------------------------------------------------------------
--H  2 |  SP  !   "   #   $   %   &   '   (   )   *   +   ,   -   .   /
--i  3 |  0   1   2   3   4   5   6   7   8   9   :   ;   <   =   >   ?
--g  4 |  @   A   B   C   D   E   F   G   H   I   J   K   L   M   N   O
--h  5 |  P   Q   R   S   T   U   V   W   X   Y   Z   [   \   ]   ^   _
--   6 |  `   a   b   c   d   e   f   g   h   i   j   k   l   m   n   o
--   7 |  p   q   r   s   t   u   v   w   x   y   z   {   |   }   ~ DEL
-----------------------------------------------------------------------
-- Example "A" is row 4 column 1, so hex value is 8'h41"
-- *see LCD Controller's Datasheet for other graphics characters available
*/
        
module LCD_Display(iCLK_50MHZ, iRST_N, hex1, hex0, 
    LCD_RS,LCD_E,LCD_RW,DATA_BUS);
input iCLK_50MHZ, iRST_N;
input [3:0] hex1, hex0;
output LCD_RS, LCD_E, LCD_RW;
inout [7:0] DATA_BUS;

parameter
HOLD = 4'h0,
FUNC_SET = 4'h1,
DISPLAY_ON = 4'h2,
MODE_SET = 4'h3,
Print_String = 4'h4,
LINE2 = 4'h5,
RETURN_HOME = 4'h6,
DROP_LCD_E = 4'h7,
RESET1 = 4'h8,
RESET2 = 4'h9,
RESET3 = 4'ha,
DISPLAY_OFF = 4'hb,
DISPLAY_CLEAR = 4'hc;

reg [3:0] state, next_command;
// Enter new ASCII hex data above for LCD Display
reg [7:0] DATA_BUS_VALUE;
wire [7:0] Next_Char;
reg [19:0] CLK_COUNT_400HZ;
reg [4:0] CHAR_COUNT;
reg CLK_400HZ, LCD_RW_INT, LCD_E, LCD_RS;

// BIDIRECTIONAL TRI STATE LCD DATA BUS
assign DATA_BUS = (LCD_RW_INT? 8'bZZZZZZZZ: DATA_BUS_VALUE);

LCD_display_string u1(
.index(CHAR_COUNT),
.out(Next_Char),
.hex1(hex1),
.hex0(hex0));

assign LCD_RW = LCD_RW_INT;

always @(posedge iCLK_50MHZ or negedge iRST_N)
    if (!iRST_N)
    begin
       CLK_COUNT_400HZ <= 20'h00000;
       CLK_400HZ <= 1'b0;
    end
    else if (CLK_COUNT_400HZ < 20'h0F424)
    begin
       CLK_COUNT_400HZ <= CLK_COUNT_400HZ + 1'b1;
    end
    else
    begin
      CLK_COUNT_400HZ <= 20'h00000;
      CLK_400HZ <= ~CLK_400HZ;
    end
// State Machine to send commands and data to LCD DISPLAY

always @(posedge CLK_400HZ or negedge iRST_N)
    if (!iRST_N)
    begin
     state <= RESET1;
    end
    else
    case (state)
    RESET1:            
// Set Function to 8-bit transfer and 2 line display with 5x8 Font size
// see Hitachi HD44780 family data sheet for LCD command and timing details
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET2;
      CHAR_COUNT <= 5'b00000;
    end
    RESET2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET3;
    end
    RESET3:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= FUNC_SET;
    end
// EXTRA STATES ABOVE ARE NEEDED FOR RELIABLE PUSHBUTTON RESET OF LCD

    FUNC_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_OFF;
    end

// Turn off Display and Turn off cursor
    DISPLAY_OFF:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h08;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_CLEAR;
    end

// Clear Display and Turn off cursor
    DISPLAY_CLEAR:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h01;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_ON;
    end

// Turn on Display and Turn off cursor
    DISPLAY_ON:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h0C;
      state <= DROP_LCD_E;
      next_command <= MODE_SET;
    end

// Set write mode to auto increment address and move cursor to the right
    MODE_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h06;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Write ASCII hex character in first LCD character location
    Print_String:
    begin
      state <= DROP_LCD_E;
      LCD_E <= 1'b1;
      LCD_RS <= 1'b1;
      LCD_RW_INT <= 1'b0;
    // ASCII character to output
      if (Next_Char[7:4] != 4'h0)
        DATA_BUS_VALUE <= Next_Char;
        // Convert 4-bit value to an ASCII hex digit
      else if (Next_Char[3:0] >9)
        // ASCII A...F
         DATA_BUS_VALUE <= {4'h4,Next_Char[3:0]-4'h9};
      else
        // ASCII 0...9
         DATA_BUS_VALUE <= {4'h3,Next_Char[3:0]};
    // Loop to send out 32 characters to LCD Display  (16 by 2 lines)
      if ((CHAR_COUNT < 31) && (Next_Char != 8'hFE))
         CHAR_COUNT <= CHAR_COUNT + 1'b1;
      else
         CHAR_COUNT <= 5'b00000; 
    // Jump to second line?
      if (CHAR_COUNT == 15)
        next_command <= LINE2;
    // Return to first line?
      else if ((CHAR_COUNT == 31) || (Next_Char == 8'hFE))
        next_command <= RETURN_HOME;
      else
        next_command <= Print_String;
    end

// Set write address to line 2 character 1
    LINE2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'hC0;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Return write address to first character postion on line 1
    RETURN_HOME:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h80;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// The next three states occur at the end of each command or data transfer to the LCD
// Drop LCD E line - falling edge loads inst/data to LCD controller
    DROP_LCD_E:
    begin
      LCD_E <= 1'b0;
      state <= HOLD;
    end
// Hold LCD inst/data valid after falling edge of E line                
    HOLD:
    begin
      state <= next_command;
    end
    endcase
endmodule

module LCD_display_string(index,out,hex0,hex1);
input [4:0] index;
input [3:0] hex0,hex1;
output [7:0] out;
reg [7:0] out;
// ASCII hex values for LCD Display
// Enter Live Hex Data Values from hardware here
// LCD DISPLAYS THE FOLLOWING:
//----------------------------
//| Count=XX                  |
//| DE2                       |
//----------------------------
// Line 1
   always 
     case (index)
    5'h00: out <= 8'h43;
    5'h01: out <= 8'h6F;
    5'h02: out <= 8'h75;
    5'h03: out <= 8'h6E;
    5'h04: out <= 8'h74;
    5'h05: out <= 8'h3D;
    5'h06: out <= {4'h0,hex1};
    5'h07: out <= {4'h0,hex0};
// Line 2
    5'h10: out <= 8'h44;
    5'h11: out <= 8'h45;
    5'h12: out <= 8'h32;
    default: out <= 8'h20;
     endcase
endmodule


























module    Reset_Delay(iCLK,oRESET);
input        iCLK;
output reg    oRESET;
reg    [19:0]    Cont;

always@(posedge iCLK)
begin
    if(Cont!=20'hFFFFF)
    begin
        Cont    <=    Cont+1'b1;
        oRESET    <=    1'b0;
    end
    else
    oRESET    <=    1'b1;
end

endmodule














