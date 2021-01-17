`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/11/2019 01:33:20 AM
// Design Name: 
// Module Name: RISC_V_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module project_top(clk, reset, BTNU);
input clk, reset, BTNU; //BTNU = pulse

wire AND_out;
wire [31:0] rd0_data, rd1_data, wr_data, DataMemOut, ALU_out, rd1_alt, instruction;
wire [31:0] PC_out, PC_in, GND, Imm, ALU_input1, ALU_input2, MuxGen, shifted_Imm, sum1, sum2;
wire wr_en, ALUsrc1, ALUsrc2, MemWrite, MemToReg, branch, pulse_out, vflag, zflag, rd1_sel, muxout;
wire [3:0] ALUop;
wire [4:0] rd0_addr, rd1_addr, rd;

assign GND = 32'd0;

inst_decode #(32) inst0(instruction, rd0_addr, rd1_addr, rd, branch, wr_en, ALUsrc1, ALUsrc2, ALUop, MemWrite, MemToReg);
Debounce inst1(BTNU, clk, pulse_out);
reg_file #(5,32) inst2(reset, pulse_out, wr_en, rd0_addr, rd1_addr, rd, wr_data, rd0_data, rd1_data);
mux1 #(32) rd0_mux(rd0_data, GND, ALUsrc1, ALU_input1);
mux2 #(32) imm_mux(Imm, rd1_alt, muxout, MuxGen); 
mux2 #(32) rd1_mux(rd1_data, MuxGen, muxout, ALU_input2); //could be bad
mux2 #(1) vio_mux(ALUsrc2, rd1_sel, rd1_sel, muxout);
ALU #(32) inst6(ALU_input1, ALU_input2, ALUop, ALU_out, zflag, vflag);
data_memory #(32) inst7(pulse_out, ALU_out, rd1_data, MemWrite, DataMemOut);
mux2 #(32) data_mem_mux(ALU_out, DataMemOut, MemToReg, wr_data);
ImmGen #(32) inst10(instruction, Imm);
PC #(32) inst11(pulse_out, reset, PC_in, PC_out);
inst_memory #(32) inst12(PC_out, instruction);
//shift_imm #(32) inst9(Imm, shifted_Imm);
Adder #(32) PC_1(32'd1, PC_out, sum1);
Adder #(32) branch_sum(PC_out, Imm, sum2);
AND_gate inst8(zflag, branch, AND_out);
mux2 #(32) PC_mux(sum1, sum2, AND_out, PC_in);


vio_0 your_instance_name (
  .clk(clk),                // input wire clk
  .probe_in0(branch),    // input wire [0 : 0] probe_in0
  .probe_in1(zflag),    // input wire [0 : 0] probe_in1
  .probe_in2(vflag),    // input wire [0 : 0] probe_in2
  .probe_in3(Imm),    // input wire [31 : 0] probe_in3
  .probe_in4(PC_in),    // input wire [31 : 0] probe_in4
  .probe_in5(PC_out),    // input wire [31 : 0] probe_in5
  .probe_in6(instruction),    // input wire [31 : 0] probe_in6
  .probe_in7(wr_data),    // input wire [31 : 0] probe_in7
  .probe_in8(ALUop),    // input wire [3 : 0] probe_in8
  .probe_in9(ALU_out),    // input wire [31 : 0] probe_in9
  .probe_in10(rd0_data),  // input wire [31 : 0] probe_in10       Might have to modify to rd0_data (32-bit)
  .probe_in11(rd1_data), //input wire [31 : 0] probe_in11         Might have to modify to rd1_data (32-bit)
  .probe_in12(rd),      // input wire [4 : 0] probe_in12
  .probe_out0(rd1_alt),  // output wire [31 : 0] probe_out0
  .probe_out1(rd1_sel)  // output wire [0 : 0] probe_out1
);
endmodule



module ALU
#(parameter w = 8) 
(A, B, Sel, F, Z, V);

input [w-1:0] A, B;
input [3:0] Sel;
output [w-1:0] F;
output V, Z;


wire overflow;
wire underflow;
reg ext;
reg [w-1:0] F;
wire addSub;

initial 
    begin
        ext = 0;
    end
    
assign overflow = (((Sel == 4'd2) || (Sel == 4'd6)) && {ext, F[w-1]} == 2'b01) ? 1:0;
assign underflow = (((Sel == 4'd6) || (Sel == 4'd2)) && {ext, F[w-1]} == 2'b10) ? 1:0;
//Underflow change
//assign V = ((overflow) || (underflow)) ? 1:0;

wire [w - 1 : 0] f_out; 
assign addSub = (Sel == 4'h4) ? 1 : 0; 
f_arithmetic F_ARITH(A, B, addSub, f_out);

always @ (*)
    begin
        case (Sel)
            4'h0:
                F <= A & B;
            4'h1:
                F <= A | B;
            4'h2:
                //F = A + B;
                {ext, F} <= {A[w-1], A} + {B[w-1], B};
            4'h3://fadd
                 F = f_out;
            4'h4://fsub
                 F = f_out;
            4'h6:
                //F = A - B;
                {ext, F} <= {A[w-1], A} - {B[w-1], B};
            4'h7:
                F <= (A < B) ? 1:0;
            4'hC:
                F <= ~(A | B);
        endcase
    end    
assign Z = (F == 0) ? 1:0;
assign V = ((overflow) || (underflow)) ? 1:0;

endmodule



module f_arithmetic (A,B,addSub,F);
input [31:0] A, B;
input addSub;
output [31:0] F;

reg [7:0] exponent_A, exponent_B;
reg [7:0] difference;
reg [7:0] exponent_out;


reg sign_bit_A, sign_bit_B;
reg sign_bit_out;

reg [25:0] mantissa_A, mantissa_B;
reg [25:0] mantissa_sum;


//initial
//begin
////    exponent_A = A[30:23];
////    exponent_B = B[30:23];
////    mantissa_A = A [22:0];
////    mantissa_B = B [22:0];
////    sign_bit_A = A [31];
////    sign_bit_B = B [31];

//end

integer position, adjust, i;

reg [31:0] c, d;

always @ (*)
begin
    //Compare the exponents and shift
//    if(exponent_A < exponent_B)
    if(A[30:23] < B[30:23])
    begin
//        difference = exponent_B - exponent_A;
//        //exponent_A = exponent_B;
//        mantissa_A = {2'b00, (exponent_A) ? 1'b1 : 1'b0, A[22:0]};
//        mantissa_B = {2'b00, (exponent_B) ? 1'b1 : 1'b0, B[22:0]};
//        mantissa_A = mantissa_A >> difference;
        c = B;
        d = A;
    end
    else
    begin
//        difference = exponent_A - exponent_B;
//        //exponent_B = exponent_A;
//        mantissa_A = {2'b00, (exponent_A) ? 1'b1 : 1'b0, A[22:0]};
//        mantissa_B = {2'b00, (exponent_B) ? 1'b1 : 1'b0, B[22:0]};
//        mantissa_B = mantissa_B >> difference;
         c = A;
         d = B;       
    end
    
    sign_bit_A = c[31];
    sign_bit_B = d[31];
    exponent_A =c[30:23];
    exponent_B =d[30:23];
    mantissa_A = {2'b00, (exponent_A) ? 1'b1 : 1'b0, c[22:0]};
    mantissa_B = {2'b00, (exponent_B) ? 1'b1 : 1'b0, d[22:0]};
    difference = exponent_A - exponent_B;
    mantissa_B = mantissa_B >> difference;
   
    //Compare the sign bits of both
    if(sign_bit_A)
        //mantissa_A = ~mantissa_A + 1; //2's complement
        mantissa_A = -mantissa_A;
    if(sign_bit_B)
        //mantissa_B = ~mantissa_B + 1; //2's complement
        mantissa_B = -mantissa_B;
    
    //Compute the sum
    if(addSub)
        mantissa_sum = mantissa_A - mantissa_B;
    else
        mantissa_sum = mantissa_A + mantissa_B;
        
    //Check sign of the sum
    sign_bit_out = mantissa_sum[25];
    
    if(sign_bit_out)
        //mantissa_sum = ~mantissa_sum + 1; //2's complement
        mantissa_sum = -mantissa_sum;
        
    //Normalize the sum (special cases(?))
     if(mantissa_sum[24])
     begin
//        if(A[30:23] < B[30:23])
//            exponent_out = exponent_B + 1; //change
//        else
//            exponent_out = exponent_A + 1;
        exponent_out = exponent_A + 1;
        mantissa_sum = mantissa_sum >> 1;
     end
     else if(mantissa_sum)
     begin
        position = 0;
        for(i = 23; i >= 0; i = i - 1)
            if(!position && mantissa_sum[i])
                position = i;
        adjust = 5'd23 - position;
//        if(A[30:23] < B[30:23])
//            if(B[30:23] < adjust)
//            begin
//                exponent_out = 0;
//                mantissa_sum = 0;
//                sign_bit_out = 0;
//            end
//            else
//            begin
//                exponent_out = B[30:23] - adjust;
//                mantissa_sum = mantissa_sum << adjust;
//            end  
        if(exponent_A < adjust)
        begin
            exponent_out = 0;
            mantissa_sum = 0;
            sign_bit_out = 0;
        end
        else
        begin
//            if(A[30:23] < adjust)
//            begin
//                exponent_out = 0;
//                mantissa_sum = 0;
//                sign_bit_out = 0;
//            end
//            else
//            begin
//                exponent_out = A[30:23] - adjust;
//                mantissa_sum = mantissa_sum << adjust;
//            end 
            exponent_out = exponent_A - adjust;
            mantissa_sum = mantissa_sum << adjust;
        end 
     end 
     else
     begin
        exponent_out = 0;
        mantissa_sum = 0;
     end    
   
end   
        
assign F = {sign_bit_out, exponent_out, mantissa_sum[22:0]};

endmodule





module inst_memory
#(parameter w = 4, dw = 32)
(PC_out, instruction);
input [dw - 1 : 0] PC_out;
output [dw - 1 : 0] instruction;

reg [dw - 1 : 0]instRom [0 : 15];



initial 
begin
        instRom[4'd0] = 32'h000070B3;
        instRom[4'd1] = 32'h00007133;
        instRom[4'd2] = 32'h000071B3;
        instRom[4'd3] = 32'h00007233;
        instRom[4'd4] = 32'h000072B3;
        instRom[4'd5] = 32'h00007333;
        instRom[4'd6] = 32'h000003B3;
        instRom[4'd7] = 32'h00000433;
        instRom[4'd8] = 32'h000000B3;
        instRom[4'd9] = 32'h00000133;
        instRom[4'd10] = 32'h002081D3;
        instRom[4'd11] = 32'h00000233;
        instRom[4'd12] = 32'h000002B3;
        instRom[4'd13] = 32'h0042F353;
        instRom[4'd14] = 32'h003303D3;
        instRom[4'd15] = 32'h0061F453;
end




assign instruction = instRom[PC_out [w - 1 : 0]]; //instRom is only 32 cells, PC_out will create 2^32 possibilities


endmodule


module inst_decode
#(parameter w = 32)
(instruction, rs1, rs2, rd, branch, wr_en, ALUsrc1, ALUsrc2, ALUop, MemWrite, MemToReg);
input [w - 1:0] instruction;
output [4:0] rs1, rs2, rd;
output reg branch, wr_en, ALUsrc1, ALUsrc2, MemWrite, MemToReg;
output reg [3:0] ALUop;


assign rs1 = instruction [19:15];
assign rs2 = instruction [24:20];
assign rd = instruction [11:7];
    
always @ (*)
begin
    case(instruction [6:0])
       7'b1010011: //FADD, FSUB
       begin
            branch <= 1'b0;
            ALUsrc1 <= 1'b0;
            ALUsrc2 <= 1'b0;
            MemWrite <= 1'b0;
            MemToReg <= 1'b0;
            wr_en <= 1'b1;
                
                case(instruction [14:12])
                    3'b000://FADD
                    begin
                        ALUop <= 4'b0011;
                    end
                    
                    3'b111://FSUB
                    begin
                        ALUop <= 4'b0100;
                    end
                    
                endcase
       end
       
       7'b0110011: //AND, OR, SUB, ADD
       begin       
              branch <= 1'b0; //not sure
              ALUsrc1 <= 1'b0;
              ALUsrc2 <= 1'b0;
              MemWrite <= 1'b0;
              MemToReg <= 1'b0;
              wr_en <= 1'b1;
            case(instruction [14:12]) //test funct3
                3'b000:
                begin
                    //test funct7
                    if(instruction [w - 1:25] == 7'b0000000) //ADD
                            ALUop <= 4'd2;   
                    else if(instruction [w - 1:25] == 7'b0100000) //SUB                          
                            ALUop <= 4'd3;  
                end
                
                3'b111://AND
                begin
                    ALUop <= 4'd0;
                end
                3'b110: //OR
                begin
                    ALUop <= 4'd1;
                end
            endcase
       end
       7'b0000011://lw
       begin
            branch <= 1'b0;
            wr_en <= 1'b1;
            ALUsrc1 <= 1'b0;
            ALUsrc2 <= 1'b1;
            ALUop <= 4'd2;
            MemWrite <= 0;
            MemToReg <= 1;
       end
       7'b0100011://sw
       begin
            branch = 1'b0;
            wr_en <= 1'b0;
            ALUsrc1 <= 1'b0;
            ALUsrc2 <= 1'b0;
            ALUop <= 4'd2;
            MemWrite <= 1'b1;
            MemToReg <= 1'b0;
       end
       7'b1100011://beq 
       begin    
            branch <= 1'b1;
            wr_en <= 1'b0;
            ALUsrc1 <= 1'b0;
            ALUsrc2 <= 1'b0;
            ALUop <= 4'd6;
            MemWrite <= 1'b0;
            MemToReg <= 1'b0;
       end
    endcase
end

endmodule

module PC
#(parameter w = 32)
(pulse_out, reset, PC_in, PC_out);
input pulse_out, reset;
input [w - 1 : 0] PC_in;
output reg [w - 1 : 0] PC_out;


always @ (posedge pulse_out)
begin
     if(reset)
        PC_out <= 32'd0;
     else
        PC_out <= PC_in; 
end 

endmodule


module Adder
#(parameter w = 32)
(add1, add2, sum);
input [w - 1 : 0] add1, add2;
output reg [w - 1 : 0] sum;


initial 
begin
    sum = 32'd0;
end

always @ (*)
begin
    sum <= add1 + add2;
end



endmodule

module AND_gate(in0, in1, out);
input in0, in1;
output out;

assign out = in0 & in1;

endmodule



module ImmGen
#(parameter w = 32)
(instruction, Imm);
input [w - 1 : 0] instruction;
output [w - 1 : 0] Imm;
 
reg [11:0] immediate;
 
always @ (*)
begin
    case(instruction [6:0])
       7'b1010011:
            immediate <= 12'd0;
       7'b0110011: //AND, OR, SUB, ADD    
       begin
            immediate <= 12'd0; //might need to change     
       end
       7'b0000011://lw
       begin
            immediate <= instruction[31:20];
       end     
       7'b0100011://sw
       begin
            immediate <= {instruction[31:25], instruction[11:7]};
       end
       7'b1100011://beq 
       begin    
            immediate <= {instruction[31], instruction[7], instruction[30:25], instruction[11:8]}; 
       end
    endcase
end

assign Imm = {{20{immediate[11]}}, immediate};
 
endmodule


module data_memory
#(parameter DW = 8)
(clk, Address, rd1_data, MemWrite, DataMemOut);
input clk, MemWrite;
input [DW - 1: 0] Address, rd1_data;
output [DW - 1: 0]DataMemOut;

reg [DW - 1:0] data_reg [0:(2**(DW)) - 1];

assign DataMemOut = data_reg [Address];

//synchronous write
always @ (posedge clk)
    begin
        if  (MemWrite)
            begin
                data_reg[Address] = rd1_data; 
            end    
    end 


endmodule


//asynchronous read, synchronous write (on the clock edge only) 
//read code is outside the always block
//rd0_addr
//reset 
module reg_file
#(parameter AW = 5, parameter  DW = 8)  //double-check AW
(Reset, clk, wr_en, rd0_addr, rd1_addr, wr_addr, wr_data, rd0_data, rd1_data);

input Reset, clk, wr_en;
input [AW-1:0] rd0_addr, rd1_addr,wr_addr;
input [DW - 1:0] wr_data;
output [DW - 1:0] rd0_data, rd1_data; 

reg [DW - 1:0] register [0: (2**AW)-1];

//asynchronous read
assign rd0_data = register [rd0_addr];
assign rd1_data = register [rd1_addr];


integer i;
//synchronous write
always @ (posedge clk)
    begin
        if(Reset == 1)
          for (i = 0; i < 2**AW; i = i + 1)
            begin
                register[i] <= 8'b00000000;    
            end  
        else if  (wr_en == 1)
            begin
                register[wr_addr] <= wr_data; 
            end    
    end 


endmodule

module mux1
#(parameter w = 8)
(in0, in1, sel, a);
input [w-1:0] in0, in1;
input  sel;
output [w-1:0] a;

assign in1 = 8'h00;

assign a = (sel) ? in1: in0; //might need to flip

endmodule


module mux2
#(parameter w = 8)
(in0, in1, sel, b);
input [w-1:0] in0, in1;
input  sel;
output [w-1:0] b;

assign b = (sel) ? in1: in0; //might need to flip

endmodule





module Debounce(BTNU, clk, pulse_out);
input clk;
input BTNU;
output pulse_out;

reg slow_clk;
reg [23:0] counter = 24'h000000;

initial
begin
    slow_clk = 0;
    counter = 24'd0;
end

always @ (posedge clk)
begin
    counter <= counter + 1'b1;
    if(counter == 24'h3D0900)
    //if(counter == 24'h1)
    begin
        slow_clk <= 1;
        counter <= 24'h000000;    
    end
    else
        slow_clk <= 0;
end

wire q1, q2, q3;

D_FF f1(BTNU, slow_clk, q1);
D_FF f2(q1, slow_clk, q2);
D_FF f3(q2, slow_clk, q3);

assign pulse_out = q2 && (~q3);


endmodule


module D_FF(D, clk, Q);
input clk, D;
output Q;

reg Q;

initial 
begin
    Q = 0;    
end

always @ (posedge clk)
begin
    Q <= D;
end
endmodule

