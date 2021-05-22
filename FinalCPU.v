`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/11/2019 12:50:39 AM
// Design Name: 
// Module Name: FinalCPU
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


module FinalCPU(input clk, BTNU, BTNC);
parameter W = 32;

wire branch, zflag, vflag, rd1_sel;
wire [3:0] aluop;
wire[W-1:0] imm_gen_out, pc_in, pc_out, inst_mem_out, wr_data, alu_out, reg_file_out_1, reg_file_out_2, rd1_alt;

wire wr_en, alusrc1, alusrc2, memwrite, memtoreg, pulse_out, M5_out;
wire[4:0] rs1_to_rd0_addr, rs2_to_rd1_addr, rd_to_wr_addr;
wire[W-1:0] rd1_data, rd0_data, datamemout, alu_a, alu_b, M2_out, A1_out, A2_out, shiftout, data_mem_out;

wire and_out = zflag & branch;

instmem #(32) A(pc_out, inst_mem_out);
pc B(pulse_out, BTNC, pc_in, pc_out); 
immgen C(inst_mem_out, imm_gen_out); 
decodertop D(inst_mem_out, rs1_to_rd0_addr, rs2_to_rd1_addr, rd_to_wr_addr, branch, wr_en, alusrc1, alusrc2, aluop, memwrite, memtoreg);
//decodertop(input[31:0] instruction, output [4:0] rs1, [4:0] rs2,
// [4:0] rd, output reg Branch, wr_en, ALUSrc1, ALUSrc2,
// output reg [3:0] ALUop, output reg MemWrite, MemToReg);
data_memory E(clk, memwrite, alu_out, rd1_data, datamemout);
//input clk, input memwrite, input [W-1:0]address,
//input [W-1:0]rd1_data, output [W-1:0] DataMemOut
fail_reg #(32, 5) F(pulse_out, BTNC, wr_en, rs1_to_rd0_addr, rs2_to_rd1_addr, rd_to_wr_addr, wr_data, rd0_data, rd1_data);
// input pulse_out, BTNC, wr_en, [V-1:0] rd0_addr, 
//[V-1:0] rd1_addr, [V-1:0]wr_addr, [W-1:0]wr_data,
//output [W-1:0]rd0_data, [W-1:0] rd1_data
Mux_1 #(32)G(rd0_data, 32'b0, alusrc1, alu_a);
Mux_2 #(32)M1(rd1_data, M2_out, M5_out, alu_b);
Mux_2 #(32)M2(imm_gen_out,rd1_alt, M5_out, M2_out); 
Mux_2 #(32)M3(A1_out, A2_out, and_out, pc_in);
Mux_2 #(32)M4(data_mem_out, alu_out, memtoreg, wr_data);
Mux_2 #(1)M5(alusrc2, rd1_sel, rd1_sel, M5_out);
ALU #(32)H(alu_a, alu_b, aluop, alu_out, vflag, zflag );
debounce(BTNU, clk, pulse_out);
adder A1(32'd1, pc_out, A1_out);
adder A2(pc_out, imm_gen_out, A2_out);
//shiftleft1 I(imm_gen_out, shiftout);

vio_0 Virtual (
  .clk(clk),                // input wire clk
  .probe_in0(branch),    // input wire [0 : 0] probe_in0
  .probe_in1(zflag),    // input wire [0 : 0] probe_in1
  .probe_in2(vflag),    // input wire [0 : 0] probe_in2
  .probe_in3(imm_gen_out),    // input wire [31 : 0] probe_in3
  .probe_in4(pc_in),    // input wire [31 : 0] probe_in4
  .probe_in5(pc_out),    // input wire [31 : 0] probe_in5
  .probe_in6(inst_mem_out),    // input wire [31 : 0] probe_in6
  .probe_in7(wr_data),    // input wire [31 : 0] probe_in7
  .probe_in8(aluop),    // input wire [3 : 0] probe_in8
  .probe_in9(alu_out),    // input wire [31 : 0] probe_in9
  .probe_in10(reg_file_out_1),  // input wire [31 : 0] probe_in10
  .probe_in11(reg_file_out_2),  // input wire [31 : 0] probe_in11
  .probe_out0(rd1_alt),  // output wire [31 : 0] probe_out0
  .probe_out1(rd1_sel)  // output wire [0 : 0] probe_out1
);

endmodule

module instmem #(parameter W = 32)(address,instruction);
input [W-1:0] address;
output reg [W-1:0] instruction;

 
integer i;

//initial
//begin
//    for (i = 0; i < W-1 ; i  = i + 1)
        
//            instruction[i] = 0;
 
//end 

always @ (*)
    begin
        case(address[3:0])
            4'd0:
                begin
                    instruction = 32'h000070B3;
                end
            4'd1:  
                begin
                    instruction = 32'h00007133;
                end 
            4'd2:
                begin
                    instruction = 32'h000071B3;
                end   
            4'd3:
                begin
                    instruction = 32'h00007233;
                end   
            4'd4:
                begin
                    instruction = 32'h000072B3;
                end   
            4'd5:
                begin
                    instruction = 32'h00007333;
                end   
            4'd6:
                begin
                    instruction = 32'h000080B3;
                end   
            4'd7:
                begin
                    instruction = 32'h00010133;
                end   
            4'd8: 
                begin
                    instruction = 32'h000282B3;
                end  
            4'd9:
                begin
                    instruction = 32'h00030333;
                end   
            4'd10:
                begin
                    instruction = 32'h00320233;
                end   
            4'd11:
                begin
                    instruction = 32'h005181B3;
                end   
            4'd12:
                begin
                    instruction = 32'h00618DE3;
                end   
            4'd13:
                begin
                    instruction = 32'hFE318DE3;
                end   
            4'd14:
                begin
                    instruction = 32'h0030A023;
                end
            4'd15:
                begin
                    instruction = 32'h00412023;
                end   
            default: 
                begin
                    instruction = 32'd0;
                end
        endcase   
    end
endmodule

module pc(pulse_out, rst, pc_in, pc_out);
input pulse_out, rst; 
input [31:0] pc_in;
output reg [31:0] pc_out;

always @(posedge pulse_out)
    begin
        if (rst == 0)
        begin
            pc_out <= pc_in;
        end
        else
        begin
            pc_out = 32'd0;
        end
    end
    
endmodule

module immgen(instruction, immediate);
input [31:0] instruction;
output reg [31:0] immediate;


wire [6:0] opcode;

assign opcode = instruction[6:0];

always @(*)
begin
case(opcode)
    7'b0000011: // i-type
            begin
                immediate[11:0] = instruction[31:20];
                immediate = {20{immediate[11]}};
            end   
    7'b0100011 : // s-type 
            begin
                immediate [4:0] = instruction[11:7];
                immediate[11:5] = instruction[31:25];
                
                immediate = {{20{immediate[11]}}, immediate};                                
            end
    7'b0110011 :  // r-type
            begin
               immediate = 32'd0; // r-type no imm
            end
    7'b1100011 :  // sb-type
            begin
                immediate = {20{instruction[31], instruction[7], instruction[30:25], instruction[11:8]}};
            end
endcase    
end



endmodule

module decodertop(input[31:0] instruction, output [4:0] rs1, [4:0] rs2, [4:0] rd, output reg Branch, wr_en, ALUSrc1, ALUSrc2, output reg [3:0] ALUop, output reg MemWrite, MemToReg);

assign rs1 = instruction[19:15];
assign rs2 = instruction[24:20];
assign rd = instruction[11:7];

always @(*)
    begin
        case (instruction[6:0]) //opcode 
         7'b0000011: //opcode = load word 
         begin
            wr_en <= 1;
            ALUSrc1 <= 0;
            ALUSrc2 <= 1;
            ALUop <= 4'b0010;
            MemWrite <= 0;
            MemToReg <= 1;  
            Branch <= 0;
         end
         7'b0100011: //opcode = store word
         begin
            wr_en <= 0;
            ALUSrc1 <= 0;
            ALUSrc2 <= 0;
            ALUop <= 0;
            MemWrite <= 1;
            MemToReg <= 0;
            Branch <= 0;
         end
         7'b1100011: //opcode = branch if equal
         begin
            wr_en <= 0;
            ALUSrc1 <= 0;
            ALUSrc2 <= 0;
            ALUop <= 6;
            MemWrite <= 0;
            MemToReg <= 0;
            Branch <= 1;    
         end
         default:
//        7'b0110011: //opcode = add, sub, or, and(R-TYPE)
            begin
            wr_en <= 1;
            ALUSrc1 <= 0;
            ALUSrc2 <= 0;
            ALUop <= 0;
            MemWrite <= 0;
            MemToReg <= 0;
            Branch <= 0;  
            if(instruction[31:25] == 7'b0100000) //funct7;sub
                begin
                ALUop <= 4'b0001;
                end
            else
                begin
                    case(instruction[14:12])
                        3'b000: //add
                            begin
                                ALUop <= 4'b0010;   
                            end 
                        3'b110: //or
                            begin
                                ALUop <= 4'b0001;
                            end
                        3'b111: //AND
                            begin
                                ALUop <= 4'b0000;
                            end
                     endcase
                 end//?
            end//?
        endcase
    end

endmodule




module data_memory #(parameter W = 32)(input clk, input memwrite, input [W-1:0]address, input [W-1:0]rd1_data, output [W-1:0] DataMemOut);
reg [W - 1:0] reginald [0: (2**(W))-1]; 

assign DataMemOut = reginald [address]; 

integer i;


always @ (posedge clk)
    begin   
        if(memwrite == 1) 
                reginald[address] <= rd1_data;//might be rd1_data
     end  
endmodule

module fail_reg #(parameter W = 32, V = 5)(input pulse_out, BTNC, wr_en, [V-1:0] rd0_addr, [V-1:0] rd1_addr, [V-1:0]wr_addr, [W-1:0]wr_data,
               output [W-1:0]rd0_data, [W-1:0] rd1_data);
               
reg [W - 1:0] reggie [0: (2**V)-1]; 


assign rd0_data = reggie [rd0_addr];
assign rd1_data = reggie [rd1_addr];

integer i;

always @ (posedge pulse_out)
    begin         if(BTNC ==1)
            for(i=0; i<2**V; i=i+1)
            begin   
                reggie[i] <= 8'b00000000;
            end
            else if (wr_en == 1)
            begin
                reggie[wr_addr] <= wr_data;
            end
     end  
endmodule

module Mux_1 #(parameter W=32)(input [W-1:0] in0,[W-1:0]in1, input sel, output [W-1:0]a);
    assign in1 = 32'd0;
    assign a = (sel) ? in1 : in0;
endmodule

module Mux_2 #(parameter W=32)(input [W-1:0]in0, [W-1:0]in1, input  sel, output [W-1:0]b);
    assign b = (sel) ? in1 : in0;
endmodule

module ALU
#(parameter W = 32) 
   (input [W-1:0] a,b ,input [3:0] s,
     output reg [W-1:0] f, output vflag, zflag
    );
   wire overflow, underflow , VFlag;
   reg ext; 
   
   initial 
   
   begin
   
        ext = 0;
   
   end 
    
 
assign overflow = ((s == 4'b0010 || s == 4'b0110 )&& {ext,f[W-1]} == 2'b01) ? (1) : (0);  
assign underflow = ((s == 4'b0110 ||s == 4'b0010 ) && {ext,f[W-1]} == 2'b10) ? (1) : (0);
assign vflag = ((overflow) || (underflow)) ? (1) : (0);  
assign zflag = (f == 0) ? 1:0;
    always@(*) 
    begin
        
        case(s)
            4'b0000: f <= a & b; //0 
            4'b0001: f <= a | b; //1
            4'b0010: {ext,f} <= {a[W-1], a} + {b[W-1], b};    //2, add Most sig bit
            4'b0110: {ext,f} <= {a[W-1], a} - {b[W-1], b};    //6, sub lsb
            4'b0111: f <= (a < b) ? 1 :0; //7
            4'b1100: f <= ~(a | b); //12
         endcase 
    end
endmodule 

module debounce(
input BTNU, clk, output pulse_out);

wire slow_clock, Q1_2D, Q2, Q3;

D_FF v1(slow_clock, BTNU, Q1_2D);
D_FF v2(slow_clock, Q1_2D, Q2);
D_FF v3(slow_clock, Q2, Q3);
counter v4(clk, slow_clock);
AND v5(Q3, Q2, pulse_out);

endmodule

module D_FF(
    input clk, D,
    output reg Q);
    
  initial
  begin
        Q = 0;
  end
   
  always @ (posedge clk)
  begin
    Q <= D;
  end
    
    
endmodule

module counter(input clk, output reg slow_clock);
 
reg [23:0] counter = 24'h000000;

initial
begin
    slow_clock = 0;
    counter = 24'd0;
end

always @(posedge clk)
    begin
        counter <= counter + 1;
       /// if (counter == 24'h3D0900)
       if (counter == 24'd1)
        begin
            slow_clock <= 1;
            counter <= 24'h000000;
       end
       else
        begin
            slow_clock <= 0;   
        end
    end

endmodule

module AND(input Q_FF3, Q_FF2, output Pulse_out);

    and(Pulse_out, ~Q_FF3, Q_FF2);

endmodule

module adder(input[31:0] inst1, inst2, output reg [31:0] sum);

    initial
    begin
    sum = 0;
    end
    
   always@(*)
   begin
   sum = inst1 + inst2; 
   end
   
endmodule

module shiftleft1(input [31:0] instruction, output [31:0] shifted1left);
    reg [31:0] inst;

    initial
    begin
    inst = instruction <<< 1;
    end
  
endmodule






