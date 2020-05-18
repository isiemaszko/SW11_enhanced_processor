module processor_2(input [0:8] DIN, input Resetn, Clock, Run, 
output reg Done, output[0:8] Addr, output[0:8] DOUT, output w);

localparam T0=3'b010, T1=3'b011, T2=3'b100, TL=3'b000, TPC=3'b001; //stany FSM
wire[0:2] I;
wire[0:7] Xreg, Yreg;
reg[0:7] Rin, Rout;
reg[0:1] Tstep_Q, Tstep_D;
reg ALU, Gin, IRin, Ain, Gout, DINout,Addrin,DOUTin,w_d, inpc;
wire [0:8] R0,R1; 
wire [0:8] A,G, R2,R3,R4,R5,R6,R7,pc;
reg[0:8] result;
wire [1:9] IR;
reg [0:9] BusWires; 

assign I=IR[1:3];

dec3to8(IR[4:6], 1'b1, Xreg);
dec3to8 (IR[7:9], 1'b1, Yreg);

//zarządzanie tabelą stanów FSM
always@(Tstep_Q,Run,Done)
begin
	case(Tstep_Q)
		TL:begin
			if(!Run) Tstep_D=TPC;
			else Tstep_D=TL;
			end
		TPC:begin
			Tstep_D=T0;
			end
		T0:
			begin
			if(Done) Tstep_D=TL;
			else Tstep_D=T1;
			end
		T1:
			begin 
			if(Done) Tstep_D=TL;
			else Tstep_D=T2;
			end
		T2:
			begin 
			Tstep_D=TL;
			end
		endcase
end
localparam mv=3'b000, mvt=3'b001, add=3'b010, sub=3'b011, ld=3'b100, st=3'b101, mvnz=3'b110;

//sterowanie wejściami FSM
always@(Tstep_Q or I or Xreg or Yreg)
begin
Done = 1'b0;Rin = 1'b0;Rout = 1'b0;ALU = 1'b0;Ain = 1'b0;Gin = 1'b0;
Gout = 1'b0;IRin = 1'b0;DINout = 1'b0; Addrin=1'b0; DOUTin=1'b0; w_d=1'b0; inpc=1'b0;
		
	case(Tstep_Q)
		TL:begin
			IRin=1'b1;
			Rout[7]=1'b1;
			Addrin=1'b1;
			end
		TPC:begin
			inpc=1'b1;
			end
		T0:
			begin
				case(I)
				mv:
				begin
					Rout = Yreg;
					Rin = Xreg;
					Done = 1'b1;
				end
				mvt:
				begin
					Rout[7] = 1'b1;
					inpc = 1'b1;
					Addrin = 1'b1;
					end
				add:
				begin
					Rout = Xreg;
					Ain = 1'b1;
					end
				sub:
				begin
					Rout = Xreg;
					Ain = 1'b1;
					end
				ld:
				begin
					Rout = Yreg;
					Addrin = 1'b1;
				end
				st:
				begin
					Rout = Yreg;
					Addrin = 1'b1;
				end
				mvnz:
				begin
					if(G!=9'b000000000)
					begin 
					Rout = Yreg;
					Rin = Xreg;
					end
					else
					Done= 1'b1;
				end
			endcase
			end
		T1:
			begin
			case(I)
				mvt:
				begin
					DINout = 1'b1;
					Rin = Xreg;
					end
				add:
				begin
					Rout = Yreg;
					Gin = 1'b1;
					end
				sub:
				begin
					Rout = Yreg;
					ALU = 1'b1;
					end
				ld:
				begin
					Rin = Xreg;
					DINout = 1'b1;
					Rin = 1'b1;
				end
				st:
				begin
					Rout = Xreg;
					DOUTin = 1'b1;
					w_d = 1'b1;
					Done = 1'b1;
				end
			endcase
			end
		T2:
			begin
			case(I)
				mvt:
				begin
					Rout[7] = 1'b1;
					Addrin = 1'b1;
					Done = 1'b1;
					end
				add:
				begin
					Rin = Xreg;
					Gout = 1'b1;
					Done=1'b1;
					end
				sub:
				begin
					Rin = Xreg;
					Gout = 1'b1;
					Done=1'b1;
					end
			endcase
			end
	endcase
end

always@(posedge Clock, negedge Resetn)
begin
	if(!Resetn) Tstep_Q=T0;
	else Tstep_Q=Tstep_D;
end

regn (BusWires, Rin[0], Clock, R0);
regn (BusWires, Rin[1], Clock, R1);
regn (BusWires, Rin[2], Clock, R2);
regn (BusWires, Rin[3], Clock, R3);
regn (BusWires, Rin[4], Clock, R4);
regn (BusWires, Rin[5], Clock, R5);
regn (BusWires, Rin[6], Clock, R6);
regn reg_IR(DIN, IRin, Clock, IR);
regn reg_A(BusWires, Ain, Clock, A);
regn reg_Addr(BusWires, Addrin, Clock, Addr);
regn reg_Dout(BusWires, DOUTin, Clock, DOUT);
regn reg_w(w_d, 1'b1, Clock, w);

counter_N_bits #(9)(Clock, Resetn, inpc, Rin[7], BusWires, pc);
always@(BusWires or A or ALU)
begin
	if(ALU)
		result = A - BusWires;
	else
		result = A + BusWires;
end

regn reg_G(result, Gin, Clock, G);

localparam Sel_D = 10'b1000000000, Sel_G = 10'b0100000000,Sel_R0 = 10'b0010000000,Sel_R1 = 10'b0001000000,
Sel_R2 = 10'b0000100000,Sel_R3 = 10'b0000010000,Sel_R4 = 10'b0000001000, Sel_R5 = 10'b0000000100,
Sel_R6 = 10'b0000000010, Sel_PC = 10'b0000000001;

assign sel = {DINout,Gout,Rout};

always @(sel or Rout or Gout or DINout) begin
	case(sel)
		Sel_R0: BusWires = R0;
			Sel_R1: BusWires = R1;
			Sel_R2: BusWires = R2;
			Sel_R3: BusWires = R3;
			Sel_R4: BusWires = R4;
			Sel_R5: BusWires = R5;
			Sel_R6: BusWires = R6;
			Sel_PC: BusWires = pc;
			Sel_G: BusWires = G;
			Sel_D: BusWires = DIN;	
	endcase
end
	
	
endmodule


module regn(D, Enable, Clock, Q);

parameter n = 9;

input [n-1:0] D;
input Enable;
input Clock;
output [n-1:0] Q;

reg [n-1:0] Q;

always @(posedge Clock)
begin
	if (Enable)
		Q <= D;
end
	
endmodule

module dec3to8(W, En, Y);

input [2:0] W;
input En;

output [7:0] Y;

reg [7:0] Y;

always @(W or En) begin
	if (En == 1)
		case (W)
				3'b000: Y = 8'b00000001;
				3'b001: Y = 8'b00000010;
				3'b010: Y = 8'b00000100;
				3'b011: Y = 8'b00001000;
				3'b100: Y = 8'b00010000;
				3'b101: Y = 8'b00100000;
				3'b110: Y = 8'b01000000;
				3'b111: Y = 8'b10000000;
		endcase
	else
		Y = 8'b00000000;
end

endmodule

module counter_N_bits
#(parameter N=4)
(input clk,aclr,enable,load,
input [N-1:0] data,
output reg [N-1:0] Q);
	always @(posedge clk, negedge aclr)
		if (!aclr) Q <= {N{1'b0}};
		else if (Q == {N{1'b1}}) Q <= {N{1'b0}};
		else if (load) Q <= data;
		else if (enable) Q <= Q + 1'b1;
		else Q <= Q;
endmodule
