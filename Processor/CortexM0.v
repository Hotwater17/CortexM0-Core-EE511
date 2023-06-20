
module CortexM0 (
	input	wire		      CLK,
	input	wire	        RESET_N, // reset when negative
	
	// For instruction memory
	output wire	    		IREQ,
	output wire [31:0]	IADDR,
	input	wire  [31:0]	INSTR,

	// For data memory
	output wire	    		DREQ,
	output wire	[31:0]	DADDR,
	output wire	  			DRW,
	output wire	[ 1:0]	DSIZE,
	input	wire	[31:0]	DIN,
	output wire	[31:0]	DOUT
);

REGFILE REGFILE (
  .CLK(CLK),
  .nRST(RESET_N),
  .WEN1(),
  .WA1(), 
  .DI1(), 
  .WEN2(),
  .WA2(),
  .DI2(),
  .RA0(id_rd_a_addr),
  .RA1(id_rd_b_addr),
  .RA2(id_rd_c_addr),
  .DOUT0(rf_reg_a_data),
  .DOUT1(rf_reg_b_data),
  .DOUT2(rf_reg_c_data)
);



// your code here

//################### PARAMS ###################

localparam  ALU_AND = 4'b0;
localparam  ALU_EOR = 4'b0001;
localparam  ALU_LSL = 4'b0010;
localparam  ALU_LSR = 4'b0011;
localparam  ALU_ASR = 4'b0100;
localparam  ALU_ADD = 4'b0101;
localparam  ALU_SUB = 4'b0110;
localparam  ALU_ROR = 4'b0111;

localparam  ALU_ORR = 4'b1100;
localparam  ALU_MUL = 4'b1101;
localparam  ALU_BIC = 4'b1110;
localparam  ALU_MVN = 4'b1111;


localparam ALU_A_SEL_REG = 2'b00;
localparam ALU_A_SEL_PC = 2'b01;
localparam ALU_B_SEL_REG = 2'b00;
localparam ALU_B_SEL_IMM = 2'b01;
localparam ALU_B_SEL_PC = 2'b10;

localparam ALU_NO_CARRY = 1'b0;
localparam ALU_USE_CARRY = 1'b1;

localparam  DEC_IMM = 6'b00xxxx;
  localparam  DEC_LSL_IMM = 5'b000xx;
  localparam  DEC_MOV_REG_T2 = 5'b00000;
  localparam  DEC_LSR_IMM = 5'b001xx;
  localparam  DEC_ASR_IMM = 5'b010xx;
  localparam  DEC_ADD_IREG = 5'b01100;
  localparam  DEC_SUB_IREG = 5'b01101;
  localparam  DEC_ADD_IMM3 = 5'b01110;
  localparam  DEC_SUB_IMM3 = 5'b01111;
  localparam  DEC_MOV_IMM = 5'b100xx;
  localparam  DEC_CMP_IMM = 5'b101xx;
  localparam  DEC_ADD_IMM8 = 5'b110xx;
  localparam  DEC_SUB_IMM8 = 5'b111xx;

localparam  DEC_DATA_PROC = 6'b010000;
  localparam  DEC_AND_REG = 4'b0;
  localparam  DEC_EOR_REG = 4'b0001;
  localparam  DEC_LSL_REG = 4'b0010;
  localparam  DEC_LSR_REG = 4'b0011;
  localparam  DEC_ASR_REG = 4'b0100;
  localparam  DEC_ADC_REG = 4'b0101;
  localparam  DEC_SBC_REG = 4'b0110;
  localparam  DEC_ROR_REG = 4'b0111;
  localparam  DEC_TST_REG = 4'b1000;
  localparam  DEC_RSB_REG = 4'b1001;
  localparam  DEC_CMP_REG = 4'b1010;
  localparam  DEC_CMN_REG = 4'b1011;
  localparam  DEC_ORR_REG = 4'b1100;
  localparam  DEC_MUL_REG = 4'b1101;
  localparam  DEC_BIC_REG = 4'b1110;
  localparam  DEC_MVN_REG = 4'b1111;

localparam  DEC_DS_BEX = 6'b010001;
  localparam DEC_ADD_REG = 4'b00xx;
  localparam DEC_CMP_1 = 4'b0101;
  localparam DEC_CMP_2 = 4'b011x;
  localparam DEC_MOV_REG = 4'b10xx;
  localparam DEC_BX = 4'b110x;
  localparam DEC_BLX = 4'b111x;

localparam DEC_LDR_LIT = 6'b01001x;

localparam  DEC_LS_1  = 6'b0101xx;
  localparam DEC_STR_REG = 3'b000;
  localparam DEC_STRH_REG = 3'b001;
  localparam DEC_STRB_REG = 3'b010;
  localparam DEC_LDRSB_REG = 3'b011;
  localparam DEC_LDR_REG = 3'b100;
  localparam DEC_LDRH_REG = 3'b101;
  localparam DEC_LDRB_REG = 3'b110;
  localparam DEC_LDRSH_REG = 3'b111;

localparam  DEC_LS_2  = 6'b011xxx;
  localparam DEC_STR_IMM = 4'b00xx;
  localparam DEC_LDR_IMM = 4'b01xx;
  localparam DEC_STRB_IMM = 4'b10xx;
  localparam DEC_LDRB_IMM = 4'b11xx;

localparam  DEC_LS_3  = 6'b100xxx;
  localparam DEC_STRH_IMM = 4'b00xx;
  localparam DEC_LDRH_IMM = 4'b01xx;
  localparam DEC_STR_SP = 4'b10xx;
  localparam DEC_LDR_SP = 4'b11xx;


localparam  DEC_PC_REL = 6'b10100x;

localparam  DEC_SP_REL = 6'b10101x;

localparam  DEC_MISC_16 = 6'b1011xx;
  localparam DEC_ADD_SPIMM = 7'b00000xx;
  localparam DEC_SUB_SPIMM = 7'b00001xx;
  localparam DEC_SXTH = 7'b001000x;
  localparam DEC_SXTB = 7'b001001x;
  localparam DEC_UXTH = 7'b001010x;
  localparam DEC_UXTB = 7'b001011x;
  localparam DEC_PUSH = 7'b010xxxx;
  localparam DEC_CPS = 7'b0110011; //UNUSED
  localparam DEC_REV = 7'b101000x;
  localparam DEC_REV16 = 7'b101001x;
  localparam DEC_REVSH = 7'b101011x;
  localparam DEC_POP = 7'b110xxxx;
  localparam DEC_BKPT = 7'b1110xxx; //UNUSED
  localparam DEC_HINT = 7'b1111xxx; //UNUSED

localparam  DEC_STM = 6'b11000x;

localparam  DEC_LDM = 6'b11001x;


localparam  DEC_B_COND = 6'b1101xx;
  localparam B_EQ = 4'b0000;
  localparam B_NE = 4'b0001;
  localparam B_CS = 4'b0010;
  localparam B_CC = 4'b0011;
  localparam B_MI = 4'b0100;
  localparam B_PL = 4'b0101;
  localparam B_VS = 4'b0110;
  localparam B_VC = 4'b0111;
  localparam B_HI = 4'b1000;
  localparam B_LS = 4'b1001;
  localparam B_GE = 4'b1010;
  localparam B_LT = 4'b1011;
  localparam B_GT = 4'b1100;
  localparam B_LE = 4'b1101;
  localparam B_AL = 4'b1110;


localparam  DEC_B_UC  = 6'b11100x;



localparam  DEC_32B_1  = 5'b11101;
localparam  DEC_32B_2  = 5'b11110;
  localparam DEC_BL = 3'b1x1;
localparam  DEC_32B_3  = 5'b11111; 

localparam SREG_NO_CHANGE = 1'b0;
localparam SREG_WRITE = 1'b1;

localparam BR_DIS = 1'b0;
localparam BR_EN = 1'b1;

localparam PC_SEL_NEXT= 2'b00;
localparam PC_SEL_BRANCH = 2'b01;
localparam PC_SEL_REG = 2'b10;

localparam WB_SEL_ALU = 1'b0;
localparam WB_SEL_MEM = 1'b1;

localparam WB_NO_WRITE = 1'b0;
localparam WB_WRITE = 1'b1;

localparam MEM_CSN_ACTIVE = 1'b0;
localparam MEM_CSN_INACTIVE = 1'b1;
localparam MEM_WE_READ = 1'b0;
localparam MEM_WE_WRITE = 1'b1;

localparam MEM_BE_BYTE = 4'b0001;
localparam MEM_BE_HALFWORD = 4'b0011;
localparam MEM_BE_WORD = 4'b1111;

localparam FWD_MUX_REG = 2'b00;
localparam FWD_MUX_EXE_MEM = 2'b10;
localparam FWD_MUX_MEM_WB = 2'b01;





//################### CONTROLS ###################

wire clk_i;
wire reset_i;
wire stall; 
wire flush; 

assign clk_i = CLK;
assign reset_i = RESET_N;
assign stall = (id_ex_memrd_i && ((id_ex_rd_i == if_id_rs1_i) || (id_ex_rd_i == if_id_rs2_i)));
assign flush = ((mem_pc_sel == PC_SEL_BRANCH) || (pc_sel == PC_SEL_REG)); 

ForwardUnit fwd(
  .exe_mem_rd_i(exe_wr_a_addr),
  .mem_wb_rd_i(mem_wr_a_addr),
  .id_exe_rs1_i(id_rd_a_addr),
  .id_exe_rs2_i(id_rd_b_addr),
  .exe_wb_a_en_i(exe_wr_a_en),
  .mem_wb_a_en_i(mem_wr_a_en),
  .fwd_mux_a_o(fwd_mux_a),
  .fwd_mux_b_o(fwd_mux_b)
);

//################### PIPELINE STAGES ###################

wire [31:0]       addr_branch;
wire [31:0]       addr_reg;
wire [1:0]        addr_sel;

wire [15:0]       if_instr_reg;
wire [15:0]       if_instr_noreg;
wire [31:0]       if_pc_addr;

wire [31:0]       rf_reg_a_data;
wire [31:0]       rf_reg_b_data;
wire [31:0]       rf_reg_c_data;   

wire [1:0]        fwd_mux_a;
wire [1:0]        fwd_mux_b;  

wire [3:0]        id_rd_a_addr;
wire [3:0]        id_rd_b_addr;
wire [3:0]        id_rd_c_addr;
wire [3:0]        id_wr_a_addr;
wire [3:0]        id_wr_b_addr;
wire              id_wr_a_en;
wire              id_wr_b_en;
wire [11:0]       id_imm;
wire [31:0]       id_reg_a_data;
wire [31:0]       id_reg_b_data;
wire [31:0]       id_reg_c_data;
wire []           id_pc_sel;
wire [31:0]       id_pc_addr;
wire [5:0]        id_alu_op;
wire [1:0]        id_alu_a_sel;
wire [1:0]        id_alu_b_sel;
wire              id_alu_with_carry
wire              id_mem_write;
wire              id_mem_cs;
wire [3:0]        id_mem_be;
wire              id_wb_sel;
wire              id_apsr_wr_en;
wire              id_Z;
wire              id_N;
wire              id_C;
wire              id_V;

wire              exe_rf_wr_a_en;
wire [3:0]        exe_rf_wr_a_addr;
wire              exe_rf_wr_b_en;
wire [3:0]        exe_rf_wr_b_addr;
wire [31:0]       exe_alu_result;
wire [1:0]        exe_pc_sel;
wire [31:0]       exe_reg_b_data;
wire              exe_wb_sel;
wire              exe_mem_write;
wire              exe_mem_cs;
wire [3:0]        exe_mem_be;
wire              exe_apsr_wr_en;
wire              exe_Z;
wire              exe_N;
wire              exe_C;
wire              exe_V;

wire              mem_rf_wr_a_en;
wire [3:0]        mem_rf_wr_a_addr;
wire              mem_rf_wr_b_en;
wire [3:0]        mem_rf_wr_b_addr;
wire [31:0]       mem_alu_result;
wire [31:0]       mem_rd_data;
wire [1:0]        mem_pc_sel;
wire              mem_wb_sel;
wire              mem_apsr_wr_en;
wire              mem_Z;
wire              mem_N;
wire              mem_C;
wire              mem_V;

wire [1:0]        wb_pc_sel;
wire [31:0]       wb_wr_data;
wire              wb_rf_wr_a_en;
wire [3:0]        wb_rf_wr_a_addr;
wire              wb_rf_wr_b_en;
wire [3:0]        wb_rf_wr_b_addr;
wire              wb_apsr_wr_en;
wire              wb_Z;
wire              wb_N;
wire              wb_C;
wire              wb_V;

assign mem_rd_data = DIN;
assign IADDR = 

InstructionFetchStage IF(
  .clk_i(clk_i),
  .reset_i(reset_i),
  .stall_i(stall),
  .flush_i(flush),
  .instr_mem_i(INSTR),
  .addr_branch_i(addr_branch),
  .addr_reg_i(addr_reg),
  .addr_sel_i(addr_sel),
  .instr_reg_o(if_instr_reg),   
  .instr_noreg_o(if_instr_noreg),
  .pc_addr_o(if_pc_addr),
  .imem_addr_o(IADDR);
  .imem_req_o(IREQ)
);

InstructionDecodeStage ID(
  //Connections to register file
  .clk_i(clk_i),
  .reset_i(reset_i),
  .stall_i(stall),
  .flush_i(flush),
  .instr_i(if_instr_reg),
  .instr_noreg_i(if_instr_noreg), 
  .pc_addr_i(if_pc_addr),
  .Z_new_i(wb_Z),
  .N_new_i(wb_N),
  .C_new_i(wb_C),
  .V_new_i(wb_V),
  .reg_a_data_i(rf_reg_a_data),
  .reg_b_data_i(rf_reg_b_data),
  .reg_c_data_i(rf_reg_c_data),
  .rf_rd_a_addr_o(id_rd_a_addr),
  .rf_rd_b_addr_o(id_rd_b_addr),
  .rf_rd_c_addr_o(id_rd_c_addr),
  .rf_wr_a_en_o(id_wr_a_en),
  .rf_wr_a_addr_o(id_wr_a_addr),
  .rf_wr_b_en_o(id_wr_b_en),
  .rf_wr_b_addr_o(id_wr_b_addr),
  .imm_o(id_imm),
  .reg_a_data_o(id_reg_a_data),
  .reg_b_data_o(id_reg_b_data),
  .reg_c_data_o(id_reg_c_data),
  .pc_sel_o(id_pc_sel),
  .pc_addr_o(id_pc_addr),
  .current_addr_o(),
  .alu_op_o(id_alu_op),
  .alu_a_sel_o(id_alu_a_sel),
  .alu_b_sel_o(id_alu_b_sel),
  .alu_with_carry_o(id_alu_with_carry),
  .mem_write_o(id_mem_write),
  .mem_cs_o(id_mem_cs),
  .mem_be_o(id_mem_be),
  .wb_sel_o(id_wb_sel),
  .apsr_wr_en_o(id_apsr_wr_en),
  .Z_o(id_Z),
  .N_o(id_N),
  .C_o(id_C),
  .V_o(id_V)
);


ExecuteStage EXE(
  .clk_i(clk_i),
  .reset_i(reset_i),
  .stall_i(stall),
  .flush_i(flush),
  .rf_rd_a_addr_i(id_rd_a_addr),
  .rf_rd_b_addr_i(id_rd_b_addr),
  .rf_rd_c_addr_i(id_rd_c_addr),
  .rf_wr_a_en_i(id_wr_a_en),
  .rf_wr_a_addr_i(id_wr_a_addr),
  .rf_wr_b_en_i(id_wr_b_en),
  .rf_wr_b_addr_i(id_wr_b_addr),
  .imm_i(id_imm),
  .reg_a_data_i(rf_reg_a_data),
  .reg_b_data_i(rf_reg_b_data),
  .reg_c_data_i(rf_reg_c_data),
  .pc_sel_i(id_pc_sel),
  .pc_addr_i(id_pc_addr),
  .current_addr_i(),
  .alu_op_i(id_alu_op),
  .alu_a_sel_i(id_alu_a_sel),
  .alu_b_sel_i(id_alu_b_sel),
  .mem_write_i(id_mem_write),
  .mem_cs_i(id_mem_cs),
  .mem_be_i(id_mem_be),
  .wb_sel_i(id_wb_sel),
  .apsr_wr_en_i(id_apsr_wr_en),
  .Z_i(id_Z),
  .N_i(id_N),
  .C_i(id_C),
  .V_i(id_V),
  .fwd_mux_a_i(fwd_mux_a),
  .fwd_mux_b_i(fwd_mux_a),
  .fwd_exe_mem_data_i(exe_alu_result),
  .fwd_mem_wb_data_i(mem_data),
  .rf_wr_a_en_o(exe_wr_a_en),
  .rf_wr_a_addr_o(exe_wr_a_addr),
  .rf_wr_b_en_o(exe_wr_b_en),
  .rf_wr_b_addr_o(exe_wr_b_addr),
  .alu_result_o(exe_alu_result),
  .pc_sel_o(exe_pc_sel),
  .reg_b_data_o(exe_reg_b_data),
  .wb_sel_o(exe_wb_sel),
  .mem_write_o(exe_mem_write),
  .mem_cs_o(exe_mem_cs),
  .mem_be_o(exe_mem_be),
  .apsr_wr_en_o(exe_apsr_wr_en),
  .Z_o(exe_Z),
  .N_o(exe_n),
  .C_o(exe_C),
  .V_o(exe_V) 

);





MemoryStage MEM(
  .clk_i(clk_i),
  .reset_i(reset_i),
  .stall_i(stall),
  .rf_wr_a_en_i(exe_wr_a_en),
  .rf_wr_a_addr_i(exe_wr_a_addr),
  .rf_wr_b_en_i(exe_wr_b_en),
  .rf_wr_b_addr_i(exe_wr_b_addr),
  .alu_result_i(exe_alu_result),
  .pc_sel_i(exe_pc_sel),
  .reg_b_data_i(exe_reg_b_data),
  .wb_sel_i(exe_wb_sel),
  .mem_write_i(exe_mem_write),
  .mem_cs_i(exe_mem_cs),
  .mem_be_i(exe_mem_be),
  .mem_rdata_i(DIN),
  .apsr_wr_en_i(exe_apsr_wr_en),
  .Z_i(exe_Z),
  .N_i(exe_N),
  .C_i(exe_C),
  .V_i(exe_V),
  .mem_write_o(DRW),
  .mem_cs_o(DREQ),
  .mem_be_o(DSIZE),
  .mem_wdata_o(DOUT),
  .mem_addr_o(DADDR),
  .rf_wr_a_en_o(mem_wr_a_en),
  .rf_wr_a_addr_o(mem_wr_a_addr),
  .rf_wr_b_en_o(mem_rf_wr_b_en),
  .rf_wr_b_addr_o(mem_rf_wr_b_addr),
  .alu_result_o(mem_alu_result),
  .pc_sel_o(mem_pc_sel),
  .wb_sel_o(mem_wb_sel),
  .apsr_wr_en_o(mem_apsr_wr_en),
  .Z_o(mem_Z),
  .N_o(mem_N),
  .C_o(mem_C),
  .V_o(mem_V) 
);

WriteBackStage WB(
  .clk_i(clk_i),
  .reset_i(reset_i),
  //.stall_i(stall),
  //.flush_i(),
  .rf_wr_a_en_i(mem_wr_a_en),
  .rf_wr_a_addr_i(mem_wr_a_addr),
  .rf_wr_b_en_i(mem_rf_wr_b_en),
  .rf_wr_b_addr_i(mem_rf_wr_b_addr),
  .mem_rdata_i(mem_rd_data),
  .alu_result_i(mem_alu_result),
  .pc_sel_i(mem_pc_sel),
  .wb_sel_i(mem_wb_sel),
  .apsr_wr_en_i(mem_apsr_wr_en),
  .Z_i(mem_Z),
  .N_i(mem_N),
  .C_i(mem_C),
  .V_i(mem_V),
  .pc_sel_o(wb_pc_sel),
  .wr_data_o(wb_wr_data),
  .rf_wr_a_en_o(wb_rf_wr_a_en),
  .rf_wr_a_addr_o(wb_rf_wr_a_addr),
  .rf_wr_b_en_o(wb_rf_wr_b_en),
  .rf_wr_b_addr_o(wb_rf_wr_b_addr),
  .apsr_wr_en_o(wb_apsr_wr_en),
  .Z_o(wb_Z),
  .N_o(wb_N),
  .C_o(wb_C),
  .V_o(wb_V) 
);

endmodule

// your code here (for other modules)



module InstructionFetchStage(

  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input [15:0]      instr_mem_i,
  input [31:0]      addr_branch_i,
  input [31:0]      addr_reg_i,
  input [1:0]       addr_sel_i,

  output reg [15:0] instr_reg_o,  //Latched instruction in pipeline register  
  output [15:0]     instr_noreg_o, //Non-latched instruction(straight from memory, next instr)
  output reg [31:0] pc_addr_o,
  output            imem_addr_o,
  output reg        imem_req_o
);

module ProgramCounter(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .new_addr_i(pc_mux_addr),
  .current_addr_o(pc_current_addr)

);

wire pc_mux_addr;
wire pc_current_addr;


assign instr_noreg_o = (~reset_i || flush_i) ? 16'b0 : instr_mem_i;
assign imem_req_o = (reset_i && ~flush_i)
assign imem_addr_o = pc_current_addr;

always @(*) begin
  case(addr_sel_i) 
    PC_SEL_NEXT : pc_mux_addr = pc_current_addr + 2;
    PC_SEL_BRANCH : pc_mux_addr = addr_branch_i;
    PC_SEL_REG : pc_mux_addr = addr_reg_i;
    default : pc_mux_addr = pc_current_addr + 2;
  endcase
end

always @(posedge clk_i or negedge reset_i or posedge flush_i)
begin
    if(~reset_i || flush_i) begin
      instr_reg_o <= 16'b0;
      pc_addr_o <= 32'b0;
    end
    else if(~stall_i) begin
      instr_reg_o <= instr_mem_i;
      pc_addr_o <= pc_current_addr;
    end
end


endmodule


module InstructionDecodeStage(
  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input  [15:0]     instr_i,      //Latched instruction in pipeline register 
  input  [15:0]     instr_noreg_i, //Non-latched instruction(straight from memory, next instr)
  input  [31:0]     pc_addr_i,
  input             Z_new_i,
  input             N_new_i,
  input             C_new_i,
  input             V_new_i,
  input [31:0]      reg_a_data_i,
  input [31:0]      reg_b_data_i,
  input [31:0]      reg_c_data_i,
 /*PUT IT INTO CORE
input         rf_wr_a_en_i,
input         rf_wr_a_addr_i,
input         rf_wr_a_data_i,
input         rf_wr_b_en_i,
input         rf_wr_b_addr_i,
input         rf_wr_b_data_i,
*/
  output reg [3:0]  rf_rd_a_addr_o,
  output reg [3:0]  rf_rd_b_addr_o,
  output reg [3:0]  rf_rd_c_addr_o,
  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [10:0] imm_o,
  output reg [31:0] reg_a_data_o,
  output reg [31:0] reg_b_data_o,
  output reg [31:0] reg_c_data_o,
  output reg [1:0]  pc_sel_o,
  output reg [31:0] pc_addr_o,
  output reg [31:0] current_addr_o,
  output reg [4:0]  alu_op_o,
  output reg [1:0]  alu_a_sel_o,
  output reg [1:0]  alu_b_sel_o,
  output reg        alu_with_carry_o,
  output reg        mem_write_o,
  output reg        mem_cs_o,
  output reg [3:0]  mem_be_o,
  output reg        wb_sel_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);

wire [3:0]  rf_rd_a_addr_w;
wire [3:0]  rf_rd_b_addr_w;
wire [3:0]  rf_rd_c_addr_w;
wire        rf_wr_a_en_w;
wire [3:0]  rf_wr_a_addr_w;
wire        rf_wr_b_en_w;
wire [3:0]  rf_wr_b_addr_w;
wire [10:0] imm_w;
wire [31:0] reg_a_data_w;
wire [31:0] reg_b_data_w;
wire [31:0] reg_c_data_w;
wire [1:0]  pc_sel_w;
wire [4:0]  alu_op_w;
wire [1:0]  alu_a_sel_w;
wire [1:0]  alu_b_sel_w;
wire        alu_with_carry_w;
wire        mem_write_w;
wire        mem_cs_w;
wire [3:0]  mem_be_w;
wire        wb_sel_w;
wire        apsr_wr_en_w;
wire        Z_w;
wire        N_w;
wire        C_w;
wire        V_w;



StatusRegister APSR(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .new_carry_i(C_new_i),
  .new_overflow_i(V_new_i),
  .new_negative_i(N_new_i),
  .new_zero_i(Z_new_i),
  .write_en_i(apsr_wr_en),
  .carry_o(C_w),
  .overflow_o(V_w),
  .negative_o(N_w),
  .zero_o(Z_w)
);

module Decoder(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .instr_i(instr_i),
  .instr_noreg_i(instr_noreg_i),
  .C_i(C_new_i), 
  .V_i(V_new_i),
  .N_i(N_new_i), 
  .Z_i(Z_new_i),
  .immediate_o(imm_w),
  .rd_a_addr_o(rf_rd_a_addr_w),
  .rd_b_addr_o(rf_rd_b_addr_w),
  .rd_c_addr_o(rf_rd_c_addr_w),
  .wr_a_addr_o(rf_wr_a_addr_w),
  .wr_b_addr_o(rf_wr_b_addr_w),
  .wr_a_en_o(rf_wr_a_en_w),
  .wr_b_en_o(rf_wr_b_en_w),
  .wr_wb_sel_o(wb_sel_w),
  .a_sel_o(alu_a_sel_w), //Reg or PC
  .b_sel_o(alu_b_sel_w), //Reg or Imm or PC
  .alu_op_sel_o(alu_op_w),
  .alu_with_carry_o(alu_with_carry_w),
  .sreg_we_o(apsr_wr_en_w),
  .pc_sel_o(pc_sel_w), //PC+4 or branch
  .dmem_csn_o(mem_cs_w),
  .dmem_wr_en_o(mem_write_w),
  .dmem_be_o(mem_be_w)/*,
  .imem_csn_o(imem),
  .imem_wr_en_o(),
  .imem_be_o()          */

);


always @(posedge clk_i or negedge reset_i or posedge flush_i)
begin
  if(~reset_i || flush_i) begin
    rf_rd_a_addr_o <= 4'b0;
    rf_rd_b_addr_o <= 4'b0;
    rf_rd_c_addr_o <= 4'b0;
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    imm_o <= 11'b0;
    reg_a_data_o <= 32'b0;
    reg_b_data_o <= 32'b0;
    reg_c_data_o <= 32'b0;
    pc_sel_o <= 2'b0;
    pc_addr_o <= 32'b0;
    current_addr_o <= 32'b0;
    alu_op_o <= 5'b0;
    alu_a_sel_o <= 2'b0;
    alu_b_sel_o <= 2'b0;
    alu_with_carry_o <= 1'b0;
    mem_write_o <= 1'b0;
    mem_cs_o <= 1'b0;
    mem_be_o <= 4'b0;
    wb_sel_o <= 1'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;

  end
  else if(~stall_i) begin

    rf_rd_a_addr_o <= rf_rd_a_addr_w;
    rf_rd_b_addr_o <= rf_rd_b_addr_w;
    rf_rd_c_addr_o <= rf_rd_c_addr_w;
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_b_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    imm_o <= imm_w;
    reg_a_data_o <= reg_a_data_i;
    reg_b_data_o <= reg_b_data_i;
    reg_c_data_o <= reg_c_data_i;
    pc_sel_o <= pc_sel_w;
    pc_addr_o <= pc_addr_i;
    
    //current_addr_o <= 

    alu_op_o <= alu_op_w;
    alu_a_sel_o <= alu_a_sel_w;
    alu_b_sel_o <= alu_b_sel_w;
    alu_with_carry_o <= alu_with_carry_w;
    mem_write_o <= mem_write_w;
    mem_cs_o <= mem_cs_w; 
    mem_be_o <= mem_be_w;
    wb_sel_o <= wb_sel_w;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule

module ExecuteStage(
  
  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input [3:0]       rf_rd_a_addr_i, //Not sure if needed here
  input [3:0]       rf_rd_b_addr_i, //Not sure if needed here
  input [3:0]       rf_rd_c_addr_i, //Not sure if needed here
  input             rf_wr_a_en_i,
  input [3:0]       rf_wr_a_addr_i,
  input             rf_wr_b_en_i,
  input [3:0]       rf_wr_b_addr_i,
  input [10:0]      imm_i,
  input [31:0]      reg_a_data_i,
  input [31:0]      reg_b_data_i,
  input [31:0]      reg_c_data_i,
  input [1:0]       pc_sel_i,
  input [31:0]      pc_addr_i,
  input [31:0]      current_addr_i,
  input [4:0]       alu_op_i,
  input [1:0]       alu_a_sel_i,
  input [1:0]       alu_b_sel_i,
  input             mem_write_i,
  input             mem_cs_i,
  input [3:0]       mem_be_i,
  input             wb_sel_i,
  input             apsr_wr_en_i,
  input             Z_i,
  input             N_i,
  input             C_i,
  input             V_i,
  input [1:0]       fwd_mux_a_i,
  input [1:0]       fwd_mux_b_i,
  input [31:0]      fwd_exe_mem_data_i,
  input [31:0]      fwd_mem_wb_data_i,

  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [31:0] alu_result_o,
  output reg [1:0]  pc_sel_o,
  output reg [31:0] reg_b_data_o,
  output reg        wb_sel_o,
  output reg        mem_write_o,
  output reg        mem_cs_o,
  output reg [3:0]  mem_be_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);

  wire        rf_wr_a_en_w;
  wire [3:0]  rf_wr_a_addr_w
  wire        rf_wr_b_en_w;
  wire [3:0]  rf_wr_b_addr_w;
  wire [31:0] alu_result_w;
  wire [1:0]  pc_sel_w;
  wire [31:0] reg_b_data_w;
  wire        wb_sel_w;
  wire        mem_write_w;
  wire        mem_cs_w;
  wire [3:0]  mem_be_w;
  wire        apsr_wr_en_w;
  wire        Z_w;
  wire        N_w;
  wire        C_w;
  wire        V_w;

  wire [31:0] alu_a_w;
  wire [31:0] alu_b_w;

  wire [31:0] fwd_mux_a_data;
  wire [31:0] fwd_mux_b_data;

  assign rf_wr_a_en_w =  rf_wr_a_en_i;
  assign rf_wr_a_addr_w = rf_wr_a_addr_i;
  assign rf_wr_b_en_w = rf_wr_b_en_i;
  assign rf_wr_b_addr_w = rf_wr_b_addr_i;
  assign pc_sel_w = pc_sel_i;
  assign reg_b_data_w = reg_b_data_i;
  assign wb_sel_w = wb_sel_i;
  assign mem_write_w = mem_write_i; 
  assign mem_cs_w = mem_cs_i;
  assign mem_be_w = mem_be_i;
  assign apsr_wr_en_w = apsr_wr_en_i;





  always @(*) begin

    case(fwd_mux_a_i)
      FWD_MUX_REG : fwd_mux_a_data = reg_a_data_i;
      FWD_MUX_EXE_MEM : fwd_mux_a_data = fwd_exe_mem_data_i;
      FWD_MUX_MEM_WB : fwd_mux_a_data = fwd_mem_wb_data_i;
      default : fwd_mux_a_data = 32'b0;
    endcase

    case(fwd_mux_b_i) 
      FWD_MUX_REG : fwd_mux_b_data = reg_b_data_i;
      FWD_MUX_EXE_MEM : fwd_mux_b_data = fwd_exe_mem_data_i;
      FWD_MUX_MEM_WB : fwd_mux_b_data = fwd_mem_wb_data_i;
      default : fwd_mux_b_data = 32'b0;
    endcase

    if(alu_a_sel_i == ALU_A_SEL_REG) alu_a_w = fwd_mux_a_data;
    else alu_a_w = pc_addr_i;

    case(alu_b_sel_i)
      ALU_B_SEL_REG : alu_b_w = fwd_mux_b_data;
      ALU_B_SEL_IMM : alu_b_w = imm_i;
      ALU_B_SEL_PC : alu_b_w = pc_addr_i;
      default : 32'b0;
    endcase  
  end

  ArithmeticLogicUnit ALU(
  .alu_op_i(alu_op_i),
  .a_i(alu_a_w),
  .b_i(alu_b_w),
  .carry_use_i(carry_use_i),
  .carry_i(C_i),
  .overflow_i(V_i),
  .negative_i(N_i),
  .zero_i(Z_i),
  .result_o(alu_result_w),
  .carry_o(C_w),
  .overflow_o(V_w),
  .negative_o(N_w),
  .zero_o(Z_w)
);

always @(posedge clk_i or negedge reset_i or posedge flush_i) begin
  if(~reset_i || flush_i) begin
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    alu_result_o <= 32'b0;
    pc_sel_o <= 2'b0;
    reg_b_data_o <= 32'b0;
    wb_sel_o <= 1'b0;
    mem_write_o <= 1'b0;
    mem_cs_o <= 1'b0;
    mem_be_o <= 4'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;
  end
  else if(~stall) begin
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_a_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    alu_result_o <= alu_result_w;
    pc_sel_o <= pc_sel_w;
    reg_b_data_o <= reg_b_data_w;
    wb_sel_o <= wb_sel_w;
    mem_write_o <= mem_write_w;
    mem_cs_o <= mem_cs_w;
    mem_be_o <= mem_be_w;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule


module MemoryStage(

  input             clk_i,
  input             reset_i,
  input             stall_i,
  //input           flush_i,

  input             rf_wr_a_en_i,
  input [3:0]       rf_wr_a_addr_i,
  input             rf_wr_b_en_i,
  input [3:0]       rf_wr_b_addr_i,
  input [31:0]      alu_result_i,
  input [1:0]       pc_sel_i,
  input [31:0]      reg_b_data_i,
  input             wb_sel_i,
  input             mem_write_i,
  input             mem_cs_i,
  input [3:0]       mem_be_i,
  input [31:0]      mem_rdata_i,
  input             apsr_wr_en_i,
  input             Z_i,
  input             N_i,
  input             C_i,
  input             V_i,

  output            mem_write_o,
  output            mem_cs_o,
  output            mem_be_o,
  output reg [31:0] mem_wdata_o,
  output reg [31:0] mem_addr_o,
  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [31:0] alu_result_o,
  output reg [1:0]  pc_sel_o,
  output reg        wb_sel_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);

  wire [31:0] mem_wdata_w;
  wire [31:0] mem_addr_w;
  wire        rf_wr_a_en_w;
  wire [3:0]  rf_wr_a_addr_w;
  wire        rf_wr_b_en_w;
  wire [3:0]  rf_wr_b_addr_w;
  wire [31:0] alu_result_w;
  wire [1:0]  pc_sel_w;
  wire        wb_sel_w;
  wire        apsr_wr_en_w;
  wire        Z_w;
  wire        N_w;
  wire        C_w;
  wire        V_w;


  assign mem_write_o = mem_write_i;
  assign mem_cs_o = mem_cs_i;
  assign mem_be_o = mem_be_i;
  assign mem_wdata_o = alu_result_i;
  assign mem_addr_o = reg_b_data_i;
  assign rf_wr_a_en_w = rf_wr_a_en_i;
  assign rf_wr_a_addr_w = rf_wr_a_addr_i;
  assign rf_wr_b_en_w = rf_wr_b_en_i;
  assign rf_wr_b_addr_w = rf_wr_b_addr_i;
  assign alu_result_w = alu_result_i;
  assign pc_sel_w = pc_sel_i;
  assign wb_sel_w = wb_sel_i;
  assign apsr_wr_en_w = apsr_wr_en_i;
  assign Z_w = Z_i;
  assign N_w = N_i;
  assign C_w = C_i;
  assign V_w = V_i;

always @(posedge clk_i or negedge reset_i) begin
  if(~reset) begin
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    alu_result_o <= 32'b0;
    pc_sel_o <= 2'b0;
    wb_sel_o <= 1'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;
  end
  else if(~stall_i) begin
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_a_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    alu_result_o <= alu_result_w;
    pc_sel_o <= pc_sel_w;
    wb_sel_o <= wb_sel_w;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule


module WriteBackStage(

  input         clk_i,
  input         reset_i,
  //input         stall_i,
  //input         flush_i,

  input         rf_wr_a_en_i,
  input [3:0]   rf_wr_a_addr_i,
  input         rf_wr_b_en_i,
  input [3:0]   rf_wr_b_addr_i,
  input [31:0]  mem_rdata_i,
  input [31:0]  alu_result_i,
  input [1:0]   pc_sel_i,
  input         wb_sel_i,
  input         apsr_wr_en_i,
  input         Z_i,
  input         N_i,
  input         C_i,
  input         V_i,

  output [1:0]  pc_sel_o,
  output [31:0] wr_data_o,
  output        rf_wr_a_en_o,
  output [3:0]  rf_wr_a_addr_o,
  output        rf_wr_b_en_o,
  output [3:0]  rf_wr_b_addr_o,
  output        apsr_wr_en_o,
  output        Z_o,
  output        N_o,
  output        C_o,
  output        V_o
);
  assign pc_sel_o = pc_sel_i;
  assign wr_data_o = (wb_sel_i == WB_SEL_MEM) ? mem_rdata_i : alu_result_i;
  assign rf_wr_a_en_o = rf_wr_a_en_i;
  assign rf_wr_a_addr_o = rf_wr_a_addr_i;
  assign rf_wr_b_en_o = rf_wr_b_en_i;
  assign rf_wr_b_addr_o = rf_wr_b_addr_i;
  assign apsr_wr_en_o = apsr_wr_en_i;
  assign Z_o = Z_i;
  assign N_o = N_i;
  assign C_o = C_i; 
  assign V_o = V_i;

endmodule


module ForwardUnit(

input   [3:0]   exe_mem_rd_i,
input   [3:0]   mem_wb_rd_i,
input   [3:0]   id_exe_rs1_i,
input   [3:0]   id_exe_rs2_i,

input           exe_wb_a_en_i,
input           mem_wb_a_en_i,

output  [1:0]   fwd_mux_a_o,
output  [1:0]   fwd_mux_b_o

);


always @(*) begin
  if((ex_wb_a_en_i == WB_WRITE) && (ex_mem_rd_i == id_ex_rs1_i)) fwd_mux_a_o = FWD_MUX_EXE_MEM;
  else if((mem_wb_a_en_i == WB_WRITE) && !(ex_wb_a_en_i && (ex_mem_rd_i == id_ex_rs1_i)) && (mem_wb_rd_i == id_ex_rs1_i)) fwd_mux_a_o = FWD_MUX_MEM_WB;
  else fwd_mux_a_o = FWD_MUX_REG;


  if((ex_wb_a_en_i == WB_WRITE) && (ex_mem_rd_i == id_ex_rs2_i)) fwd_mux_b_o = FWD_MUX_EXE_MEM;
  else if((mem_wb_a_en_i == WB_WRITE) && !(ex_wb_a_en_i && (ex_mem_rd_i == id_ex_rs2_i)) && (mem_wb_rd_i == id_ex_rs2_i)) fwd_mux_b_o = FWD_MUX_MEM_WB;
  else fwd_mux_b_o = FWD_MUX_REG;
end

endmodule


module ProgramCounter(

  //Common inputs
  input           clk_i,
  input           reset_i,
  //Address ports
  input   [31:0]  new_addr_i,
  output reg [31:0]  current_addr_o

);

reg [31:0]  pc_addr;

always @(posedge clk_i or negedge reset_i) begin
  if(!reset_i) pc_addr <= 32'h0;
  else pc_addr <= new_addr_i;
end

assign current_addr_o = pc_addr;

endmodule


module StatusRegister(

  //Common inputs
  input   clk_i,
  input   reset_i,

  //New flag values
  input   new_carry_i,
  input   new_overflow_i,
  input   new_negative_i,
  input   new_zero_i,

  input   write_en_i,

  //Flag register outputs
  output reg carry_o,
  output reg overflow_o,
  output reg negative_o,
  output reg zero_o
);

reg   C, V, N, Z;

always @(posedge clk or negedge reset_i) begin
  if(!reset_i) C <= 1'b0; V <= 1'b0; N <= 1'b0; Z <= 1'b0;
  else if(write_en_i) C <= new_carry_i; V <= new_overflow_i; N <= new_negative_i; Z <= new_zero_i;
end

assign carry_o = C;
assign overflow_o = V;
assign negative_o = N;
assign zero_o = Z;

endmodule

module Decoder(

  //Common inputs
  input           clk_i,
  input           reset_i,

  //Instruction  
  input   [15:0]  instr_i,
  input   [15:0]  instr_noreg_i,
  //Flags
  input           C_i, 
  input           V_i,
  input           N_i, 
  input           Z_i,

  output  [31:0]  immediate_o,

  //Register file control signals
  output [3:0]    rd_a_addr_o,
  output [3:0]    rd_b_addr_o,
  output [3:0]    rd_c_addr_o,
  output [3:0]    wr_a_addr_o,
  output [3:0]    wr_b_addr_o,
  output          wr_a_en_o,
  output          wr_b_en_o,
  output          wr_wb_sel_o,

  //ALU control signals
  output  [1:0]   a_sel_o, //Reg or PC
  output  [1:0]   b_sel_o, //Reg or Imm or PC 
  output  [3:0]   alu_op_sel_o,
  output          alu_with_carry_o,

  //Status register control signals
  output          sreg_we_o,
  //PC control signals
  output  [1:0]   pc_sel_o, //PC+4 or branch

  //Data memory control signals(SRAM PORT 1)
  output          dmem_csn_o,
  output          dmem_wr_en_o,
  output  [3:0]   dmem_be_o,

  //Instruction memory control signals (SRAM PORT 2)
  output          imem_csn_o,
  output          imem_wr_en_o,
  output  [3:0]   imem_be_o          

);
  always @(*) begin
    case(instr_i[15:10])

      DEC_IMM : begin
        //immediate_o = 32'b0;
        //rd_a_addr_o = {1'b0, instr_i[5:3]};
        //rd_b_addr_o = {1'b0, instr_i[2:0]};
        rd_c_addr_o = 4'b0;
        //wr_a_addr_o = {1'b0, instr_i[2:0]};
        //wr_a_en_o = WB_WRITE;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = WB_NO_WRITE; 
        wr_wb_sel_o = WB_SEL_ALU;
        a_sel_o = ALU_A_SEL_REG;
        //b_sel_o = ALU_B_SEL_IMM;
        //alu_op_sel_o = instr_i[9:6]; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_INACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;   

        case(instr_i[13:11])
          DEC_LSL_IMM : begin
            immediate_o = instr_i[10:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_LSL;
          end

          DEC_MOV_REG_T2 : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD;           
          end

          DEC_LSR_IMM : begin
            immediate_o = instr_i[10:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_LSR;
          end
          DEC_ASR_IMM : begin
            immediate_o = instr_i[10:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ASR;
          end
          DEC_ADD_IREG : begin
            immediate_o = 32'b0;
            rd_a_addr_o = instr_i[8:6];
            rd_b_addr_o = instr_i[5:3];
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_REG;
            alu_op_sel_o = ALU_ADD;
          end
          DEC_SUB_IREG : begin
            immediate_o = 32'b0;
            rd_a_addr_o = instr_i[8:6];
            rd_b_addr_o = instr_i[5:3];
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_REG;
            alu_op_sel_o = ALU_SUB;
          end
          DEC_ADD_IMM3 : begin
            immediate_o = instr_i[8:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD;
          end
          DEC_SUB_IMM3 : begin
            immediate_o = instr_i[8:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_SUB;
          end
          DEC_MOV_IMM : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            //This needs a NOP in ALU - just move
            wr_a_addr_o = instr_i[10:8];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            //alu_op_sel_o = ALU_;
          end
          DEC_CMP_IMM : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = instr_i[10:8];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = WB_NO_WRITE;
            b_sel_o = ALU_B_SEL_IMM;    
            alu_op_sel_o = ALU_SUB;        
          end
          DEC_ADD_IMM8 : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = instr_i[10:8];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[10:8];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD; 
          end
          DEC_SUB_IMM8 : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = instr_i[10:8];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[10:8];
            wr_a_en_o = WB_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_SUB; 
          end
          default : begin
            immediate_o = 32'b0;
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = WB_NO_WRITE;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD;            
          end 
        endcase
      end

      DEC_DATA_PROC : begin

        immediate_o = 32'b0;
        rd_a_addr_o = {1'b0, instr_i[5:3]};
        rd_b_addr_o = {1'b0, instr_i[2:0]};
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[2:0]};
        wr_a_en_o = WB_WRITE;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = WB_NO_WRITE; 
        wr_wb_sel_o = WB_SEL_ALU;
        a_sel_o = ALU_A_SEL_REG;
        b_sel_o = ALU_B_SEL_REG;
        alu_op_sel_o = instr_i[9:6]; //NOT COMPLETE, (WHY?) 
        alu_with_carry_o = ALU_USE_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_INACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;        


      end

      DEC_DS_BEX : begin

        immediate_o = 32'b0;
        wr_wb_sel_o = WB_SEL_ALU;
        sreg_we_o = SREG_WRITE;
        dmem_csn_o = MEM_CSN_INACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;   

        case(instr_i[9:6])

          DEC_ADD_REG : begin
            rd_a_addr_o = {instr_i[7] instr_i[2:0]}
            rd_b_addr_o = instr_i[6:3];
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7] instr_i[2:0]};
            wr_a_en_o = WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = (rd_a_addr_o == 15) ? ALU_A_SEL_PC : ALU_A_SEL_REG;
            b_sel_o = (rd_b_addr_o == 15) ? ALU_B_SEL_PC : ALU_B_SEL_REG;        
            alu_op_sel_o = ALU_ADD;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = (wr_a_addr_o == 15) ? PC_SEL_BRANCH : PC_SEL_NEXT;
          end

          DEC_CMP_1 : begin
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = instr_i[2:0];
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = WB_NO_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_REG;        
            alu_op_sel_o = ALU_SUB;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_NEXT;        
          end

          DEC_CMP_2 : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = {instr_i[7] instr_i[2:0]};
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            wr_a_en_o = WB_NO_WRITE;
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_REG;        
            alu_op_sel_o = ALU_SUB;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_NEXT;         
          end

          DEC_MOV_REG : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7] instr_i[2:0]};
            wr_a_en_o = WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_IMM;        
            alu_op_sel_o = ALU_ADD;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = (wr_addr_a_o == 15) ? PC_SEL_BRANCH : PC_SEL_NEXT;            
          end

          DEC_BX : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7] instr_i[2:0]};
            wr_a_en_o = WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_IMM;        
            alu_op_sel_o = ALU_ADD;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_BRANCH;           
          end

          DEC_BLX : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7] instr_i[2:0]};
            wr_a_en_o = WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE;            
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_IMM;        
            alu_op_sel_o = ALU_ADD;
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_REG;          
          end

        endcase
        
      end

      DEC_LDR_LIT : begin
        immediate_o = instr_i[7:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = WB_WRITE;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = WB_NO_WRITE;
        wr_wb_sel_o = WB_SEL_MEM; //Write data from memory
        a_sel_o = ALU_A_SEL_PC;
        b_sel_o = ALU_B_SEL_IMM;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;          
      end

      DEC_LS_1 : begin
        case(instr_i[11:9])

        
        rd_a_addr_o = instr_i[8:6];
        rd_b_addr_o = instr_i[5:3];
        rd_c_addr_o = instr_i[2:0];
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_wb_sel_o = WB_SEL_MEM;

        immediate_o = 32'b0;
        a_sel_o = ALU_A_SEL_REG;
        b_sel_o = ALU_B_SEL_REG;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_ACTIVE;
        imem_csn_o = MEM_CSN_ACTIVE;
/*
        wr_a_en_o = WB_WRITE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
*/
          
          DEC_STR_REG : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_WORD;           
          end
          DEC_STRH_REG : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_HALFWORD;              
          end
          DEC_STRB_REG : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_BYTE;               
          end
          DEC_LDRSB_REG : begin
            //Signed
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_BYTE;             
          end
          DEC_LDR_REG : begin
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;              
          end
          DEC_LDRH_REG : begin
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;           
          end
          DEC_LDRB_REG : begin
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_BYTE;              
          end
          DEC_LDRSH_REG : begin
            //Signed
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;  
          end

        endcase
      end

      DEC_LS_2 : begin
        rd_a_addr_o = instr_i[5:3]
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_wb_sel_o = WB_SEL_MEM;
        wr_b_en_o = WB_NO_WRITE; 

        immediate_o = {27{1'b0}, instr_i[10:6]};
        a_sel_o = ALU_A_SEL_REG;
        b_sel_o = ALU_B_SEL_IMM;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_ACTIVE;
        imem_csn_o = MEM_CSN_ACTIVE;
/*
        wr_a_en_o = WB_WRITE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
*/
        case(instr_i[11:9])
          DEC_STR_IMM : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_WORD;            
          end
          DEC_LDR_IMM : begin
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;            
          end
          DEC_STRB_IMM : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_BYTE;              
          end
          DEC_LDRB_IMM : begin
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_BYTE;              
          end
          default : begin
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;              
          end
        endcase
      end

      DEC_LS_3 : begin
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_wb_sel_o = WB_SEL_MEM;
        wr_b_en_o = WB_NO_WRITE;
        
        a_sel_o = ALU_A_SEL_REG;
        b_sel_o = ALU_B_SEL_IMM;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_WRITE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_ACTIVE;
        imem_csn_o = MEM_CSN_ACTIVE;
/*
        rd_a_addr_o = instr_i[5:3]
        immediate_o = {27{1'b0}, instr_i[10:6]};
        wr_a_en_o = WB_WRITE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
*/        
        case(instr_i[11:9])
          DEC_STRH_IMM : begin
            rd_a_addr_o = instr_i[5:3];
            immediate_o = {27{1'b0}, instr_i[10:6]};
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_HALFWORD;             
          end
          DEC_LDRH_IMM : begin
            rd_a_addr_o = instr_i[5:3];
            immediate_o = {27{1'b0}, instr_i[10:6]};
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;              
          end
          DEC_STR_SP : begin
            rd_a_addr_o = 4'b1101; //R13 - SP
            immediate_o = {24{1'b0}, instr_i[7:0]};
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_WRITE;
            dmem_be_o = MEM_BE_HALFWORD;               
          end
          DEC_LDR_SP : begin
            rd_a_addr_o = 4'b1101; //R13 - SP
            immediate_o = {24{1'b0}, instr_i[7:0]};
            wr_a_en_o = WB_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;             
          end
          default : begin
            rd_a_addr_o = 4'b0;
            immediate_o = 32'b0
            wr_a_en_o = WB_NO_WRITE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;                   
          end
        endcase
      end

      DEC_PC_REL : begin
        immediate_o = instr_i[7:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = WB_WRITE;
        wr_wb_sel_o = WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = WB_NO_WRITE; 
        a_sel_o = ALU_A_SEL_PC;
        b_sel_o = ALU_B_SEL_IMM;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_NO_CHANGE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_INACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;          
      end

      DEC_SP_REL : begin
        immediate_o = instr_i[7:0];
        rd_a_addr_o = 4'b1101; //R13 - SP 
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = WB_WRITE;
        wr_wb_sel_o = WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = WB_NO_WRITE; 
        a_sel_o = ALU_A_SEL_REG;
        b_sel_o = ALU_B_SEL_IMM;
        alu_op_sel_o = ALU_ADD; 
        alu_with_carry_o = ALU_NO_CARRY;
        sreg_we_o = SREG_NO_CHANGE;
        pc_sel_o = PC_SEL_NEXT;
        dmem_csn_o = MEM_CSN_INACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_WORD;
        imem_csn_o = MEM_CSN_ACTIVE;
        dmem_wr_en_o = MEM_WE_READ;
        dmem_be_o = MEM_BE_HALFWORD;           
      end

      DEC_MISC_16 : begin
        /*
localparam  DEC_MISC_16 = 6'b1011xx;
  localparam DEC_ADD_SPIMM = 7'b00000xx;
  localparam DEC_SUB_SPIMM = 7'b00001xx;
  localparam DEC_SXTH = 7'b001000x;
  localparam DEC_SXTB = 7'b001001x;
  localparam DEC_UXTH = 7'b001010x;
  localparam DEC_UXTB = 7'b001011x;
  localparam DEC_PUSH = 7'b010xxxx;
  localparam DEC_CPS = 7'b0110011; //UNUSED
  localparam DEC_REV = 7'b101000x;
  localparam DEC_REV16 = 7'b101001x;
  localparam DEC_REVSH = 7'b101011x;
  localparam DEC_POP = 7'b110xxxx;
  localparam DEC_BKPT = 7'b1110xxx; //UNUSED
  localparam DEC_HINT = 7'b1111xxx; //UNUSED    
  */    
        case(instr_i[11:5])
          DEC_ADD_SPIMM : begin
            immediate_o = instr_i[6:0];
            rd_a_addr_o = 4'b1101; //R13 - SP 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b1101;;
            wr_a_en_o = WB_WRITE;
            wr_wb_sel_o = WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD; 
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_NEXT;
            dmem_csn_o = MEM_CSN_INACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;
            imem_csn_o = MEM_CSN_ACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;    
          end
          DEC_SUB_SPIMM : begin
            immediate_o = instr_i[6:0];
            rd_a_addr_o = 4'b1101; //R13 - SP 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b1101;;
            wr_a_en_o = WB_WRITE;
            wr_wb_sel_o = WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_REG;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_SUB; 
            alu_with_carry_o = ALU_NO_CARRY;
      
            pc_sel_o = PC_SEL_NEXT;
            dmem_csn_o = MEM_CSN_INACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;
            imem_csn_o = MEM_CSN_ACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;   
          end


        endcase

      end

      DEC_STM : begin
        
      end

      DEC_LDM : begin
        
      end

      DEC_B_COND : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = WB_NO_WRITE;
            wr_wb_sel_o = WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_PC;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD; 
            alu_with_carry_o = ALU_NO_CARRY;
            sreg_we_o = SREG_NO_CHANGE;
            dmem_csn_o = MEM_CSN_INACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;
            imem_csn_o = MEM_CSN_ACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;

            case(instr_i[11:8])
              B_EQ : pc_sel_o = (Z_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;            
              B_NE : pc_sel_o = (~Z_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_CS : pc_sel_o = (C_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_CC : pc_sel_o = (~C_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_MI : pc_sel_o = (N_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_PL : pc_sel_o = (~N_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_VS : pc_sel_o = (V_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_VC : pc_sel_o = (~V_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_HI : pc_sel_o = (C_i && ~Z_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_LS : pc_sel_o = (~C_i || Z_i) ? PC_SEL_BRANCH : PC_SEL_NEXT; 
              B_GE : pc_sel_o = (N_i == V_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_LT : pc_sel_o = (N_i != V_i) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_GT : pc_sel_o = (~Z_i && (N_i == V_i)) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_LE : pc_sel_o = (Z_i || (N_i != V_i)) ? PC_SEL_BRANCH : PC_SEL_NEXT;
              B_AL : pc_sel_o = PC_SEL_BRANCH; 
              default : pc_sel_o = PC_SEL_NEXT;
            endcase           
      end

      DEC_B_UC : begin
            immediate_o = instr_i[10:0];
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = WB_NO_WRITE;
            wr_wb_sel_o = WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = WB_NO_WRITE; 
            a_sel_o = ALU_A_SEL_PC;
            b_sel_o = ALU_B_SEL_IMM;
            alu_op_sel_o = ALU_ADD; 
            alu_with_carry_o = ALU_NO_CARRY;
            sreg_we_o = SREG_NO_CHANGE;
            pc_sel_o = PC_SEL_BRANCH;
            dmem_csn_o = MEM_CSN_INACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_WORD;
            imem_csn_o = MEM_CSN_ACTIVE;
            dmem_wr_en_o = MEM_WE_READ;
            dmem_be_o = MEM_BE_HALFWORD;        
      end

      DEC_32B_2 : begin
          //BL
          //instr_noreg
      end

    endcase
  end




endmodule

module ArithmeticLogicUnit(
  
  input   [3:0]   alu_op_i,
  input           a_i,
  input           b_i,
  input           carry_use_i,

  //Flags in
  input           carry_i,
  input           overflow_i,
  input           negative_i,
  input           zero_i,

  //Result and flags out
  output          result_o,
  output          carry_o,
  output          overflow_o,
  output          negative_o,
  output          zero_o

);



always @(*) begin
    
    case(alu_op_i)

      ALU_AND : begin
        result_o = a_i & b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end

      ALU_EOR : begin
        result_o = a_i ^ b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;        
      end

      ALU_LSL : begin
        {carry_o, result_o} = a_i << b_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;    
      end

      ALU_LSR : begin
        result_o = a_i >> b_i;
        carry_o =  a_i[b_i-1];
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;         
      end

      ALU_ASR : begin
        result_o = a_i >>> b_i;
        carry_o =  a_i[b_i-1]; 
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;          
      end

      ALU_ADD : begin
        {carry_o, result_o} = a_i + b_i + (carry_i & carry_use_i);
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end

      ALU_SUB : begin
        {carry_o, result_o} = a_i - b_i - (carry_i & carry_use_i); //carry 
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;          
      end

      ALU_ROR : begin
        result_o = (a_i << b_i[4:0])| (a_i >> 32-(b_i[4:0]));
        carry_o =  result_o[31];
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;        
      end

      ALU_ORR : begin
        result_o = a_i | b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;       
      end

      ALU_MUL : begin
        result_o = a_i * b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;   
      end

      ALU_BIC : begin
        result_o = a_i & ~b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;         
      end

      ALU_MVN : begin
        result_o = ~b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;           
      end
      
      default : begin
        result_o = 32'h0;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = negative_i;
        zero_o = zero_i;          
      end
  

    endcase

end

endmodule


