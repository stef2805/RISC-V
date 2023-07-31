module IF(
    input clk,
    input res,
    input PCSrc,
    input [31:0] Sum,
    
    output reg [31:0] pc,
    output reg [31:0] instruction
);
    always@(posedge clk)   //e nevoie aici pt a putea rula prima instructiune
        if(res)
            pc <= 0;
    
            
    reg [32 / 4 -1:0] mem [0:256 * 4 -1];
    
    initial begin
        $readmemh( "memory_code.mem", mem);     //initializare memorie
    end
    
    always@(posedge clk)
        if(!res)
            instruction = {mem[pc+3], mem[pc+2], mem[pc+1], mem[pc]};   //citim instr din mem de la adresa lui pc
        
    
    
    always@(posedge clk)begin   //ultima chestie de fct
        if(!res)
            if(PCSrc == 0)
                pc <= pc+4; 
            else
                pc <=  Sum;         //rezultatul sumatorului pt brenchuri
    end
endmodule

module ID(
    input        clk,
    input        res,
    input [31:0] oldinstruction,
    input [31:0] oldpc,
    input [31:0] writeData,
    input [5:0]  writeRegister,
    input        RegWrite,
    
    output reg [31:0] ReadData1,
    output reg [31:0] ReadData2,
    output reg [31:0] pc2,
    output reg [6:0]  imm,
    output reg [31:0] instruction2,
    output reg [1:0]  controlWB,  //bit 0 = RegWrite bit 1= MemToReg
    output reg [2:0]  controlMEM,     //{Branch,MemRead,MemWrite}
    output reg [2:0]  controlEX      //bit 0,1 = AluOP, bit 2 = AluSrc
);
    reg [31:0] mem [0:31];
    
    always@(posedge clk)begin
        if(res)begin
            ReadData2 <= 0;
            ReadData1 <= 0;
            mem[0] <= 0; mem[1] <= 0; mem[2] <= 0; mem[3] <= 0; mem[4] <= 0; mem[5] <= 0; mem[6] <= 0; mem[7] <= 0; 
            mem[8] <= 0; mem[9] <= 0; mem[10] <= 0; mem[11] <= 0; mem[12] <= 0; mem[13] <= 0; mem[14] <= 0; mem[15] <= 0;
            mem[16] <= 0; mem[17] <= 0; mem[18] <= 0; mem[19] <= 0; mem[20] <= 0; mem[21] <= 0; mem[22] <= 0; mem[23] <= 0;
            mem[24] <= 0; mem[25] <= 0; mem[26] <= 0; mem[27] <= 0; mem[28] <= 0; mem[29] <= 0; mem[30] <= 0; mem[31] <= 0;
        end
        else
            begin
                if(RegWrite)
                        mem[writeRegister] <= writeData;
                ReadData2 <= mem[oldinstruction[24:20]];
                ReadData1 <= mem[oldinstruction[19:15]];
            end
        
        case(oldinstruction[6:0])
            7'b0000011,
            7'b0001111,
            7'b0011011,
            7'b1100111,
            7'b1110011,
            7'b0010011: imm = { {20{oldinstruction[31]}}, oldinstruction[31:20]};
            7'b0100011: imm = { {20{oldinstruction[31]}}, oldinstruction[31:25], oldinstruction[11:7]};
            7'b1100011: imm = { {20{oldinstruction[31]}}, oldinstruction[7], oldinstruction[30:25], oldinstruction[11:8], 1'b0};            
            7'b1101111: imm = { {12{oldinstruction[31]}}, oldinstruction[19:12], oldinstruction[20], oldinstruction[30:25], oldinstruction[11:8], 1'b0};            
            7'b0010111,
            7'b0110111: imm = { oldinstruction[31:12], {12{1'b0}}}; 
            default:
                imm = 32'h0000_0000;
        endcase
    
    
            //asignari pt urmatoarea etapa
        pc2 <= oldpc;
        instruction2 <= oldinstruction;
    
    //////////////////// CONTROLUL////////////////////
    
        case(oldinstruction[6:0])  //opcode-ul instructiunii
            7'b0110011: {controlEX, controlMEM, controlWB} = 8'b10_0__0_0_0__1_0;  // R
            7'b0000011: {controlEX, controlMEM, controlWB} = 8'b00_1__0_1_0__1_1;  // lw
            7'b0100011: {controlEX, controlMEM, controlWB} = 8'b00_1__0_0_1__0_0;  // sw
            7'b1100011: {controlEX, controlMEM, controlWB} = 8'b01_0__1_0_0__0_0;  // beq
            
            7'b0010011: {controlEX, controlMEM, controlWB} = 8'b11_1__0_0_0__1_0;  // I
            
            default:
                {controlEX, controlMEM, controlWB} = 8'b0_0_0_0_0_0_00;
        endcase
    end 
endmodule


module EX(
    input        clk, 
    input        res,
    input [31:0] ReadData1IDEX,
    input [31:0] ReadData2IDEX,
    input [31:0] pc2IDEX,
    input [6:0]  immIDEX,
    input [31:0] instructionIDEX,
    input [1:0]  controlWBIDEX,      //bit 0 = RegWrite bit 1= MemToReg
    input [2:0]  controlMEMIDEX,     //{Branch,MemRead,MemWrite}
    input [2:0]  controlEXIDEX,       //bit 2,1 = AluOP, bit 0 = AluSrc
    input [5:0]  writeRegister,           //reg dest din ultimul etaj
    input [31:0] writeData,             //instr din ultimul etaj
    input [31:0] instruction_last,      //instr din urm etaj
    input [31:0] ALUres,             //data din urm etaj
    
    output reg [31:0] Sum,
    output reg        Zero,
    output reg [31:0] ALUresult,
    output reg [31:0] ReadData_next,
    output reg [31:0] Instruction_next,
    output reg [1:0]  controlWB_next,      //bit 0 = RegWrite bit 1= MemToReg
    output reg [2:0]  controlMEM_next,      //{Branch,MemRead,MemWrite}
    output reg        PCSrc,
    output reg [31:0] SumPC
);
    reg [2:0] alu_ctrl;
    reg [31:0] operand2;
    reg [31:0] operand1;
    
    ////////////////Sumatorul//////////////
    always@(posedge clk)begin                  
        Sum <= pc2IDEX + immIDEX;
    end
    
    /////////////////AluControl////////////
    always @(posedge clk) begin
    
        casex({controlEXIDEX[2:1] , instructionIDEX[31:25] , instructionIDEX[14:12]})
            12'b00_xxxxxxx_xxx: alu_ctrl = 4'b0010; // lw sw
            12'b01_xxxxxxx_xxx: alu_ctrl = 4'b0110; // beq
            
            12'b10_0000000_000: alu_ctrl = 4'b0010; // add
            12'b10_0100000_000: alu_ctrl = 4'b0110; // sub            
            12'b10_0000000_111: alu_ctrl = 4'b0000; // and
            12'b10_0000000_110: alu_ctrl = 4'b0001; // or
            12'b10_0000000_100: alu_ctrl = 4'b0011; // xor
            
            12'b11_xxxxxxx_000: alu_ctrl = 4'b0010; // addi
            12'b11_xxxxxxx_111: alu_ctrl = 4'b0000; // andi
            12'b11_xxxxxxx_110: alu_ctrl = 4'b0001; // ori
            12'b11_xxxxxxx_100: alu_ctrl = 4'b0011; // xori           
            
            default:
                alu_ctrl = 4'b0000;
        endcase  
    
    /////////////FORWARDING//////////
        operand1 = ReadData1IDEX;                      
        operand2 = (controlEXIDEX[0] ? immIDEX : ReadData2IDEX);  //in caz ca nu avem fwd ramane ca operanzii sa fie registrii normali
        
        if(instruction_last[11:7]==instructionIDEX[19:15])      //daca rd de la instr urm == rs1 de la instr curenta   
            operand1 = ALUres;
        if(instruction_last[11:7]==instructionIDEX[24:20])      ////daca rd de la instr urm == rs2 de la instr curenta
            operand2 =  ALUres;
        if(writeRegister == instructionIDEX[19:15])      //daca rd de la instr urm == rs1 de la instr curenta   
            operand1 = writeData;
        if(writeRegister == instructionIDEX[24:20])      ////daca rd de la instr urm == rs2 de la instr curenta
            operand2 =  writeData;
        
    /////////////ALU/////////////////
        
        
        case(alu_ctrl)
            4'b0010: ALUresult = operand1 + operand2;
            4'b0110: ALUresult = operand1 - operand2;
            4'b0000: ALUresult = operand1 & operand2;
            4'b0001: ALUresult = operand1 | operand2;
            4'b0011: ALUresult = operand1 ^ operand2;
            default:
                ALUresult = 32'hFFFF_FFFF;
        endcase
        Zero = ~(|ALUresult);  
    end
    
    always@(posedge clk)    
        case({ controlMEMIDEX[2],Zero})
            2'b11:  PCSrc = 1;
            default: PCSrc = 0;
        endcase
    
    ///////////PASARE DATE IN PIPELINE///////////////
     always@(posedge clk)begin                  
        controlWB_next  = controlWBIDEX;  
        controlMEM_next = controlMEMIDEX; 
        Instruction_next =  instructionIDEX;
        ReadData_next = ReadData2IDEX;
        SumPC = Sum-12;
    end
endmodule

module MEM(
    input             clk,
    input             res,
    input [31:0]      SumEXMEM,             
    input             ZeroEXMEM,            
    input [31:0]      ALUresultEXMEM,       
    input [31:0]      ReadData_nextEXMEM,   
    input [31:0]      Instruction_nextEXMEM,
    input [1:0]       controlWB_nextEXMEM,  
    input [2:0]       controlMEM_nextEXMEM,  
    
    output reg [31:0] dout,
    output reg [31:0] instruction_last,
    output reg [31:0] ALUres,
    output reg [1:0]  controlWB_last
);
    
    reg [32 / 4 - 1:0] memory [0:256 * 4 - 1];
    
    always@(posedge clk)begin
        casex({controlMEM_nextEXMEM[1],controlMEM_nextEXMEM[0]}) //MemRead si MemWrite
            2'b10:  dout = { memory[ALUresultEXMEM+3], memory[ALUresultEXMEM+2], memory[ALUresultEXMEM+1], memory[ALUresultEXMEM] };
            2'b01:  { memory[ALUresultEXMEM+3], memory[ALUresultEXMEM+2], memory[ALUresultEXMEM+1], memory[ALUresultEXMEM] } = ReadData_nextEXMEM;
            default: dout = 32'hFFFF_FFFF;
        endcase
    end
          
            
    always@(posedge clk)begin
        ALUres = ALUresultEXMEM;
        controlWB_last = controlWB_nextEXMEM;
        instruction_last = Instruction_nextEXMEM;
    end  
    
    initial begin
        $readmemh( "memory_data.mem", memory);
    end
   
endmodule


module WB(
    input            clk,
    input            res,
    
    input [31:0]     doutMEMW,            
    input [31:0]     instruction_lastMEMW,
    input [31:0]     ALUresMEMW,          
    input [1:0]      controlWB_lastMEMW,   
    
    output reg       RegWrite,
    output reg[31:0] writeData,
    output reg [5:0] writeRegister
);
    always@(controlWB_lastMEMW or doutMEMW or instruction_lastMEMW or ALUresMEMW)begin
        RegWrite = controlWB_lastMEMW[1];
        if(controlWB_lastMEMW[0])
            writeData = doutMEMW;
        else begin
            writeData = ALUresMEMW;
            end
        writeRegister = instruction_lastMEMW[11:7];
    end
endmodule