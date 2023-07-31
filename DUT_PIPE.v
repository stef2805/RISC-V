
module DUT_PIPE(
   
    );
    reg clk;
    reg res;
    wire PCSrc;
    wire [31:0] SumPC;
    wire [31:0] pc, instruction;
    wire [6:0] imm;
    wire [31:0] writeData;
    wire [5:0] writeRegister;
    wire RegWrite;
    
    wire[31:0]ReadData1; 
    wire[31:0]ReadData2;
    wire[31:0]pc2;  
    wire[6:0]imm;        
    wire[31:0]instruction2;
    wire [1:0] controlWB;
    wire [2:0] controlMEM;
    wire [2:0] controlEX;
    wire [31:0] Summ;            
    wire Zero;                   
    wire [31:0] ALUresult;       
    wire [31:0] ReadData_next;   
    wire [31:0] Instruction_next;
    wire [1:0] controlWB_next;   
    wire [2:0] controlMEM_next; 
    wire [31:0] dout;             
    wire [31:0] instruction_last; 
    wire [31:0] ALUres;           
    wire        PCSrc;            
    wire        Sum;              
    wire [31:0] controlWB_last;   
    
    
      

    IF instr_fetch (
        clk,
        res,
        PCSrc,
        SumPC,
       
        pc,
        instruction
    );
    
    ID instr_decode(
        clk,            
        res,   
                 
        instruction, 
        pc,          
        writeData,      
        writeRegister,  
        RegWrite,  
            
        ReadData1,                                      
        ReadData2,                                      
        pc2,                                            
        imm,                                            
        instruction2,                                   
        controlWB,  //bit 0 = RegWrite bit 1= MemToReg  
        controlMEM,     //{Branch,MemRead,MemWrite}     
        controlEX      //bit 0,1 = AluOP, bit 2 = AluSrc
         
    );

    EX execution(
         clk,                                                    
         res,     
                                                        
         ReadData1,                                      
         ReadData2,                                      
         pc2,                                            
         imm,                                            
         instruction2,                                   
         controlWB,  //bit 0 = RegWrite bit 1= MemToReg  
         controlMEM,     //{Branch,MemRead,MemWrite}     
         controlEX,      //bit 0,1 = AluOP, bit 2 = AluSrc
          
         writeRegister,           //reg dest din ultimul etaj    
         writeData,             //instr din ultimul etaj         
         instruction_last,      //instr din urm etaj             
         ALUres,             //data din urm etaj                 
                                                         
             
        
        Summ,
        Zero,
        ALUresult,
        ReadData_next,
        Instruction_next,
        controlWB_next,      //bit 0 = RegWrite bit 1= MemToReg
        controlMEM_next,      //{Branch,MemRead,MemWrite}
        PCSrc,            
        SumPC
    );
    
    
    MEM memory_acces(
        clk,                     
        res,    
                         
        Summ,
        Zero,
        ALUresult,
        ReadData_next,
        Instruction_next,
        controlWB_next,      //bit 0 = RegWrite bit 1= MemToReg
        controlMEM_next,      //{Branch,MemRead,MemWrite}
                                 
        dout,                    
        instruction_last,        
        ALUres,                  
        controlWB_last            
    );
    
  
    WB WrteBack(
        clk,                  
        res,                  
                              
        dout,             
        instruction_last, 
        ALUres,                       
        controlWB_last,   
                              
        RegWrite,             
        writeData,            
        writeRegister           
    );
    
initial begin
        clk = 0; res = 0;
        #10 res = 1;
        #10 res = 0;
        #300 $finish;
    end

    always #5 clk = ~clk;

endmodule