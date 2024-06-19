
module i2c_master(input             i_clk,      
                  input             reset_n,           
                  input      [7:0]  i_addr_w_rw,     
                  input      [15:0] i_sub_addr,         
                  input             i_sub_len,          
                  input      [23:0] i_byte_len,       
                  input      [7:0]  i_data_write,       
                  input             req_trans,          
                  

                  output reg [7:0]  data_out,
                  output reg        valid_out,
             
                  inout             scl_o,            
                  inout             sda_o,              
                  
                  output reg        req_data_chunk ,    
                  output reg        busy,               
                  output reg        nack               
                  
                  `ifdef DEBUG
                  ,
                  output reg [3:0]  state,
                  output reg [3:0]  next_state,
                  output reg        reg_sda_o,
                  output reg [7:0]  addr,
                  output reg        rw,
                  output reg [15:0] sub_addr,
                  output reg        sub_len,
                  output reg [23:0] byte_len,
                  output reg        en_scl,
                  output reg        byte_sent,
                  output reg [23:0] num_byte_sent,
                  output reg [2:0]  cntr,
                  output reg [7:0]  byte_sr,
                  output reg        read_sub_addr_sent_flag,
                  output reg [7:0]  data_to_write,
                  output reg [7:0]  data_in_sr,
                  
                  //400KHz clock generation
                  output reg        clk_i2c,
                  output reg [15:0] clk_i2c_cntr,
                  
                  //sampling sda and scl
                  output reg        sda_prev,
                  output reg [1:0]  sda_curr,
                  output reg        scl_prev,
                  output reg        scl_curr,
                  output reg        ack_in_prog,
                  output reg        ack_nack,
                  output reg        en_end_indicator,
                  output reg        grab_next_data,
                  output reg        scl_is_high,
                  output reg        scl_is_low
                  `endif
                  );

//For state machine                 
localparam [3:0] IDLE        = 4'd0,
                 START       = 4'd1,
                 RESTART     = 4'd2,
                 SLAVE_ADDR  = 4'd3,
                 SUB_ADDR    = 4'd4,
                 
                 READ        = 4'd5,
                 WRITE       = 4'd6,
                 GRAB_DATA   = 4'd7,
                 ACK_NACK_RX = 4'd8,
                 ACK_NACK_TX = 4'd9,
                 STOP        = 4'hA,
                 RELEASE_BUS = 4'hB;
                 
localparam [15:0] DIV_100MHZ = 16'd125;         
localparam [7:0]  START_IND_SETUP  = 70,  //Time before negedge of scl
                  START_IND_HOLD   = 60,  //Time after posedge of clock when start occurs (not used)
                  DATA_SETUP_TIME  =  2,  //Time needed before posedge of scl 
                  DATA_HOLD_TIME   =  3,  //Time after negedge that scl is held
                  STOP_IND_SETUP   = 60;  //Time after posedge of scl before stop occurs
                  
`ifndef DEBUG
reg [3:0]  state;
reg [3:0]  next_state;
reg        reg_sda_o;
reg [7:0]  addr;
reg        rw;
reg [15:0] sub_addr;
reg        sub_len;
reg [23:0] byte_len;
reg        en_scl;
reg        byte_sent;
reg [23:0] num_byte_sent;
reg [2:0]  cntr;
reg [7:0]  byte_sr;
reg        read_sub_addr_sent_flag;
reg [7:0]  data_to_write;
reg [7:0]  data_in_sr;

//For generation of 400KHz clock
reg clk_i2c;
reg [15:0] clk_i2c_cntr;

//For taking a sample of the scl and sda
reg [1:0] sda_curr;    
reg       sda_prev;
reg scl_prev, scl_curr;          

reg ack_in_prog;      //For sending acks during read
reg ack_nack;
reg en_end_indicator;

reg grab_next_data;
reg scl_is_high;
reg scl_is_low;
`endif

always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n)
        {clk_i2c_cntr, clk_i2c} <= 17'b1;
    else if(!en_scl)
        {clk_i2c_cntr, clk_i2c} <= 17'b1;
    else begin
        clk_i2c_cntr <= clk_i2c_cntr + 1;
        if(clk_i2c_cntr == DIV_100MHZ-1) begin
            clk_i2c <= !clk_i2c;
            clk_i2c_cntr <= 0;
        end
    end
end

//Main FSM
always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {data_out, valid_out} <= 0;
        {req_data_chunk, busy, nack} <= 0;
        {addr, rw, sub_addr, sub_len, byte_len, en_scl} <= 0;
        {byte_sent, num_byte_sent, cntr, byte_sr} <= 0;
        {read_sub_addr_sent_flag, data_to_write, data_in_sr} <= 0;
        {ack_nack, ack_in_prog, en_end_indicator} <= 0;
        {scl_is_high, scl_is_low, grab_next_data} <= 0;
        reg_sda_o <= 1'bz;
        state <= IDLE;
        next_state <= IDLE;
    end
    else begin
        valid_out <= 1'b0;
        req_data_chunk <= 1'b0;
        case(state)
            IDLE: begin
                if(req_trans & !busy) begin
                    //set busy
                    busy <= 1'b1;
                    //Set FSM in motion
                    state <= START;
                    next_state <= SLAVE_ADDR;
                    
                    //Set all master inputs to local registers to modify and or reference later
                    addr <= i_addr_w_rw;
                    rw <= i_addr_w_rw[0];
                    sub_addr <= i_sub_len ? i_sub_addr : {i_sub_addr[7:0], 8'b0};
                    sub_len <= i_sub_len;
                    data_to_write <= i_data_write;
                    byte_len <= i_byte_len;

                    //begin the 400kHz generation                    
                    en_scl <= 1'b1;
                    reg_sda_o <= 1'b1;
                    
                    //Reset flags and or counters
                    nack <= 1'b0;  
                    read_sub_addr_sent_flag <= 1'b0;
                    num_byte_sent <= 0;
                    byte_sent <= 1'b0;
                end
            end
            
            START: begin
                if(scl_prev & scl_curr & clk_i2c_cntr == START_IND_SETUP) begin   //check that scl is high, and that a necessary wait time is held
                    reg_sda_o <= 1'b0;                                       //set start bit for negedge of clock, and toggle for the clock to begin
                    byte_sr <= {addr[7:1], 1'b0};                            //Don't need to check read or write, will always have write in a read request as well
                    state <= SLAVE_ADDR;
                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: START INDICATION!", $time);
                end
            end
            
            RESTART: begin
                if(!scl_curr & scl_prev) begin
                    reg_sda_o <= 1'b1;              //Set line high
                end
                
                if(!scl_prev & scl_curr) begin      //so i2c cntr has reset
                    scl_is_high <= 1'b1;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == START_IND_SETUP) begin   //Must wait minimum setup time
                        scl_is_high <= 1'b0;
                        reg_sda_o <= 1'b0;
                        state <= SLAVE_ADDR;
                        byte_sr <= addr;
                    end
                end
            end

            SLAVE_ADDR: begin
                //When scl has fallen, we can change sda 
                if(byte_sent & cntr[0]) begin
                    byte_sent <= 1'b0;                      //deassert the flag
                    next_state <= read_sub_addr_sent_flag ? READ : SUB_ADDR;   
                    byte_sr <= sub_addr[15:8];          
                    state <= ACK_NACK_RX;                   //await for nack_ack
                    reg_sda_o <= 1'bz;                      //release sda line
                    cntr <= 0;
                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SLAVE_ADDR SENT!", $time);
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;      
                            reg_sda_o <= byte_sr[7];                //send MSB
                            byte_sr <= {byte_sr[6:0], 1'b0};        //shift out MSB
                            scl_is_low <= 1'b0;
                        end
                    end
                end
            end
          
            SUB_ADDR: begin
                if(byte_sent & cntr[0]) begin
                    if(sub_len) begin                       //1 for 16 bit
                        state <= ACK_NACK_RX;
                        next_state <= SUB_ADDR;
                        sub_len <= 1'b0;                    //denote only want 8 bit next time
                        byte_sr <= sub_addr[7:0];           //set the byte shift register
                        $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: MSB OF SUB ADDR SENT", $time);
                    end
                    else begin
                        next_state <= rw ? RESTART : WRITE;   //move to appropriate state
                        byte_sr <= rw ? byte_sr : data_to_write; //if write, want to setup the data to write to device
                        read_sub_addr_sent_flag <= 1'b1;    //For dictating state of machine
                        $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SUB ADDR SENT", $time);
                    end
                    
                    cntr <= 0;
                    byte_sent <= 1'b0;                      //deassert the flag
                    state <= ACK_NACK_RX;                   //await for nack_ack
                    reg_sda_o <= 1'bz;                       //release sda line
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            scl_is_low <= 1'b0;
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;       //incr cntr, with overflow being caught
                            reg_sda_o <=  byte_sr[7];               //send MSB
                            byte_sr <= {byte_sr[6:0], 1'b0};        //shift out MSB
                        end
                    end
                end
            end
            
            /***
             * State: Reads
             * Purpose: Read 1 byte messages that are set on posedge of i2c_clk
             * How it Works: Need to read all 8 bits, on posedge of clock. SDA will be
             *               stable high before this occurs, thus it's fine to grab sda_prev,
             *               which is synchronous to i_clk. Every 
             */
            READ: begin
                if(byte_sent) begin
                    byte_sent <= 1'b0;          //reset flag
                    data_out  <= data_in_sr;    //put information in valid output
                    valid_out <= 1'b1;          //Let master know valid output
                    state <= ACK_NACK_TX;       //Send ack
                    next_state <= (num_byte_sent == byte_len-1) ? STOP : READ;      //Have we read all bytes?
                    ack_nack <= num_byte_sent == byte_len-1;                        //If true, then 1, which is a nack
                    num_byte_sent <= num_byte_sent + 1;  //Incr number of bytes read
                    ack_in_prog <= 1'b1;
                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: READ BYTE #%d SENT!", $time, num_byte_sent);
                end
                else begin
                    if(!scl_prev & scl_curr) begin
                        scl_is_high <= 1'b1;
                    end
                    
                    if(scl_is_high) begin
                        if(clk_i2c_cntr == START_IND_SETUP) begin
                            valid_out <= 1'b0;
                            {byte_sent, cntr} <= cntr + 1;
                            data_in_sr <= {data_in_sr[6:0], sda_prev}; //MSB first
                            scl_is_high <= 1'b0;
                        end
                    end
                end
            end
            
            /***
             * State: Write
             * Purpose: Write specified data words starting from address and incrementing by 1
             * How it Works: Simply send data out 1 byte at a time, with corresponding acks form slave.
             *               When all bytes are written, quit comms.
             */
            WRITE: begin
                if(byte_sent & cntr[0]) begin
                    cntr <= 0;
                    byte_sent <= 1'b0;
                    state <= ACK_NACK_RX;
                    reg_sda_o <= 1'bz;
                    next_state <= (num_byte_sent == byte_len-1) ? STOP : GRAB_DATA;
                    num_byte_sent <= num_byte_sent + 1'b1;
                    grab_next_data <= 1'b1;
                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: WRITE BYTE #%d SENT!", $time, num_byte_sent);
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin //negedge
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;
                            reg_sda_o <= byte_sr[7];
                            byte_sr <= {byte_sr[6:0], 1'b0};        //shift out MSB
                            scl_is_low <= 1'b0;
                        end
                    end
                end
            end
            
            /***
             * State: GRAB_DATA
             * Purpose: Grab next 8 bit segment as needed
             * How it works: dequeue data, then grab the word requested (dequeue is req_data_chunk)
             */
            GRAB_DATA: begin
                if(grab_next_data) begin
                    req_data_chunk <= 1'b1;
                    grab_next_data <= 1'b0;
                end
                else begin
                    state <= WRITE;
                    byte_sr <= i_data_write;
                end
            end
            
            /***
             * State: ACK_NACK_RX
             * Purpose: Receive ack_nack from slave
             * How it works: sda is already freed, simply look at posedges of scl, and look at data
             *               remember low is considered an ack, and high is a nack
             */
            ACK_NACK_RX: begin
                if(!scl_prev & scl_curr) begin
                    scl_is_high <= 1'b1;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == START_IND_SETUP) begin
                        if(!sda_prev) begin      //checking for the ack condition (its low)
                            state <= next_state;
                            $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: rx ack encountered", $time);
                        end
                        else begin
                            $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: rx nack encountered", $time);
                            nack <= 1'b1;
                            busy <= 1'b0;
                            reg_sda_o <= 1'bz;
                            en_scl <= 1'b0;
                            state <= IDLE;
                        end  
                        scl_is_high <= 1'b0;
                    end
                end
            end
            
            /***
             * State: ACK_NACK_TX
             * Purpose: Take hold of SDA to acknowledge the read
             * How it works: On first negedge, since previous state will move on posedge, 
             *               pull the line low for an ack. On second negedge, release sda.
             */
            ACK_NACK_TX: begin
                if(!scl_curr & scl_prev) begin
                    scl_is_low <= 1'b1;
                end
                if(scl_is_low) begin          //negedge
                    if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                        if(ack_in_prog) begin 
                            reg_sda_o <= ack_nack;          //write ack until negedge of clk
                            ack_in_prog <= 1'b0;
                        end
                        else begin
                            reg_sda_o <= next_state == STOP ? 1'b0 : 1'bz;
                            en_end_indicator <= next_state == STOP ? 1'b1 : en_end_indicator;
                            state <= next_state;
                        end
                        scl_is_low <= 1'b0;
                    end
                end
            end
            
            /***
             * State: STOP
             * Purpose: Pulls bus low on negedge, and waits for scl to be high
             *          drive sda to high from low, which is stop indication
             */
            STOP: begin 
                if(!scl_curr & scl_prev & !rw) begin //negedge only if we are writing
                    reg_sda_o <= 1'b0;               //Set to low
                    en_end_indicator <= 1'b1;
                end
                
                //Note addition of counter, needed to ensure that there is enough delay for target device
                if(scl_curr & scl_prev & en_end_indicator) begin
                    scl_is_high <= 1'b1;
                    en_end_indicator <= 1'b0;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == STOP_IND_SETUP) begin
                        reg_sda_o <= 1'b1;
                        state <= RELEASE_BUS;
                        scl_is_high <= 1'b0;
                    end
                end
            end
            
            /***
             * State: Release bus
             * Purpose: Release the bus
             * How it works: Turn off 400KHz out and release the sda line, go back to idle
             */
            RELEASE_BUS: begin
                if(clk_i2c_cntr == DIV_100MHZ-3) begin
                    en_scl <= 1'b0;
                    state <= IDLE;
                    reg_sda_o <= 1'bz;
                    busy <= 1'b0;
                end
            end
            
            default:
                state <= IDLE;
        endcase
    end
end

/*
 * Purpose: grabbing sda from slave
 */
always@(negedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {sda_curr, sda_prev} <= 0;
        {scl_curr, scl_prev} <= 0;
    end
    else begin
        sda_curr <= {sda_curr[0], sda_o};  //2 flip flop synchronization chain
        sda_prev <= sda_curr[1];
        scl_curr <= clk_i2c;
        scl_prev <= scl_curr;
    end
end

//inout cannot be reg
assign sda_o = reg_sda_o;
assign scl_o = en_scl ? clk_i2c : 1'bz;     //the line will be pulled up to VCC so 1'bz is high
endmodule