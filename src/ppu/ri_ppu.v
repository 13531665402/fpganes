//verilog语句：always@*（）表明后面括号内的输入变量均为敏感变量
//八个寄存器是在cpu上的
// write操作：cpu的值写到ppu里
// read操作：cpu从ppu里读入,
module ri_ppu
(
    input wire clk_in,
    input wire rst_in,
    input wire ncs_in,//是否进行寄存器操作
    input wire r_w_sel_in,//读操作还是写操作
    input wire [2: 0] sel_reg_in,//具体的操作寄存器
    input wire [7: 0] cpu_data_in,//write操作的时候写入的值
    output wire [7: 0] cpu_data_out,//read操作的时候输出的值
    output reg addr_inc_out,
    output wire upd_cntrs_out,
    //$2000 write
    output wire vram_addr_inc_out,// 0:add 1, 1:add 32;
    output wire spr_pat_addr_out,//0:$0000, 1:$1000;
    output wire bg_pat_addr_out,//0:$0000, 1:$1000;
    output wire spr_sz_out,//0：8 * 8， 1：16 * 16
    output wire nmi_out,//0:off, 1:on;
    //$2001 write
    output wire bg_show_lf_out,//0:hide, 1:show, 以下三个同理；
    output wire spr_show_lf_out,
    output wire bg_show_out,
    output wire spr_show_out,
    //$2002 read
    input wire spr_overflow_in,//记录精灵的个数有没有超过8个；
    input wire spr_zero_hit_in,
    input wire vblank_in,
    output wire vblank_out,
    //$2003 write
    output wire [7: 0] oam_addr_out,
    //$2004 read/write
    output reg oam_r_w_out,
    //write
    output reg [7: 0] oam_data_out,
    //read
    input wire [7: 0] oam_data_in,
    //$2005 write * 2
    //$2006 write * 2
    output wire [2: 0] fine_v_out,
    output wire [4: 0] v_tile_index_out,
    output wire [2: 0] fine_h_out,
    output wire [4: 0] h_tile_index_out,
    output wire v_nt_sel_out,
    output wire h_nt_sel_out,
    //$2007 read/write
    input wire [13: 0] vram_addr_in,
    output reg vram_r_w_out,
    output reg pram_r_w_out,
    //write
    output reg [7: 0] vram_data_out,
    //read
    input wire [7: 0] vram_data_in,
    input wire [7: 0] pram_data_in
    
);
//Scroll Registers
//vertical:
reg [2: 0] q_fine_v, d_fine_v;
reg [4: 0] q_v_tile_index, d_v_tile_index;
reg        q_v_nt_sel, d_v_nt_sel;
//horizontal:
reg [2: 0] q_fine_h, d_fine_h;
reg [4: 0] q_h_tile_index, d_h_tile_index;
reg        q_h_nt_sel, d_h_nt_sel;

//External State Registers
reg [7: 0]q_cpu_data_out, d_cpu_data_out;
reg q_upd_cntrs_out, d_upd_cntrs_out;
reg q_vram_addr_inc_out, d_vram_addr_inc_out;
reg q_spr_pat_addr_out, d_spr_pat_addr_out;
reg q_bg_pat_addr_out, d_bg_pat_addr_out;
reg q_spr_sz_out, d_spr_sz_out;
reg q_nmi_out, d_nmi_out;

reg q_bg_show_lf_out, d_bg_show_lf_out;
reg q_spr_show_lf_out, d_spr_show_lf_out;
reg q_bg_show_out, d_bg_show_out;
reg q_spr_show_out, d_spr_show_out;

reg q_vblank, d_vblank;

reg [7: 0] q_oam_addr_out, d_oam_addr_out;
reg [7: 0] q_oam_data_out, d_oam_data_out;

reg [7: 0] q_vram_data_out, d_vram_data_out;

//Internal State Registers
reg q_w, d_w;
reg [7: 0] q_read_buf, d_read_buf;
reg q_read_buf_upd, d_read_buf_upd;
reg q_ncs_in;
reg q_vblank_in;

always @(posedge clk_in)
    begin
    if(rst_in)
        begin
          q_fine_v  <= 2'h0;
          q_v_tile_index <= 5'h0;
          q_v_nt_sel <= 1'h0;
          q_fine_h <= 2'h0;
          q_h_tile_index <= 5'h0;
          q_h_nt_sel <= 1'h0;
          q_cpu_data_out <= 8'h00;
          q_vram_addr_inc_out <= 1'h0;
          q_spr_pat_addr_out <= 1'h0;
          q_bg_pat_addr_out <= 1'h0;
          q_spr_sz_out <= 1'h0;
          q_upd_cntrs_out <= 1'h0;
          q_nmi_out <= 1'h0;
          q_bg_show_lf_out <= 1'h0;
          q_spr_show_lf_out <= 1'h0;
          q_bg_show_out <= 1'h0;
          q_spr_show_out <= 1'h0;
          q_vblank <= 1'h0;
          q_oam_addr_out <= 8'h00;
          q_oam_data_out <= 8'h00;
          q_vram_data_out <= 8'h00;
          q_w <= 1'h0;
          q_read_buf <= 8'h00;
          q_read_buf_upd <= 1'h0;
          q_ncs_in <= 1'h1;
          q_vblank_in <= 1'h0;
        end
    else 
        begin
          q_fine_v  <= d_fine_v;
          q_v_tile_index <= d_v_tile_index;
          q_v_nt_sel <= d_v_nt_sel;
          q_fine_h <= d_fine_h;
          q_h_tile_index <= d_h_tile_index;
          q_h_nt_sel <= d_h_nt_sel;
          q_cpu_data_out <= d_cpu_data_out;
          q_upd_cntrs_out <= d_upd_cntrs_out;
          q_vram_addr_inc_out <= d_vram_addr_inc_out;
          q_spr_pat_addr_out <= d_spr_pat_addr_out;
          q_bg_pat_addr_out <= d_bg_pat_addr_out;
          q_spr_sz_out <= d_spr_sz_out;
          q_nmi_out <= d_nmi_out;
          q_bg_show_lf_out <= d_bg_show_lf_out;
          q_spr_show_lf_out <= d_spr_show_lf_out;
          q_bg_show_out <= d_bg_show_out;
          q_spr_show_out <= d_spr_show_out;
          q_vblank <= d_vblank;
          q_oam_addr_out <= d_oam_addr_out;
          q_w <= d_w;
          q_read_buf <= d_read_buf;
          q_read_buf_upd <= d_read_buf_upd;
          q_ncs_in <= ncs_in;
          q_vblank_in <= vblank_in;
        end     
    end

always @*
    begin

      d_fine_v  = q_fine_v;
      d_v_tile_index = q_v_tile_index;
      d_v_nt_sel = q_v_nt_sel;
      d_fine_h = q_fine_h;
      d_h_tile_index = q_h_tile_index;
      d_h_nt_sel = q_h_nt_sel;

      d_cpu_data_out = q_cpu_data_out;
      d_vram_addr_inc_out = q_vram_addr_inc_out;
      d_spr_pat_addr_out = q_spr_pat_addr_out;
      d_bg_pat_addr_out = q_bg_pat_addr_out;
      d_spr_sz_out = q_spr_sz_out;
      d_nmi_out = q_nmi_out;
      d_bg_show_lf_out = q_bg_show_lf_out;
      d_spr_show_lf_out = q_spr_show_lf_out;
      d_bg_show_out = q_bg_show_out;
      d_spr_show_out = q_spr_show_out;

      d_oam_addr_out = q_oam_addr_out;
      d_w = q_w;
      d_upd_cntrs_out = 1'b0;
      if(~q_vblank_in & vblank_in)
        d_vblank = 1'h1;
      else if(~vblank_in)
        d_vblank = 1'h0;
      else d_vblank = q_vblank;
      vram_r_w_out = 1'h0;
      pram_r_w_out = 1'h0;
      oam_r_w_out = 1'h0;
      addr_inc_out = 1'h0;
      if(d_read_buf_upd)
        d_read_buf = vram_data_in;
      else d_read_buf = q_read_buf;
      vram_data_out = 8'h00;
      oam_data_out = 8'h00;
      if(q_ncs_in & ~ncs_in)
        begin
          if(r_w_sel_in)
            begin
              case(sel_reg_in)
                3'h2:
                    begin
                      d_cpu_data_out[5] = spr_overflow_in;
                      d_cpu_data_out[6] = spr_zero_hit_in;
                      d_cpu_data_out[7] = q_vblank;
                      d_cpu_data_out[4: 0] = 5'b00000;
                      d_vblank = 0;
                    end
                3'h4:
                    begin
                      d_cpu_data_out = oam_data_in;
                    end
                3'h7:
                    begin
                      if(vram_addr_in[13: 8] == 6'h3F)
                        d_cpu_data_out = pram_data_in;
                      else d_cpu_data_out = q_read_buf;
                      d_read_buf_upd = 1'b1;//hint
                      addr_inc_out = 1'b1;
                    end
                endcase
            end
          else
            begin
              case(sel_reg_in)
                3'h0:
                    begin
                      d_h_nt_sel = cpu_data_in[0];
                      d_v_nt_sel = cpu_data_in[1];
                      d_vram_addr_inc_out = cpu_data_in[2];
                      d_spr_pat_addr_out = cpu_data_in[3];
                      d_bg_pat_addr_out = cpu_data_in[4];
                      d_spr_sz_out = cpu_data_in[5];
                      d_nmi_out = cpu_data_in[7];
                    end
                3'h1:
                    begin
                      d_bg_show_lf_out = cpu_data_in[1];
                      d_spr_show_lf_out = cpu_data_in[2];
                      d_bg_show_out = cpu_data_in[3];
                      d_spr_show_out = cpu_data_in[4];
                    end
                3'h3:
                    begin
                     d_oam_addr_out = cpu_data_in;
                    end
                3'h4:
                    begin
                      oam_r_w_out = 1'b1;
                      oam_data_out = cpu_data_in;
                     d_oam_addr_out = q_oam_addr_out + 8'h01;
                    end
                3'h5:
                    begin
                      d_w = ~q_w;
                      if(~q_w)
                        begin
                          d_fine_h = cpu_data_in[2: 0];
                          d_h_tile_index = cpu_data_in[7: 3];
                        end
                      else 
                        begin
                          d_fine_v = cpu_data_in[2: 0];
                          d_v_tile_index = cpu_data_in[7: 3];
                        end
                    end
                3'h6:
                    begin
                      d_w = ~q_w;
                      if(~q_w)
                        begin
                          d_fine_v = {1'b0, cpu_data_in[5: 4]};
                          d_v_nt_sel = cpu_data_in[3];
                          d_h_nt_sel = cpu_data_in[2];
                          d_v_tile_index[4: 3] = cpu_data_in[1: 0];
                        end
                      else 
                        begin
                          d_v_tile_index[2: 0] = cpu_data_in[7: 5];
                          d_h_tile_index = cpu_data_in[4: 0];
                          d_upd_cntrs_out = 1'b1;
                        end
                    end
                3'h7:
                    begin
                      if(vram_addr_in[13: 8] == 6'h3F)
                        pram_r_w_out = 1'b1;
                      else vram_r_w_out = 1'b1;
                      vram_data_out = cpu_data_in;
                      addr_inc_out = 1'b1;
                    end
                endcase
            end
        end
    end
assign fine_v_out = q_fine_v;
assign v_tile_index_out = q_v_tile_index;
assign v_nt_sel_out = q_v_nt_sel;
assign fine_h_out = q_fine_h;
assign h_tile_index_out = q_h_tile_index;
assign h_nt_sel_out = q_h_nt_sel;
assign cpu_data_out = (~ncs_in & r_w_sel_in)? q_cpu_data_out: 8'h00;
assign vram_addr_inc_out = q_vram_addr_inc_out;
assign nmi_out = q_nmi_out;
assign vblank_out = q_vblank;
assign bg_show_lf_out = q_bg_show_lf_out;
assign bg_show_out = q_bg_show_out;
assign spr_show_lf_out = q_spr_show_lf_out;
assign spr_show_out = q_spr_show_out;
assign spr_sz_out = q_spr_sz_out;
assign spr_pat_addr_out = q_spr_pat_addr_out;
assign oam_addr_out = q_oam_addr_out;
assign upd_cntrs_out = q_upd_cntrs_out;

endmodule