clear all

model_name='anniversary_3rd';
% SSY & SRH
% 恋爱三周年纪念
% 2022年11月26日

%读取图片
img=imread("1.bmp");
img=rgb2gray(img)<255/2;

%% 电路参数
[row_n,col_n]=size(img);
ts=0.01;
%正弦波形频率越低仿真越快，二极管电容CJ设为0，加快仿真
sin_feq=1/ts*3;

%地址位数
addr_bits=ceil(log2(col_n));

is_demo=false;%是否展示模型创建过程
demo_pause_time=0.5;

dc_v=5;%电源电压
diode_v=0.3;%二极管导通压降
vh=3;%高电平阈值
vl=2;%低电平阈值

%% 元件参数
unit_length=30; %单位长度

resistor.name='fl_lib/Electrical/Electrical Elements/Resistor';
resistor.width=unit_length;
resistor.height=unit_length;

converter.name='simulink/Signal Attributes/Data Type Conversion';
converter.width=unit_length;
converter.height=unit_length;

bits_extractor.name= 'simulink/Logic and Bit Operations/Extract Bits';
bits_extractor.width=unit_length*2;
bits_extractor.height=unit_length;

counter.name='simulink/Sources/Counter Limited';
counter.width=unit_length*2;
counter.height=unit_length*2;

amplifier.name='fl_lib/Electrical/Electrical Elements/Op-Amp';
amplifier.width=unit_length;
amplifier.height=unit_length;

ground.name='fl_lib/Electrical/Electrical Elements/Electrical Reference';
ground.width=unit_length;
ground.height=unit_length;

not_gate.name= 'ee_lib/Integrated Circuits/Logic/CMOS NOT';
not_gate.width=unit_length;
not_gate.height=unit_length;

elec_switch.name= 'fl_lib/Electrical/Electrical Elements/Switch';
elec_switch.width=unit_length;
elec_switch.height=unit_length;

open_circuit.name='fl_lib/Electrical/Electrical Elements/Open Circuit';
open_circuit.width=unit_length*2;
open_circuit.height=unit_length;

diode.name='ee_lib/Semiconductors & Converters/Diode';
diode.width=unit_length*1.5;
diode.height=unit_length;

buffer.name='ee_lib/Integrated Circuits/Logic/CMOS Buffer';
buffer.width=unit_length;
buffer.height=unit_length;

solver_config.name='nesl_utility/Solver Configuration';
solver_config.width=unit_length;
solver_config.height=unit_length;

dc_source.name='fl_lib/Electrical/Electrical Sources/DC Voltage Source';
dc_source.width=unit_length*1.5;
dc_source.height=unit_length;

ac_source.name='fl_lib/Electrical/Electrical Sources/AC Voltage Source';
ac_source.width=unit_length*1.5;
ac_source.height=unit_length;

sim_ps.name='nesl_utility/Simulink-PS Converter';
sim_ps.width=unit_length;
sim_ps.height=unit_length;

ps_sim.name='nesl_utility/PS-Simulink Converter';
ps_sim.width=unit_length;
ps_sim.height=unit_length;

voltage_sensor.name='fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
voltage_sensor.width=unit_length;
voltage_sensor.height=unit_length;

bus_creator.name='simulink/Signal Routing/Bus Creator';
bus_creator.width=10;
bus_creator.height=unit_length;

dead_zone.name='simulink/Discontinuities/Dead Zone';
dead_zone.width=unit_length;
dead_zone.height=unit_length;

scope.name='simulink/Sinks/Scope';
scope.width=unit_length;
scope.height=unit_length;

%% 布局参数
unit_margin=20; %单位间隔
matrix_margin_v=unit_margin;%矩阵格子内纵向间隔
matrix_margin_h=unit_margin*2;%矩阵格子内横向间隔
matrix_row_height=matrix_margin_v*2+diode.height;
matrix_addr_row_margin=matrix_row_height/2;
matrix_col_width=matrix_margin_h*2+diode.width;
matrix_right_x=matrix_col_width*col_n;

%%
if bdIsLoaded(model_name)
    close_system(model_name,0);
end
new_system(model_name);
set_param(model_name,'StopTime',num2str(ts*col_n));

if is_demo
    open_system(model_name);
    pause(demo_pause_time*10);
end
%set_param(model_name,'ZoomFactor','FitSystem')

%up,down时交换width,height
%获取开关端口相对位置（不在中间）
sw=add_block_at(model_name,elec_switch,'switch',0,0,[0,0],'left');
sw_port_y_rel=get_port_pos(model_name,sw,'LConn',1);
sw_port_y_rel=sw_port_y_rel(2)/elec_switch.height;
delete_block([model_name,'/',sw]);


%获取运放端口相对位置（不在中间）
amp0=add_block_at(model_name,amplifier,'amp',0,0,[0,0],'right','BlockRotation',num2str(180));
amp_port_neg_y_rel=get_port_pos(model_name,amp0,'LConn',2);
amp_port_neg_y_rel=amp_port_neg_y_rel(2)/amplifier.height;
amp_port_pos_y_rel=get_port_pos(model_name,amp0,'LConn',1);
amp_port_pos_y_rel=amp_port_pos_y_rel(2)/amplifier.height;
amp_port_out_y_rel=get_port_pos(model_name,amp0,'RConn',1);
amp_port_out_y_rel=amp_port_out_y_rel(2)/amplifier.height;
delete_block([model_name,'/',amp0]);

%获取电压表端口相对位置（不在中间）
vs=add_block_at(model_name,voltage_sensor,'amp',0,0,[0,0],'left');
vs_port_out_y_rel=get_port_pos(model_name,vs,'RConn',1);
vs_port_out_y_rel=vs_port_out_y_rel(2)/voltage_sensor.height;
delete_block([model_name,'/',vs]);

%% 创建电路
main_dc=add_block_at(model_name,dc_source,'main dc source',0,0,[0.5,0],'up','v0',num2str(dc_v));
ground_at_main_dc_source=add_block_at(model_name,ground,'ground at main dc source',0,-ground.width-unit_margin*2,[0.5,0],'up');
add_line(model_name,[ground_at_main_dc_source,'/LConn1'],[main_dc,'/RConn1'],'autorouting','off');
solver_config0=add_block_at(model_name,solver_config,'solver config',ground.height+unit_margin,-unit_margin,[0,0.5],'left','DoDC','on'); %DoDC 从稳态开始仿真，加速
add_line(model_name,[solver_config0,'/RConn1'],[main_dc,'/RConn1'],'autorouting','off');

set_param(model_name,'ZoomFactor','60');

main_dc_pos=get_param([model_name,'/',main_dc],'Position');
addr_line_res_y=main_dc_pos(4)+unit_margin*4;%地址线头y
first_addr_line_x=(main_dc_pos(1)+main_dc_pos(3))/2;%第一条地址线x
first_addr_bit_y=addr_line_res_y+resistor.width+unit_margin;%第一条地址位线y
main_dc_neg_port_pos=get_port_pos(model_name,main_dc,'RConn',1);
main_dc_pos_port_pos=get_port_pos(model_name,main_dc,'LConn',1);

addr_bottom_y=first_addr_bit_y+(matrix_row_height+matrix_addr_row_margin)*addr_bits;%地址线尾y
data_top_y=addr_bottom_y+matrix_addr_row_margin*2;%最上方数据线y
data_bottom_y=data_top_y+matrix_row_height*row_n;   %最下方数据线y

gap_between_vcc_and_ground=2*resistor.width+4*unit_margin; %往下拉电源线和地线，留出中间空间放用于确定输出电压的2个分压电阻

addr_bit_pull_up_r=1e5;
for bi=0:addr_bits-1
    y1=first_addr_bit_y+bi*(matrix_row_height+matrix_addr_row_margin);
    y2=y1+matrix_row_height;
    %反相器
    ngx=first_addr_line_x-not_gate.width-unit_margin;
    ng=add_block_at(model_name,not_gate,['addr bit not gate' num2str(bi)],ngx,y2,[0,0.5],'right','V_IL',num2str(vl),'V_IH',num2str(vh),'V_OL','0','V_OH',num2str(dc_v));
    rsx=ngx-resistor.width-unit_margin*2;
    rs=add_block_at(model_name,resistor,['addr bit resistor' num2str(bi)],rsx,y1,[0,0.5],'right','R',num2str(addr_bit_pull_up_r));
    swx=rsx-elec_switch.width-unit_margin*2;
    sw=add_block_at(model_name,elec_switch,['switch' num2str(bi)],swx,y2,[0,sw_port_y_rel],'left');
    oc1=add_block_at(model_name,open_circuit,['addr bit open circuit' num2str(bi)],matrix_right_x+unit_margin,y1,[0,0.5],'right');
    oc2=add_block_at(model_name,open_circuit,['addr bit open circuit not' num2str(bi)],matrix_right_x+unit_margin,y2,[0,0.5],'right');
    add_line(model_name,[rs,'/RConn1'],[oc1,'/LConn1'],'autorouting','off');
    add_line(model_name,[ng,'/RConn1'],[oc2,'/LConn1'],'autorouting','off');
    add_line(model_name,[sw,'/LConn1'],[ng,'/LConn1'],'autorouting','off');
    add_line(model_name,[sw,'/LConn1'],[rs,'/RConn1'],'autorouting','smart');
    if bi==0
        %         %用坐标连线，效果不好
        %         ps=[rsx,y1;rsx,main_dc_pos_port_pos(2);main_dc_pos_port_pos];
        %         add_line(model_name,ps)
        %         sw_port_pos=get_port_pos(model_name,sw,'RConn',2);
        %         ps=[sw_port_pos;sw_port_pos(1),main_dc_neg_port_pos(2);main_dc_neg_port_pos];
        %         add_line(model_name,ps)
        add_line(model_name,[rs,'/LConn1'],[main_dc,'/LConn1'],'autorouting','smart');
        add_line(model_name,[sw,'/RConn2'],[main_dc,'/RConn1'],'autorouting','smart');
    else
        add_line(model_name,[rs,'/LConn1'],[last_rs,'/LConn1'],'autorouting','off');
        add_line(model_name,[sw,'/RConn2'],[last_sw,'/RConn2'],'autorouting','off');
    end
    last_rs=rs;
    last_sw=sw;
    
    spx=swx-sim_ps.width-unit_margin*2-gap_between_vcc_and_ground;
    sw_port_pos=get_port_pos(model_name,sw,'RConn',1);
    sp=add_block_at(model_name,sim_ps,['sim to ps' num2str(bi)],spx,sw_port_pos(2),[0,0.5],'right');
    add_line(model_name,[sp,'/RConn1'],[sw,'/RConn1'],'autorouting','off');
    
    cvx=spx-converter.width-unit_margin*2;
    cv=add_block_at(model_name,converter,['converter' num2str(bi)],cvx,sw_port_pos(2),[0,0.5],'right');
    add_line(model_name,[cv,'/1'],[sp,'/1'],'autorouting','off');
    
    bex=cvx-bits_extractor.width-unit_margin*2;
    be=add_block_at(model_name,bits_extractor,['bit extrator' num2str(bi)],bex,sw_port_pos(2),[0,0.5],'right','bitsToExtract','Range of bits','bitIdxRange',num2str(bi));
    add_line(model_name,[be,'/1'],[cv,'/1'],'autorouting','off');
    
    if bi==0
        ctx=bex-counter.width-unit_margin*2;
        ct=add_block_at(model_name,counter,['counter'],ctx,sw_port_pos(2),[0,0.5],'right','uplimit',num2str(col_n),'tsamp',num2str(ts));
    end
    add_line(model_name,[ct,'/1'],[be,'/1'],'autorouting','smart');
    
    if is_demo && (bi==0 || bi==addr_bits-1)
        pause(demo_pause_time);
    end
end

sw_port_pos=get_port_pos(model_name,last_sw,'RConn',2);
ground_line_x=sw_port_pos(1);
oc_ground=add_block_at(model_name,open_circuit,'ground open circuit',ground_line_x,data_bottom_y,[0.5,0],'down');
add_line(model_name,[oc_ground,'/LConn1'],[sw,'/RConn2'],'autorouting','on');
vcc_line_x=ground_line_x-gap_between_vcc_and_ground;
oc_vcc=add_block_at(model_name,open_circuit,'vcc open circuit',vcc_line_x,data_bottom_y,[0.5,0],'down');
% add_line(model_name,[vcc_line_x,data_bottom_y;vcc_line_x,main_dc_pos_port_pos(2);main_dc_pos_port_pos]);%vcc端连不上
add_line(model_name,[oc_vcc,'/LConn1'],[main_dc,'/LConn1'],'autorouting','smart');

addr_line_pull_up_r=1e3;%地址线上拉电阻大小
for ci=0:col_n-1
    rx=first_addr_line_x+ci*matrix_col_width;
    %上拉电阻
    rs=add_block_at(model_name,resistor,['data bit resistor' num2str(ci)],rx,addr_line_res_y,[0.5,0],'down','R',num2str(addr_line_pull_up_r));
    if ci==0
        add_line(model_name,[rs,'/LConn1'],[main_dc,'/LConn1'],'autorouting','off');
    else
        add_line(model_name,[rs,'/LConn1'],[last_rs,'/LConn1'],'autorouting','off');
    end
    last_rs=rs;
    
    %开路
    oc=add_block_at(model_name,open_circuit,['data bit open circuit' num2str(ci)],rx,data_bottom_y,[0.5,0],'down');
    add_line(model_name,[rs,'/RConn1'],[oc,'/LConn1'],'autorouting','off');
    
    %地址二极管
    for bi=0:addr_bits-1
        y1=first_addr_bit_y+bi*(matrix_row_height+matrix_addr_row_margin);
        y2=y1+matrix_row_height;
        dio_x=rx+matrix_margin_h;
        dio_y=y1+matrix_margin_v;
        dio=add_block_at(model_name,diode,['addr diode', num2str(bi), '_',num2str(ci)],dio_x,dio_y,[0,0],'right','Vf',num2str(diode_v),'CJ','0');
        add_line(model_name,[rx,dio_y+diode.height/2;dio_x,dio_y+diode.height/2]);
        if bitget(ci,bi+1)==0
            add_line(model_name,[dio_x+diode.width,dio_y+diode.height/2;dio_x+diode.width,y1]);
        else
            add_line(model_name,[dio_x+diode.width,dio_y+diode.height/2;dio_x+diode.width,y2]);
        end
    end
    if is_demo && (ci==0 || ci==col_n-1)
        pause(demo_pause_time);
    end
end


data_line_pull_r=1e5;
output_level_base_r=1e3;%避免短路，上拉电阻基本值
output_level_total_r=1e4;%分压电阻之和
output_level_diodo_offset_r=diode_v/( dc_v/(output_level_base_r+output_level_total_r));%电平转换二极管有压降，将输出抬升一下
output_level_total_r=output_level_total_r-output_level_diodo_offset_r;

amp_r=1e6;% 同相加法器的电阻

for ri=0:row_n-1
    y1=data_top_y+matrix_row_height*ri;
    bfx=first_addr_line_x-resistor.width-buffer.width-unit_margin;
    bf=add_block_at(model_name,buffer,['data line buffer', num2str(ri), '_',num2str(ci)],bfx,y1,[0,0.5],'left','V_IL',num2str(vl),'V_IH',num2str(vh),'V_OL','0','V_OH',num2str(dc_v));
    oc=add_block_at(model_name,open_circuit,['data line open circuit' num2str(ri)],matrix_right_x+unit_margin,y1,[0,0.5],'right');
    add_line(model_name,[bf,'/LConn1'],[oc,'/LConn1'],'autorouting','off');
    diox=bfx-unit_margin-diode.width;
    dio=add_block_at(model_name,diode,['data line diode', num2str(ri), '_',num2str(ci)],diox,y1,[0,0.5],'right','Vf',num2str(diode_v),'CJ','0');
    add_line(model_name,[dio,'/RConn1'],[bf,'/RConn1'],'autorouting','off');
    
    rsx=first_addr_line_x-resistor.width-matrix_margin_h*2;
    rs=add_block_at(model_name,resistor,['data line resistor' num2str(ri)],rsx,y1+matrix_margin_v,[0,0],'left','R',num2str(data_line_pull_r));
    rs_pos=get_port_pos(model_name,rs,'LConn',1);
    add_line(model_name,[rs_pos;rs_pos(1)+matrix_margin_h,rs_pos(2);rs_pos(1)+matrix_margin_h,y1]);
    rs_pos=get_port_pos(model_name,rs,'RConn',1);
    add_line(model_name,[rs_pos;ground_line_x,rs_pos(2)]);
    
    y2=rs_pos(2);
    
    %输出电压的分压电阻，低
    rs_ol_r=(row_n-ri)/row_n*output_level_total_r;
    rs_ol_x=ground_line_x-resistor.width-unit_margin;
    rs_ol=add_block_at(model_name,resistor,['output level resistor low' num2str(ri)],rs_ol_x,y2,[0,0.5],'right','R',num2str(output_level_diodo_offset_r+ rs_ol_r));
    rs_oh_x=vcc_line_x+unit_margin;
    rs_oh=add_block_at(model_name,resistor,['output level resistor high' num2str(ri)],rs_oh_x,y2,[0,0.5],'right','R',num2str(output_level_base_r+output_level_total_r-rs_ol_r));
    add_line(model_name,[ground_line_x-unit_margin,y2;ground_line_x,rs_pos(2)]);
    add_line(model_name,[rs_ol,'/LConn1'],[rs_oh,'/RConn1'],'autorouting','off');
    add_line(model_name,[rs_ol,'/LConn1'],[dio,'/LConn1'],'autorouting','smart');
    add_line(model_name,[vcc_line_x,y2;vcc_line_x+unit_margin,rs_pos(2)]);
    
    %运放电阻
    amp_res_in_x=vcc_line_x-unit_margin*2-resistor.width;
    amp_res_in_level=add_block_at(model_name,resistor,['amp input level resistor' num2str(ri)],amp_res_in_x,y1,[0,0.5],'right','R',num2str(amp_r));
    amp_res_in_sin=add_block_at(model_name,resistor,['amp input sin resistor' num2str(ri)],amp_res_in_x,y2,[0,0.5],'right','R',num2str(amp_r));
    add_line(model_name,[amp_res_in_level,'/RConn1'],[dio,'/LConn1'],'autorouting','off');
    add_line(model_name,[amp_res_in_level,'/LConn1'],[amp_res_in_sin,'/LConn1'],'autorouting','off');
    if ri>0
        add_line(model_name,[amp_res_in_sin,'/RConn1'],[last_amp_res_in_sin,'/RConn1'],'autorouting','off');
    end
    last_amp_res_in_sin=amp_res_in_sin;
    amp_res_out_low_x=amp_res_in_x-unit_margin*2-resistor.width;
    amp_res_out_low=add_block_at(model_name,resistor,['amp output low resistor' num2str(ri)],amp_res_out_low_x,y1,[0,0.5],'right','R',num2str(amp_r));
    amp_res_out_high_x=amp_res_out_low_x-unit_margin*2-resistor.width;
    amp_res_out_high=add_block_at(model_name,resistor,['amp output high resistor' num2str(ri)],amp_res_out_high_x,y1,[0,0.5],'right','R',num2str(amp_r));
    add_line(model_name,[amp_res_out_high,'/RConn1'],[amp_res_out_low,'/LConn1'],'autorouting','off');
    if ri>0
        add_line(model_name,[amp_res_out_low,'/RConn1'],[last_amp_res_out_low,'/RConn1'],'autorouting','off');
    end
    last_amp_res_out_low=amp_res_out_low;
    
    %运放
    amp=add_block_at(model_name,amplifier,['amp' num2str(ri)],amp_res_out_high_x,y2,[0,amp_port_pos_y_rel],'right','BlockRotation',num2str(180));
    add_line(model_name,[amp,'/LConn1'],[amp_res_in_sin,'/LConn1'],'autorouting','off');
    add_line(model_name,[amp,'/LConn2'],[amp_res_out_high,'/RConn1'],'autorouting','off');
    add_line(model_name,[amp,'/RConn1'],[amp_res_out_high,'/LConn1'],'autorouting','off');
    
    %电压表
    vsx=amp_res_out_high_x-unit_margin*2-voltage_sensor.width;
    vs=add_block_at(model_name,voltage_sensor,['voltage sensor' num2str(ri)],vsx,y1,[0,0.5],'left');
    add_line(model_name,[vs,'/LConn1'],[amp_res_out_high,'/LConn1'],'autorouting','off');
    if ri>0
        add_line(model_name,[vs,'/RConn2'],[last_vs,'/RConn2'],'autorouting','off');
    end
    last_vs=vs;
    
    %转换器
    p_sx=vsx-unit_margin*2-ps_sim.width;
    p_s=add_block_at(model_name,ps_sim,['ps to simulink' num2str(ri)],p_sx,y1+(-0.5+vs_port_out_y_rel)*voltage_sensor.height,[0,0.5],'left');
    add_line(model_name,[vs,'/RConn1'],[p_s,'/LConn1'],'autorouting','off');
    
    if ri==0
        if is_demo
            pause(demo_pause_time);
        end
        amp_res_in_sin_pos=get_port_pos(model_name,amp_res_in_sin,'RConn',1);
        ac_amp=dc_v/(output_level_base_r+output_level_total_r+output_level_diodo_offset_r)*output_level_total_r/row_n/2;%
        ac=add_block_at(model_name,ac_source,'ac source',amp_res_in_sin_pos(1),data_top_y-matrix_margin_v,[0.5,1],'up','amp',num2str(ac_amp),'frequency',num2str(sin_feq));
        add_line(model_name,[amp_res_in_sin,'/RConn1'],[ac,'/LConn1'],'autorouting','smart');
        add_line(model_name,[ac,'/RConn1'],[last_sw,'/RConn2'],'autorouting','smart');
        add_line(model_name,[ac,'/RConn1'],[amp_res_out_low,'/RConn1'],'autorouting','smart');
        add_line(model_name,[ac,'/RConn1'],[vs,'/RConn2'],'autorouting','smart');
        
        bus_creator.height=data_bottom_y-data_top_y;
        busx=p_sx-unit_margin*2-bus_creator.width;
        bus=add_block_at(model_name,bus_creator,'bus',busx,data_top_y,[0,0],'left','Inputs',num2str(row_n));
        
        dzx=busx-unit_margin*4-dead_zone.width;
        dzy=data_top_y+matrix_margin_v;
        dz=add_block_at(model_name,dead_zone,'dead zone',dzx,dzy,[0,0.5],'left','UpperValue',num2str(diode_v+ac_amp),'LowerValue','-inf');
        add_line(model_name,[bus,'/1'],[dz,'/1'],'autorouting','smart');
        
        scx=dzx-unit_margin-scope.width;
        sc=add_block_at(model_name,scope,'scope',scx,dzy,[0,0.5],'left');
        add_line(model_name,[dz,'/1'],[sc,'/1'],'autorouting','off');
    end
    add_line(model_name,[p_s,'/1'],[bus,'/',num2str(ri+1)],'autorouting','smart');
    
    %数据二极管
    for ci=0:col_n-1
        if img(ri+1,ci+1)
            x=first_addr_line_x+ci*matrix_col_width;
            dio_x=x+matrix_margin_h;
            dio_y=y1+matrix_margin_v;
            dio=add_block_at(model_name,diode,['data diode', num2str(ri), '_',num2str(ci)],dio_x,dio_y,[0,0],'right','Vf',num2str(diode_v),'CJ','0');
            add_line(model_name,[x,dio_y+diode.height/2;dio_x,dio_y+diode.height/2]);
            add_line(model_name,[dio_x+diode.width,dio_y+diode.height/2;dio_x+diode.width,y1]);
            dio_x=rx+matrix_margin_h;
        end
    end
    if is_demo && (ri==0 || ri==row_n-1)
        pause(demo_pause_time);
    end
    
end

if ~is_demo
    open_system(model_name);
end
save_system(model_name);

function p=get_port_pos(model_name,name,port_name,port_i)
    h=get_param([model_name,'/',name],'PortHandles');
    pgroup=getfield(h,port_name);
    if numel(pgroup)==1
        if port_i==1
            p=get_param(pgroup,'Position');
        else
            error 'port_i too large'
        end
    else
        p=get_param(pgroup(port_i),'Position');
    end
end


function name=add_block_at(model_name,binfo,name,x,y,align,orien,varargin)
    ShowName='off';
    if strcmp( orien,'left') || strcmp(orien,'right')
        wh=[binfo.width,binfo.height];
    else
        wh=[binfo.height,binfo.width];
    end
    xy=[x,y]-align.*wh;
    add_block(binfo.name,[model_name,'/', name],'Position', [xy,xy+wh],'Orientation',orien,'ShowName',ShowName,varargin{:});
end


