function [flow_comp_time,runtime_Varband_dyn_toporouting,runtime_Varband_dyn_mathching] = get_Varband_Hand_Fabric(flow_arr_time,flow_arr_size,flow_sour_dest,flow_arr_num,Traf_Dis_Matrix,topo_FSONum_perRack,oppo_FSONum_perRack,link_rate,varband_flag,bundle_pktNum,recon_time,lambda)

rack_num = size(Traf_Dis_Matrix,1);
flow_depart_time = zeros(1,flow_arr_num);%流离开时间
flow_comp_time = zeros(1,flow_arr_num);%流完成时间
runtime_Varband_dyn_toporouting = zeros(1,flow_arr_num);%记录拓扑重构的运行时间
runtime_Varband_dyn_mathching = zeros(1,flow_arr_num);%记录匹配的运行时间
dediTopo_schetime_Matrix = zeros(rack_num);%专用固定拓扑实时流量调度时间矩阵
buffer_longflow = 0;
buffer_shortflow = 0;
buffer_flow = [];

clock = 0;%系统时钟
schetime_clock = 0;%机会链路的调度时钟
flow_sche_table = [];%机会链路的流调度任务表
pkt_bundle_size = 1500*bundle_pktNum/1e6;%MB,一个分组1500字节,一个分组捆100个分组
round_time = 1500*bundle_pktNum*8/link_rate;%机会链路的传输持续时间120us
FSONum_perRack = topo_FSONum_perRack+oppo_FSONum_perRack;%每个机架上FSO总数
topo_FSONum_perRack = max(2,min(14,ceil(FSONum_perRack*sum(flow_arr_size(flow_arr_size<pkt_bundle_size))/sum(flow_arr_size))));%当一个机架上链路数为16时,固定拓扑的链路数不超过14; 

old_topo_FSONum_perRack = topo_FSONum_perRack;
oppo_FSONum_perRack = FSONum_perRack-topo_FSONum_perRack;
flow_arr_cycle = 1/lambda;
each_toporecon_cycle = max(300*flow_arr_cycle,150*recon_time);%拓扑重建的周期1e-3s,注意重构时间较大时,取二者最大值
toporecon_cycle = each_toporecon_cycle;
fixed_pkt_bundle_size = pkt_bundle_size;

%% 根据流量分布矩阵得到固定专用链路拓扑和带宽分配
[Dedicated_Topo_Matrix,rate_Matrix,runtime_topo_band] = get_Varband_WPL_topo(Traf_Dis_Matrix,topo_FSONum_perRack);
runtime_Varband_dyn_toporouting(:)=runtime_topo_band;

for flow_index=1:flow_arr_num
    %% 系统时钟
    clock = flow_arr_time(flow_index);

    %% 更新实时流调度任务
    sour_rack = flow_sour_dest(1,flow_index);
    dest_rack = flow_sour_dest(2,flow_index);
    scheduling_flow_size = flow_arr_size(flow_index);

    %% 判断是否存在动态调度
    if varband_flag=="dyn_resources"       
        %% 周期性先前流量矩阵估计(FireFly)
        if scheduling_flow_size<pkt_bundle_size
            buffer_shortflow = buffer_shortflow+scheduling_flow_size;
        else
            buffer_longflow = buffer_longflow+scheduling_flow_size;
        end
        if toporecon_cycle < clock 
            check_ratio = min(1,buffer_shortflow/(buffer_shortflow+buffer_longflow));
            if check_ratio>2/16
                topo_FSONum_perRack = max(2,min(14,ceil(FSONum_perRack*check_ratio)));%固定拓扑的链路数不超过8
                oppo_FSONum_perRack = FSONum_perRack-topo_FSONum_perRack;
                if topo_FSONum_perRack ~= old_topo_FSONum_perRack
                    [Dedicated_Topo_Matrix,rate_Matrix,runtime_topo_band] = get_Varband_WPL_topo(Traf_Dis_Matrix,topo_FSONum_perRack);
                    runtime_Varband_dyn_toporouting(flow_index)=runtime_topo_band;
                    dediTopo_schetime_Matrix(:,:) = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为固定拓扑的重启时间
                    schetime_clock = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为重构拓扑的重启时间
                    old_topo_FSONum_perRack = topo_FSONum_perRack;
                end
            end
            buffer_longflow = 0;
            buffer_shortflow = 0;
            toporecon_cycle = toporecon_cycle+each_toporecon_cycle;
        end
    end

    %% 判断是否存在动态流大小划分
    if varband_flag=="dyn_flowsize"
        buffer_flow = [buffer_flow scheduling_flow_size];
        if toporecon_cycle < clock
            check_ratio = 2/16;
            sorted_buffer_flow = sort(buffer_flow, 'ascend'); %排序数组（升序或降序）
            totalSum = sum(sorted_buffer_flow);%计算总和
            cumulativeSum = cumsum(sorted_buffer_flow);%计算累积和   
            targetSum = check_ratio*totalSum;
            index = find(cumulativeSum >= targetSum, 1);%找到累积和首次达到或超过总和15%的位置
            tmp_pkt_bundle_size = sorted_buffer_flow(index);%单位MB,返回流大小的分界数
            if tmp_pkt_bundle_size<fixed_pkt_bundle_size && pkt_bundle_size~=fixed_pkt_bundle_size %%判断如果低于最低阈值则不改变流阈值,为直达链路保证利用率
                pkt_bundle_size = fixed_pkt_bundle_size;
                dediTopo_schetime_Matrix(:,:) = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为固定拓扑的重启时间
                schetime_clock = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为重构拓扑的重启时间
            else
                if tmp_pkt_bundle_size>=fixed_pkt_bundle_size && pkt_bundle_size~=tmp_pkt_bundle_size
                    pkt_bundle_size = fixed_pkt_bundle_size;
                    dediTopo_schetime_Matrix(:,:) = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为固定拓扑的重启时间
                    schetime_clock = max(flow_depart_time)+recon_time;%以最大的流离开时间+重构时间为重构拓扑的重启时间
                end
            end
            buffer_flow = [];
            toporecon_cycle = toporecon_cycle+each_toporecon_cycle;
        end
    end

    if scheduling_flow_size<pkt_bundle_size %判断一个流内的分组是否存在一捆,采用专用链路还是机会链路处理流
        %% 专用固定链路拓扑处理短流
        [flow_depart_time(flow_index),dediTopo_schetime_Matrix] = get_Varband_kshortest_routing(Dedicated_Topo_Matrix,rate_Matrix,dediTopo_schetime_Matrix,scheduling_flow_size,sour_rack,dest_rack,clock,link_rate);
    else
        %% 机会链路处理长流
        [flow_depart_time_set,flow_index_set,flow_sche_table,schetime_clock,runtime_Varband_dyn_mathching(flow_index)] = get_Varband_mathching(flow_sche_table,schetime_clock,scheduling_flow_size,flow_index,sour_rack,dest_rack,clock,round_time,recon_time,oppo_FSONum_perRack,bundle_pktNum,rack_num,link_rate);
        if ~isempty(flow_depart_time_set)%机会链路按周期调度的,有可能一个周期内有多个流被调度完成
            flow_depart_time(flow_index_set)=flow_depart_time_set;
        end
    end

end   

%% 计算所有流完成时间
flow_comp_time = flow_depart_time-flow_arr_time;

end