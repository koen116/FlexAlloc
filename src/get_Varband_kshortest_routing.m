function [flow_depart_time,dediTopo_schetime_Matrix] = get_Varband_kshortest_routing(Dedicated_Topo_Matrix,rate_Matrix,dediTopo_schetime_Matrix,scheduling_flow_size,sour_rack,dest_rack,clock,link_rate)

flow_depart_time = 0;

%% 计算最短路径
topo_G = digraph(Dedicated_Topo_Matrix);%digraph()表示有向图,graph()表示无向图
[sche_Path,dis] = shortestpath(topo_G,sour_rack,dest_rack);%求节点i到节点j的最短路径path,最短长度为dis

%% 更新实时流量调度时间矩阵,注意————假设只有排队延时和传输延时
for j=1:length(sche_Path)-1
    sch_time_perPath = scheduling_flow_size*1e6*8/(rate_Matrix(sche_Path(j),sche_Path(j+1))*link_rate);%scheduling_flow_size的单位为MB
    if j==1
        esti_clock_perPath = clock;%根据到达时间估计的每段路径的最早到达时间
        if dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1))>esti_clock_perPath
            dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)) = dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1))+sch_time_perPath;
        else
            dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)) = esti_clock_perPath+sch_time_perPath;
        end
        if j==length(sche_Path)-1
            flow_depart_time = max(flow_depart_time,dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)));%计算流的离开时间
        end
    else 
        if dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1))>dediTopo_schetime_Matrix(sche_Path(j-1),sche_Path(j))
            dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)) = dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1))+sch_time_perPath;
        else
            dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)) = dediTopo_schetime_Matrix(sche_Path(j-1),sche_Path(j))+sch_time_perPath;
        end
        if j==length(sche_Path)-1
            flow_depart_time = max(flow_depart_time,dediTopo_schetime_Matrix(sche_Path(j),sche_Path(j+1)));%计算流的离开时间
        end
    end
end

end