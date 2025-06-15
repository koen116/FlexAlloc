function [topo_Matrix,rate_Matrix,runtime_topo_band] = get_Varband_WPL_topo(Traf_Dis_Matrix,FSO_num)

rack_num = size(Traf_Dis_Matrix,1);
topo_Matrix = zeros(rack_num);
topo_source_sum = zeros(1,rack_num);
topo_destination_sum = zeros(1,rack_num);
rate_Matrix = zeros(rack_num);
total_rate = 0;


%% 采用通信概率-权重直接排序的方法,能够采用这种方法得益于后面执行的最短路径感知的可变带宽分配,降低复杂度的实现方式
tic
dis_weight_ij = vec(Traf_Dis_Matrix);
[~,sort_index] = sort(dis_weight_ij,'descend');%根据通信概率权重直接降序排列
sort_num = length(sort_index);

%% 初始化一个权重贪婪的连通环
loop_set = [1:rack_num 1];
address_loop_set = 1:rack_num;
for i=1:sort_num
    index = sort_index(i);
    add_destination = ceil(index/rack_num);
    add_source = index-rack_num*(add_destination-1);
    if add_source~=add_destination
        topo_source_sum(add_source) = topo_source_sum(add_source)+1;
        topo_destination_sum(add_destination) = topo_destination_sum(add_destination)+1;
        if topo_source_sum(add_source)>1 || topo_destination_sum(add_destination)>1 || address_loop_set(add_source)>=rack_num %%判断拓扑是否满足条件
            topo_source_sum(add_source) = topo_source_sum(add_source)-1;
            topo_destination_sum(add_destination) = topo_destination_sum(add_destination)-1;
        else
            address_loop_set(loop_set(address_loop_set(add_source)+1)) = address_loop_set(add_destination);
            loop_set(address_loop_set(add_destination)) = loop_set(address_loop_set(add_source)+1);
            loop_set(address_loop_set(add_source)+1) = add_destination;
            address_loop_set(add_destination) = address_loop_set(add_source)+1;
        end
    end
end
loop_set(rack_num+1) = loop_set(1);
for i=1:rack_num
    topo_Matrix(loop_set(i),loop_set(i+1)) = 1;
end

%% 按权重贪婪的方式生成拓扑
for i=1:sort_num
    index = sort_index(i);
    add_destination = ceil(index/rack_num);
    add_source = index-rack_num*(add_destination-1);
    if add_source~=add_destination
        topo_Matrix(add_source,add_destination) = topo_Matrix(add_source,add_destination)+1;
        topo_source_sum(add_source) = topo_source_sum(add_source)+1;
        topo_destination_sum(add_destination) = topo_destination_sum(add_destination)+1;
        if topo_source_sum(add_source)>FSO_num || topo_destination_sum(add_destination)>FSO_num || topo_Matrix(add_source,add_destination)>1 %%判断拓扑是否满足条件
            topo_Matrix(add_source,add_destination) = topo_Matrix(add_source,add_destination)-1;
            topo_source_sum(add_source) = topo_source_sum(add_source)-1;
            topo_destination_sum(add_destination) = topo_destination_sum(add_destination)-1;
        end
    end
end
runtime_topo = toc;

%% 分配速率矩阵
topo_G = digraph(topo_Matrix);%digraph()表示有向图,graph()表示无向图
for i=1:rack_num
    tic
    for j=1:rack_num
        if i~=j
            [path,dis] = shortestpath(topo_G,i,j);%求节点i到节点j的最短路径path,最短长度为dis
            Traf_weight = Traf_Dis_Matrix(i,j);
            if dis ~= Inf
                for k=1:dis
                    rate_Matrix(path(k),path(k+1)) = rate_Matrix(path(k),path(k+1))+Traf_weight;
                    total_rate = total_rate+1;
                end
            end
        end
    end
    runtime_rate = toc;
end

tic
Total_Link_Num = FSO_num*rack_num;
deploy_flag = "unlimited_deploy";
rate_set=[0 3/12:3/12:3];
mn_set=[0 (2:1:13).^2];
M = 64*Total_Link_Num;%M设为足够大
traf_vec = rate_Matrix(topo_Matrix>0).';%根据topo_Matrix将rate_Matrix变换成一行
[allo_band_vec] = get_Varband_allocation_band(traf_vec,rate_set,deploy_flag,mn_set,M,FSO_num,rack_num);%这里分配带宽
rate_Matrix(topo_Matrix>0) = allo_band_vec.';
runtime_band = toc;

runtime_topo_band = runtime_topo+runtime_rate+runtime_band;
end