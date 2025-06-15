function [flow_depart_time_set,flow_index_set,flow_sche_table,schetime_clock,runtime_Varband_dyn_mathching] = get_Varband_mathching(flow_sche_table,schetime_clock,scheduling_flow_size,flow_index,sour_rack,dest_rack,clock,round_time,recon_time,oppo_FSONum_perRack,bundle_pktNum,rack_num,link_rate)

flow_depart_time_set = [];
flow_index_set = [];
runtime_Varband_dyn_mathching = 0;%记录运行时间

%% 更新任务调度表
priority = 0;
scheduling_pkt_num = floor(scheduling_flow_size*1e6/1500)+(rem(scheduling_flow_size*1e6,1500)>0);%计算一个流中的分组数量
flow_sche_table = [flow_sche_table;flow_index sour_rack dest_rack scheduling_pkt_num clock priority];%流调度表:1.流索引;2.源机架;3.目的机架;4.流中分组数;5.流到达时间;6.流优先级
flow_sche_table(:,6) = clock-flow_sche_table(:,5);%更新各个流的优先级

%% 比较调度时钟和流到达时钟的差异
if clock-schetime_clock>=round_time+recon_time %比较调度时钟和流到达时钟的差异
    recon_num = floor((clock-schetime_clock)/(round_time+recon_time));%机会链路重构的次数
    [~,max_index] = max(flow_sche_table(:,6));%根据优先级降序排列
    recon_start_num =  max(0,floor((flow_sche_table(max_index,5)-schetime_clock)/(round_time+recon_time)));%计算当前最晚的重构调度,节约时间
    schetime_clock = schetime_clock+recon_start_num*(round_time+recon_time);%更新调度时钟
    %%%%%%%%还需要确定哪些流的分组捆在一个重构时期内被调度
   
    for i=recon_start_num+1:recon_num
        tic
        [~,sort_index] = sort(flow_sche_table(:,6),'descend');%根据优先级降序排列
        flow_num = length(sort_index);
        schetime_clock_flag = 0;
        if flow_num==1
            schetime_clock = schetime_clock+(recon_num-i+1)*(round_time+recon_time);%调度列表只有一个任务时,更新调度时钟
            break;
        end
        match_Matrix = zeros(rack_num);%每轮需要初始化匹配矩阵
        match_traf_Matrix = zeros(rack_num);%每轮需要初始化匹配的流量矩阵
        match_rate_Matrix = zeros(rack_num);%每轮需要初始化带宽分配矩阵
        sche_flow_index_set = [];
        sche_flow_size_set = [];
        del_index = [];%%初始化需要删除的行
        for j=1:flow_num
            index = sort_index(j);
            if flow_sche_table(index,5)>schetime_clock+round_time+recon_time %流调度表中的流到达时间和schetime_clock差在round_time+recon_time之下,才能进行处理
                break;
            end
            schetime_clock_flag = 1;
            sour = flow_sche_table(index,2);
            dest = flow_sche_table(index,3);
            match_Matrix(sour,dest) = match_Matrix(sour,dest)+1;%假设一捆分组只能用一条链路传输
            if max(sum(match_Matrix(sour,:)),sum(match_Matrix(:,dest)))<=oppo_FSONum_perRack
                sche_flow_index_set = [sche_flow_index_set index];
                pkt_num = flow_sche_table(index,4);
                sche_flow_size_set = [sche_flow_size_set pkt_num];
                match_traf_Matrix(sour,dest) = match_traf_Matrix(sour,dest)+pkt_num;%可能存在相同源-目的对的多个流
            else
                match_Matrix(sour,dest) = match_Matrix(sour,dest)-1;%跟匹配矩阵限制存在冲突,去掉该连接,继续下一次循环
                continue;%跟匹配矩阵限制存在冲突,继续下一次循环
            end
        end
        %% 根据缓存的分组数计算传输带宽矩阵
        Total_Link_Num = oppo_FSONum_perRack*rack_num;
        deploy_flag = "unlimited_deploy";
        rate_set=[0 3/12:3/12:3];
        mn_set=[0 (2:1:13).^2];
        M = 64*Total_Link_Num;%M设为足够大
        traf_vec = match_traf_Matrix(match_traf_Matrix>0).';%变换成一行,match_rate_Matrix中元素初始值为buffer分组数量
        matching_Matrix = match_traf_Matrix>0;
        [allo_band_vec] = get_Varband_allocation_linklimit_band(traf_vec,matching_Matrix,rate_set,deploy_flag,mn_set,M,oppo_FSONum_perRack,rack_num);%这里分配带宽
        match_rate_Matrix(match_traf_Matrix>0) = allo_band_vec.';
        runtime_Varband_dyn_mathching=toc;

        %% 提前更新调度时钟,因为前面的floor少算了一个round_time+recon_time
        if schetime_clock_flag==1
            schetime_clock = schetime_clock+round_time+recon_time;%更新调度时钟
        end

        %% 根据缓存的分组数分配传输带宽
        sche_flow_num = length(sche_flow_size_set);
        for k=1:sche_flow_num
            sche_index = sche_flow_index_set(k);
            pkt_num = flow_sche_table(sche_index,4);
            match_rate = match_rate_Matrix(flow_sche_table(sche_index,2),flow_sche_table(sche_index,3));
            sche_pktNum = floor(match_rate*bundle_pktNum);%一个源ToR分配可用的FSO资源,match_rate_Matrix中对应的元素为1时则为bundle_pktNum
            flow_sche_table(sche_index,4) = max(0,pkt_num-sche_pktNum);%不同流之间的分组不合成为一捆,因为不同流按等待时长定优先级
            if max(0,pkt_num-sche_pktNum)==0
                    flow_index_set = [flow_index_set flow_sche_table(sche_index,1)];%存储第flow_index个流的索引
                    flow_depart_time_set = [flow_depart_time_set schetime_clock+pkt_num*1500*8/(match_rate*link_rate)+recon_time];%计算第flow_index个流的离开时间
                    del_index = [del_index sche_index];%记录需要删除的行
            end
        end

        %% 删除第flow_index个流的调度任务
        flow_sche_table(del_index,:) = [];
    end
end

end