function [allo_band_vec] = get_Varband_allocation_linklimit_band(traf_vec,matching_Matrix,rate_set,deploy_flag,mn_set,M,FSONum_perRack,rack_num)
    max_rate=max(rate_set);
    delta_rate=rate_set(3)-rate_set(2);
    mat_index_set = find(matching_Matrix>0).';%以列为单位依次计数
    link_matching_Matrix = 0+matching_Matrix;%逻辑矩阵变为实数矩阵
    min_rate = rate_set(2);
    len=length(traf_vec);
    sum_rate=min(len*FSONum_perRack*max_rate,FSONum_perRack*rack_num);
    allo_band_vec=ones(1,len);
    traf_org=traf_vec;
    traf_vec=sum_rate*traf_vec/sum(traf_vec);
    if isempty(traf_vec)
        return;
    end

    %% Discrete Bandwidth Allocation
    if deploy_flag=="unlimited_deploy"
        %% incremental algorithm
        allo_band_vec=zeros(1,len);
        allo_band_vec(:)=min_rate;
        traf_vec_tmp=allo_band_vec;
        while sum(allo_band_vec)+delta_rate<=sum_rate && sum(traf_vec_tmp(traf_org>0)+1e10)~=0
            allo_band_vec_tmp=traf_vec_tmp+delta_rate;
            allo_band_vec_tmp(allo_band_vec_tmp<0)=-1e10;
            [~,index]=max(traf_org./allo_band_vec_tmp);
            allo_band_vec(index)=allo_band_vec(index)+delta_rate;
            traf_vec_tmp(index)=allo_band_vec(index);
            y = ceil(mat_index_set(index)/rack_num);%第y列
            x = mat_index_set(index)-rack_num*(y-1);%第x行
            link_matching_Matrix(x,y) = ceil(allo_band_vec(index)/max_rate);
            r_xy = max(sum(link_matching_Matrix(x,:)),sum(link_matching_Matrix(:,y)));
            if r_xy>FSONum_perRack
                allo_band_vec(index)=allo_band_vec(index)-delta_rate;
                traf_vec_tmp(index)=-1e10;%%速率达到max_rate时，在下次循环时，该项仍可能最小，将对应的值设置为足够大避免陷入死循环
                link_matching_Matrix(x,y) = ceil(allo_band_vec(index)/max_rate);
            end
        end
    end

end