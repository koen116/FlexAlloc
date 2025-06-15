function [allo_band_vec] = get_Varband_allocation_band(traf_vec,rate_set,deploy_flag,mn_set,M,FSONum_perRack,rack_num)
    max_rate=max(rate_set);
    delta_rate=rate_set(3)-rate_set(2);
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
        if max_rate*sum(traf_vec>0)+min_rate*sum(traf_vec<=0)<=sum_rate
            allo_band_vec(traf_vec>0)=max_rate;
            allo_band_vec(traf_vec<=0)=min_rate;
        else
            traf_vec_tmp=allo_band_vec;
            while sum(allo_band_vec)<sum_rate && sum(traf_vec_tmp(traf_org>0)+1e10)~=0
                allo_band_vec_tmp=traf_vec_tmp+delta_rate;
                [~,index]=max(traf_org./allo_band_vec_tmp);
                allo_band_vec(index)=allo_band_vec(index)+delta_rate;
                traf_vec_tmp(index)=allo_band_vec(index);
                if allo_band_vec(index)>max_rate
                    allo_band_vec(index)=max_rate;
                    traf_vec_tmp(index)=-1e10;%%速率达到max_rate时，在下次循环时，该项仍可能最小，将对应的值设置为足够大避免陷入死循环
                end
            end
            if sum(allo_band_vec)>sum_rate
                allo_band_vec(index)=allo_band_vec(index)-delta_rate;
            end
        end
    end

end