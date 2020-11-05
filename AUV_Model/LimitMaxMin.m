function data=LimitMaxMin(data,max,min)
    if data>max
        data=max;
    elseif data<min
        data=min;
    end
end