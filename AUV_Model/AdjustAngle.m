function[angle]=AdjustAngle(angle)
    %保持角度值位于-180度——+180度
    while(abs(angle) > pi)
        angle = angle - sign(angle)*2.0*pi;
    end
    if angle==pi
        angle=angle-2*pi;
    end
end