function[angle]=AdjustAngle(angle)
    %���ֽǶ�ֵλ��-180�ȡ���+180��
    while(abs(angle) > pi)
        angle = angle - sign(angle)*2.0*pi;
    end
    if angle==pi
        angle=angle-2*pi;
    end
end