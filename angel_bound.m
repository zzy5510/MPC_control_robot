function res = angel_bound(x)
    res=x;
    for i=1:size(res,2)
        if res(i)<-pi
            res(i)=res(i)+2*pi;
        end
        if res(i)>pi
           res(i)=res(i)-2*pi;
        end
    end
    for i=1:size(res,1)
        if res(i)<-pi
            res(i)=res(i)+2*pi;
        end
        if res(i)>pi
           res(i)=res(i)-2*pi;
        end
    end
    

end
