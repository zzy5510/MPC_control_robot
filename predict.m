function [Ahat,Bhat] = predict(theta_r,v_r,dt,P)
for i=1:P
    A(:,:,i)=[1,0,-v_r(i)*sin(theta_r(i))*dt;
       0,1, v_r(i)*cos(theta_r(i))*dt;
       0,0,                 1];
    B(:,:,i)=[cos(theta_r(i))*dt,0;
       sin(theta_r(i))*dt,0;
       0,                dt];
end
Ahat=zeros(3*P,3);Bhat=zeros(3*P,2*P);
Atemp=1;
for i=1:P
    Atemp=Atemp*A(:,:,i);
    Ahat(3*i-2:3*i,:)=Atemp;
    Btemp=1;
    for k=1:i
        j=i-k+1;
        Bhat(3*i-2:3*i,2*j-1:2*j)=Btemp*B(:,:,j);
        Btemp=Btemp*A(:,:,j);
    end
end

end