v_steel=10;phi=0.1;
%机器人参数
l=1.3;d=0.3;
%初始位置
xr=zeros(1,20);yr=zeros(1,20);tr=zeros(1,20);
x=0;y=0;theta=0;
v=v_steel*cos(phi);
dtheta=v_steel/l*sin(phi);
for k=1:20
    x=x+v*cos(theta);
    y=y+v*sin(theta);
    theta=theta+dtheta;
    xr(k)=x;
    yr(k)=y;
    tr(k)=theta;
end