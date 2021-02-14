clear;
%可调参数
dt = 0.1; n = 400; P=20;
%初始化
last_theta_r=0;
pose_last=[0;0;0];u1_last=0;u2_last=0;u1=0;u2=0;
pose=[0;1;1];dpose=[0;0;0];dertau=[0;0];Ahat=zeros(3);Bhat=zeros(3,2);
%位置和控制量
x_rs=zeros(1,n);y_rs=zeros(1,n);theta_rs=zeros(1,n);v_rs=zeros(1,n);w_rs=zeros(1,n);%跟踪路径参考量（用于画图）
xs=zeros(1,n);ys=zeros(1,n);thetas=zeros(1,n);vs=zeros(1,n);ws=zeros(1,n);%机器人的位置和输入（用于画图）
x_rp=zeros(P,1);y_rp=zeros(P,1);theta_rp=zeros(P,1);%参考机器人位置（用于预测环节）
v_rp=zeros(P,1);w_rp=zeros(P,1);%参考输入量
es=zeros(3,n);
%初始化待跟踪轨迹
for i=1:P
    x_r = 2*cos(0.5*i*dt);
    y_r = sin(0.5*i*dt);
    vx_r = -sin(0.5*i*dt);
    vy_r =  0.5*cos(0.5*i*dt);
    v_r = sqrt(vx_r^2+vy_r^2);
    theta_r = atan2(vy_r,vx_r);
    if i==1
        last_theta_r=theta_r;
    end
    w_r=angel_bound(theta_r-last_theta_r)/dt;
    last_theta_r=theta_r;
    %直线
    %x_r=2*i*dt;
    %y_r=0;
    %v_r=2;
    %theta_r=0;
    x_rp(i)=x_r;y_rp(i)=y_r;theta_rp(i)=theta_r;v_rp(i)=v_r;w_rp(i)=w_r;
    x_rs(i)=x_r;y_rs(i)=y_r;theta_rs(i)=theta_r;v_rs(i)=v_r;w_rs(i)=v_r;
end
%开始循环
for i=P+1:n
    %正弦
    x_r = 2*cos(0.5*i*dt);
    y_r = sin(0.5*i*dt);
    vx_r = -sin(0.5*i*dt);
    vy_r =  0.5*cos(0.5*i*dt);
    v_r = sqrt(vx_r^2+vy_r^2);
    theta_r = atan2(vy_r,vx_r);
    w_r=angel_bound(theta_r-last_theta_r)/dt;
    
    %直线
    %x_r=2*i*dt;
    %y_r=0;
    %v_r=2;
    %theta_r=0;
    
    pose_r = [x_rp, y_rp, theta_rp];
    pose_r = pose_r';
    %反馈校正
    e=feedback_correction(dt,u1_last,u2_last,pose_last,pose);
    es(:,i)=e;

    %更新用于反馈的量
    pose_last=pose;
    last_theta_r=theta_r;
    
    %预测环节
    [Ahat,Bhat] = predict(theta_rp,v_rp,dt,P);
    %滚动优化
    dertau = optimize(pose,pose_r(:,1),Ahat,Bhat,P,e);
    u1=dertau(1)+v_rp(1);
    u2=dertau(2)+w_rp(1);
    %将控制量输入对象
    pose = car_model(dt,u1,u2,pose);
    %更新参考量
    x_rp=[x_rp(2:P,:);x_r];
    y_rp=[y_rp(2:P,:);y_r];
    theta_rp=[theta_rp(2:P,:);theta_r];
    v_rp=[v_rp(2:P,:);v_r];
    w_rp=[w_rp(2:P,:);w_r];
    
    u1_last=u1;u2_last=u2;

    %将数据记录
    x_rs(i)=x_r;y_rs(i)=y_r;theta_rs(i)=theta_r;v_rs(i)=v_r;w_rs(i)=w_r;
    xs(i-P)=pose(1);ys(i-P)=pose(2);thetas(i-P)=pose(3);vs(i-P)=u1;ws(i-P)=u2;

end
%绘图
grid on;
%对比图
% plot(xs);hold on;plot(x_rs);hold off;
% figure();plot(ys);hold on;plot(y_rs);hold off;
% figure();plot(thetas);hold on;plot(theta_rs);hold off;
% figure();plot(vs);hold on;plot(v_rs);hold off;
% figure();plot(ws);hold on;plot(w_rs);hold off;
%误差图
figure();plot(xs-x_rs);hold on;
figure();plot(ys-y_rs)
figure();plot(angel_bound(thetas-theta_rs))
figure();plot(vs-v_rs)
figure();plot(ws-w_rs)
%figure();plot(es(1,:));
