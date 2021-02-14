clear;
%�ɵ�����
dt = 0.1; n = 400; P=20;
%��ʼ��
last_theta_r=0;
pose_last=[0;0;0];u1_last=0;u2_last=0;u1=0;u2=0;
pose=[0;1;1];dpose=[0;0;0];dertau=[0;0];Ahat=zeros(3);Bhat=zeros(3,2);
%λ�úͿ�����
x_rs=zeros(1,n);y_rs=zeros(1,n);theta_rs=zeros(1,n);v_rs=zeros(1,n);w_rs=zeros(1,n);%����·���ο��������ڻ�ͼ��
xs=zeros(1,n);ys=zeros(1,n);thetas=zeros(1,n);vs=zeros(1,n);ws=zeros(1,n);%�����˵�λ�ú����루���ڻ�ͼ��
x_rp=zeros(P,1);y_rp=zeros(P,1);theta_rp=zeros(P,1);%�ο�������λ�ã�����Ԥ�⻷�ڣ�
v_rp=zeros(P,1);w_rp=zeros(P,1);%�ο�������
es=zeros(3,n);
%��ʼ�������ٹ켣
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
    %ֱ��
    %x_r=2*i*dt;
    %y_r=0;
    %v_r=2;
    %theta_r=0;
    x_rp(i)=x_r;y_rp(i)=y_r;theta_rp(i)=theta_r;v_rp(i)=v_r;w_rp(i)=w_r;
    x_rs(i)=x_r;y_rs(i)=y_r;theta_rs(i)=theta_r;v_rs(i)=v_r;w_rs(i)=v_r;
end
%��ʼѭ��
for i=P+1:n
    %����
    x_r = 2*cos(0.5*i*dt);
    y_r = sin(0.5*i*dt);
    vx_r = -sin(0.5*i*dt);
    vy_r =  0.5*cos(0.5*i*dt);
    v_r = sqrt(vx_r^2+vy_r^2);
    theta_r = atan2(vy_r,vx_r);
    w_r=angel_bound(theta_r-last_theta_r)/dt;
    
    %ֱ��
    %x_r=2*i*dt;
    %y_r=0;
    %v_r=2;
    %theta_r=0;
    
    pose_r = [x_rp, y_rp, theta_rp];
    pose_r = pose_r';
    %����У��
    e=feedback_correction(dt,u1_last,u2_last,pose_last,pose);
    es(:,i)=e;

    %�������ڷ�������
    pose_last=pose;
    last_theta_r=theta_r;
    
    %Ԥ�⻷��
    [Ahat,Bhat] = predict(theta_rp,v_rp,dt,P);
    %�����Ż�
    dertau = optimize(pose,pose_r(:,1),Ahat,Bhat,P,e);
    u1=dertau(1)+v_rp(1);
    u2=dertau(2)+w_rp(1);
    %���������������
    pose = car_model(dt,u1,u2,pose);
    %���²ο���
    x_rp=[x_rp(2:P,:);x_r];
    y_rp=[y_rp(2:P,:);y_r];
    theta_rp=[theta_rp(2:P,:);theta_r];
    v_rp=[v_rp(2:P,:);v_r];
    w_rp=[w_rp(2:P,:);w_r];
    
    u1_last=u1;u2_last=u2;

    %�����ݼ�¼
    x_rs(i)=x_r;y_rs(i)=y_r;theta_rs(i)=theta_r;v_rs(i)=v_r;w_rs(i)=w_r;
    xs(i-P)=pose(1);ys(i-P)=pose(2);thetas(i-P)=pose(3);vs(i-P)=u1;ws(i-P)=u2;

end
%��ͼ
grid on;
%�Ա�ͼ
% plot(xs);hold on;plot(x_rs);hold off;
% figure();plot(ys);hold on;plot(y_rs);hold off;
% figure();plot(thetas);hold on;plot(theta_rs);hold off;
% figure();plot(vs);hold on;plot(v_rs);hold off;
% figure();plot(ws);hold on;plot(w_rs);hold off;
%���ͼ
figure();plot(xs-x_rs);hold on;
figure();plot(ys-y_rs)
figure();plot(angel_bound(thetas-theta_rs))
figure();plot(vs-v_rs)
figure();plot(ws-w_rs)
%figure();plot(es(1,:));
