function e = feedback_correction(dt,u1,u2,pose,pose_now)
p=zeros(3,1);
p(3)=angel_bound(pose(3)+u2*dt);
p(1)=pose(1)+u1*cos(p(3))*dt;
p(2)=pose(2)+u1*sin(p(3))*dt;
e=pose_now-p;
end
