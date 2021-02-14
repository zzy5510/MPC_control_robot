function uhat = optimize(pose,pose_r,Ahat,Bhat,P,e)
Q=1*diag(ones(3*P,1));R=0.01*diag(ones(2*P,1));

h=0.3;
%误差量
xe=pose-pose_r;
xe(3)=angel_bound(xe(3));
%求解
uhat=-inv(Bhat'*Q*Bhat+R)*Bhat'*Q*Ahat*(xe-h*e);
%方法二：使用qb方程求解（可以限制控制量的范围）
% 控制量ut的上下限
% lb=-10*ones(2*P,1);
% ub=10*ones(2*P,1);
% H=(Bhat'*Q*Bhat + R);
% f=Bhat'*Q*Ahat;
%f=f*xe;
%[uhat, fval, exitflag]=quadprog(H,f',[],[],[],[],lb,ub);

end