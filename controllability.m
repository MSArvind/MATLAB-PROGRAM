m1=72000;
wn1=2*pi*3.28;
k1=m1*wn1^2;
zeta1=1.5/100;
c1=zeta1*2*sqrt(k1*m1);
mu=2/100;
m2=mu*m1;
g=9.81;
%simulink model running
sim('projectq8')
step_amp=1000;
m2mm=1000;
time=ans.data.time
exe=ans.data.signals.values(:,1)
acc=ans.data.signals.values(:,2)
force=ans.force.signals.values
subplot(211);hold on
plot(time,m2mm*step_amp*exe)
ylabel('exercusion mm')
subplot(212);hold on
plot(time,100/g*step_amp*acc)
ylabel('acceleration,%g')
%model of the system
A=[0 0 1 0;0 0 0 1;-k1/m1 0 c1/m1 0;k1/m1 0 c1/m1 0]
B=[0 0;0 0;1/m1 -1/m1;-1/m1 (m1+m2)/(m1*m2)]
B_control=B(:,2)
B_force=B(:,1)
C=[1 0 0 0;0 1 0 0]
C_structure=C(1,:)
C_dis=C(2,:)
D=zeros(2,2)
C_struc_acc=A(3,:)
C_struc_exe=A(2,:)
D_struc_acc=B(3,1)
sys_exe=ss(A,B_force,C_dis,0);
sys_struc_acc=ss(A,B_force,C_struc_acc,D_struc_acc)
subplot(211);
exc=lsim(m2mm*step_amp*sys_exe,force,time)
plot(time,exc)
subplot(212);
acc=lsim(100/g*step_amp*sys_struc_acc,force,time)
plot(time,acc)
hold off
controllability=rank(ctrb(A,B_control))
wn_des=8;
p1=wn_des*(-cosd(25)+sind(25)*i)
p2=conj(p1)
p3=wn_des*(-cosd(75)+sind(75)*i)
p4=conj(p3)
p_des=[p1;p2;p3;p4]
k=place(A,B_control,p_des)
Acl=A-B_control*k
sys_cntrl_exe=ss(Acl,B_force,C_dis,0);
cntl_exe=lsim(m2mm*step_amp*sys_cntrl_exe,force,time)
subplot(211);%plot(time,cntl_exe);
sys_cntrl_acc=ss(Acl,B_force,C_struc_acc,D_struc_acc)
cntrl_acc=lsim(100/g*step_amp*sys_cntrl_acc,force,time)
subplot(212);%plot(time,cntrl_acc);hold off
%frequency response
[num_acc,den_acc]=ss2tf(A,B_force,C_struc_acc,D_struc_acc)
tf_acc=tf(num_acc,den_acc)
bode(tf_acc)
%observability
ob=rank(obsv(A,C_struc_acc))
c_struc_acc_exe=[-k1/m1 0 c1/m1 0;0 0 0 1;]%c matrix
d_struc_acc_exe=[1/m1 -1/m1;0 0]
L=10*p_des
obs_gain=place(A,C_struc_acc',L)
A_obs=A-B_control*obs_gain



