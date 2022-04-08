%Vasileios Kyrgyridis 9617, Project in  Robotics 2021
clear all
clc

q0= [2.6180; -0.6695; 1.2719; 3.1416; 1.2002; -0.9821];
lwr = lwr_create(); 
lwr.links(); 
J_e6DOF=lwr.jacobe(q0);
T_akrou_arxiko=lwr.fkine(q0); %pinakas gia na vgalw thn arxiki thesi tou akrou
D = rad2deg(q0); %Metatropi apo gwnies se rad
T_plat_teliko=[1 0 0 2.097557529; 0 1 0 0.845; 0 0 1 0; 0 0 0 1]; %Epithimiti thesi ths platformas se sxesi me to {0}, h opoia vghke mesw geometrias
T_plat_arxiko=eye(4);
g_MB = [1 0 0 0; 0 1 0 0.35; 0 0 1 0.5; 0 0 0 1]; %Statheros Pinakas M/S twn plaisiwn M kai B
g_0M=[1 0 0 2.097557529; 0 1 0 0.845; 0 0 1 0; 0 0 0 1]; %Epithimiti thesi ths platformas se sxesi me to {0}, h opoia vghke mesw geometrias
g_OB=g_0M*g_MB; %Ypologismos ths neas vasis tou robot me skopo na ananewsoume to montelo
lwr.base=g_OB; %Ananewsi vasis montelou
T_akrou_teliko=lwr.fkine(q0); %Teliki thesi akrou
qf=lwr.ikine(T_akrou_teliko,'q0',[2.6180 -0.6695 1.2719 3.1416 1.2002 -0.9821]);

timestep=0.1; %Orizoume to vima
t = 0:timestep:5;
[R_plat_0,t_plat_0] = tr2rt(T_plat_arxiko);
[R_plat_teliko,t_plat_teliko] = tr2rt(T_plat_teliko);
quat_plat_0 = rotm2quat(R_plat_0); %Metatropi twn parapanw pinakwn se quaternions
quatm_plat_teliko = rotm2quat(R_plat_teliko);
[s,sd] = tpoly(0,1,t);
Q_plat_start=UnitQuaternion(R_plat_0); %Simfwna me tous typous tou Peter Corke eftiaksa ta Qstart kai finish gia na parw tis apostaseis kai ta quaternions
Q_plat_final=UnitQuaternion(R_plat_teliko);
for i = 1:length(t)-1
    if i ==length(t)
        Q_plat(length(t))=Q_plat_start.interp(Q_plat_final,1); %Eixa thema me to teleutaio oti to s den anike sto [0,1] epomenws evala auti thn synthiki termatismou
    end
    p_plat(:,i) = t_plat_0*(1-s(i)) + s(i)*t_plat_teliko ; %Symfwna me tous typous tou Peter Corke evgala tis theseis
    Q_plat(i)=Q_plat_start.interp(Q_plat_final,(s(i)));
end


[R_akrou_0,t_akrou_0] = tr2rt(T_akrou_arxiko);
[R_akrou_teliko,t_akrou_teliko] = tr2rt(T_akrou_teliko);
quat_akrou_0 = rotm2quat(R_akrou_0); %Metatropi twn parapanw pinakwn se quaternions
quatm_akrou_teliko = rotm2quat(R_akrou_teliko);
Q_akrou_start=UnitQuaternion(R_akrou_0); %Simfwna me tous typous tou Peter Corke eftiaksa ta Qstart kai finish gia na parw tis apostaseis kai ta quaternions
Q_akrou_final=UnitQuaternion(R_akrou_teliko);
for i = 1:length(t)-1
    if i ==length(t)
        Q_akrou(length(t))=Q_akrou_start.interp(Q_akrou_final,1); %Eixa thema me to teleutaio oti to s den anike sto [0,1] epomenws evala auti thn synthiki termatismou
    end
    p_akrou(:,i) = t_akrou_0*(1-s(i)) + s(i)*t_akrou_teliko ; %Symfwna me tous typous tou Peter Corke evgala tis theseis
    Q_akrou(i)=Q_akrou_start.interp(Q_akrou_final,(s(i)));
end


quat_akrou_cyl = rotm2quat(R_akrou_0); %Metatropi twn parapanw pinakwn se quaternions
quatm_akrou_cyl = rotm2quat(R_akrou_teliko);
Q_akrou_start_cyl=UnitQuaternion(R_akrou_0); %Simfwna me tous typous tou Peter Corke eftiaksa ta Qstart kai finish gia na parw tis apostaseis kai ta quaternions
Q_akrou_final_cyl=UnitQuaternion(R_akrou_teliko);
t_akrou_arxiko_cyl=p_akrou(:,(length(t)-1));
t_akrou_final_cyl=[1.5 1.5 0.7];

for i = 1:length(t)-1
    if i ==length(t)
        Q_akrou_cyl(length(t))=Q_akrou_start_cyl.interp(Q_akrou_final_cyl,1); %Eixa thema me to teleutaio oti to s den anike sto [0,1] epomenws evala auti thn synthiki termatismou
    end
    p_akrou_cyl(:,i) = t_akrou_arxiko_cyl'*(1-s(i)) + s(i)*t_akrou_final_cyl ; %Symfwna me tous typous tou Peter Corke evgala tis theseis
    Q_akrou_cyl(i)=Q_akrou_start_cyl.interp(Q_akrou_final_cyl,(s(i)));
end

T_move = rt2tr(Q_akrou_cyl.R(), p_akrou_cyl);
T_move_new = SE3.convert(T_move);
qtr=lwr.ikine(T_move_new,'q0',qf);

q_A=qtr(length(qtr),:);
T_A=lwr.fkine(q_A');
[R_0A,t_0A] = tr2rt(T_A);
t_0An=[2.097557573; 1.5400000255; 0.7+0.01];
T_A1=rt2tr(R_0A,t_0An);

timestep=0.1;
tn = 0:timestep:2;
[R0n,t0n] = tr2rt(T_A);
[Rfn,tfn] = tr2rt(T_A1);
quat0n = rotm2quat(R0n);
quatfn = rotm2quat(Rfn);
[sn,sd] = tpoly(0,1,tn);
p0n=transl(T_A);
pfn=tfn';
Qstartn=UnitQuaternion(R0n);
Qfinaln=UnitQuaternion(Rfn);
for i = 1:length(tn)-1
    if i ==length(tn)
        Q_0fn(length(tn))=Qstartn.interp(Qfinaln,1);
    end
    p0fn(:,i) = p0n*(1-sn(i)) + sn(i)*pfn ;
    Q_0fn(i)=Qstartn.interp(Qfinaln,(s(i)));
end
Rn=Q_0fn.R();
Tpa = rt2tr(Rn, p0fn);
Tpa_new=SE3.convert(Tpa);
qtr1=lwr.ikine(Tpa_new,'q0',q_A);

q_cd=qtr1(length(qtr1),:);
T_CD=lwr.fkine(q_cd');
[R_CD,t_CD] = tr2rt(T_CD);
t_cd=[2.692200322; 0.845; 0.7+0.01];
T_CD_n=rt2tr(R_CD,t_cd);


timestep=0.1;
tcd = 0:timestep:3;
[R0cd,t0cd] = tr2rt(T_CD);
[Rfcd,tfcd] = tr2rt(T_CD_n);
quat0cd = rotm2quat(R0cd);
quatfcd = rotm2quat(Rfcd);
[scd,sd_cd] = tpoly(0,1,tcd);
p0cd=transl(T_CD);
pfcd=tfcd';
Qstart_cd=UnitQuaternion(R0cd);
Qfinal_cd=UnitQuaternion(Rfcd);
for i = 1:length(tcd)-1
    if i ==length(tcd)
        Q_0cd(length(tcd))=Qstart_cd.interp(Qfinal_cd,1);
    end
    p0_cd(:,i) = p0cd*(1-scd(i)) + scd(i)*pfcd ;
    Q_0cd(i)=Qstart_cd.interp(Qfinal_cd,(scd(i)));
end
R_cd=Q_0cd.R();
T_cdnew = rt2tr(R_cd, p0_cd);
Tcd_new2=SE3.convert(T_cdnew);
qtr2=lwr.ikine(Tcd_new2,'q0',q_cd);

%Idia diadikasia me prin gia na vgei to 3o kommati tou 4ou erwtimatos

q_dm=qtr2(length(qtr2),:);
T_DM=lwr.fkine(q_dm');
[R_DM,t_DM] = tr2rt(T_DM);
t_dm=[2.097755753; 0.845; 0.5];
T_DM_n=rt2tr(R_DM,t_dm);

timestep=0.1;
tdm = 0:timestep:3;
[R0dm,t0dm] = tr2rt(T_DM);
[Rfdm,tfdm] = tr2rt(T_DM_n);
quat0dm = rotm2quat(R0dm);
quatfdm = rotm2quat(Rfdm);
[sdm,sd_dm] = tpoly(0,1,tdm);
p0dm=transl(T_DM);
pfdm=tfdm';
Qstart_dm=UnitQuaternion(R0dm);
Qfinal_dm=UnitQuaternion(Rfdm);
for i = 1:length(tdm)-1
    if i ==length(tdm)
        Q_0dm(length(tdm))=Qstart_dm.interp(Qfinal_dm,1);
    end
    p0_dm(:,i) = p0dm*(1-sdm(i)) + sdm(i)*pfdm ;
    Q_0dm(i)=Qstart_dm.interp(Qfinal_dm,(sdm(i)));
end
R_dm=Q_0dm.R();
T_dmnew = rt2tr(R_dm, p0_dm);
Tdm_new2=SE3.convert(T_dmnew);
qtr3=lwr.ikine(Tdm_new2,'q0',q_dm);

%Plot gia troxies


p_akrou_z_plot=zeros(length(p_akrou),1);
for i=1:length(p_akrou_z_plot)
    p_akrou_z_plot(i,1)=0.5;
end
syntet_00=zeros(length(qtr(:,1,1)),3);
syntet_0=zeros(length(qtr(:,1,1)),3);
for i=1:length(qtr(:,1,1))
    syntet_plot_0=lwr.fkine([qtr(i,1,1) qtr(i,2,1) qtr(i,3,1) qtr(i,4,1) qtr(i,5,1) qtr(i,6,1)]);
    syntet_0(i,1)=syntet_plot_0.t(1);
    syntet_0(i,2)=syntet_plot_0.t(2);
    syntet_0(i,3)=syntet_plot_0.t(3);
    
end
syntet_1=zeros(length(qtr1(:,1,1)),3);
for i=1:length(qtr1(:,1,1))
    syntet_plot_1=lwr.fkine([qtr1(i,1,1) qtr1(i,2,1) qtr1(i,3,1) qtr1(i,4,1) qtr1(i,5,1) qtr1(i,6,1)]);
    syntet_1(i,1)=syntet_plot_1.t(1);
    syntet_1(i,2)=syntet_plot_1.t(2);
    syntet_1(i,3)=syntet_plot_1.t(3);
    
end
syntet_2=zeros(length(qtr2(:,1,1)),3);
for i=1:length(qtr2(:,1,1))
    syntet_plot_2=lwr.fkine([qtr2(i,1,1) qtr2(i,2,1) qtr2(i,3,1) qtr2(i,4,1) qtr2(i,5,1) qtr2(i,6,1)]);
    syntet_2(i,1)=syntet_plot_2.t(1);
    syntet_2(i,2)=syntet_plot_2.t(2);
    syntet_2(i,3)=syntet_plot_2.t(3);
    
end
syntet_3=zeros(length(qtr3(:,1,1)),3);
for i=1:length(qtr3(:,1,1))
    syntet_plot_3=lwr.fkine([qtr3(i,1,1) qtr3(i,2,1) qtr3(i,3,1) qtr3(i,4,1) qtr3(i,5,1) qtr3(i,6,1)]);
    syntet_3(i,1)=syntet_plot_3.t(1);
    syntet_3(i,2)=syntet_plot_3.t(2);
    syntet_3(i,3)=syntet_plot_3.t(3);
    
end

%5o erwtima : Visualisation
%1h kinisi (2o erwtima)
%Kinisi platformas
for i=1:length(t)-1

    r=0.025; %Dimiourgia Cylinder
    [X,Y,Z] = cylinder(r);
    h=mesh(X+1.5,Y+1.5,Z*0.1+0.6,'facecolor',[1 0 0]);
    plat=plotcube([0.75 1 0.5],[p_plat(1,i)-0.375 p_plat(2,i)-0.5 0],.8,[0 0 1]); %Dimiourgia platformas mesw ths plotcube
    table=plotcube([0.35 0.35 0.6],[1.325 1.325 0],.8,[0 0 1]); % Dimiourgia trapeziou mesw ths plotcube
    lwr.base=SE3(p_plat(1,i), p_plat(2,i)+0.35, 0.5);% Ananewsi vashs wste na kineitai to robot
    lwr.plot(q0','workspace',[-2 3 -1 3 0 1.6],'floorlevel',0,'nobase','noname'); %Sximatismos ths kinisis tou robot
    if i ~= length(t)-1
        delete(plat);
    end 
end

%Kinisi Akrou
lwr.plot([qtr(:,1,1) qtr(:,2,1) qtr(:,3,1) qtr(:,4,1) qtr(:,5,1) qtr(:,6,1)],'workspace',[-2 3 -1 3 0 1.6],'floorlevel',0,'nobase','noname');

%4o erwtima (Xwristike se 3 kiniseis)
delete(h); %Arxika diagrafi tou cylinder kathws den tha fainetai mesa sto akro tou vraxiona
%1o kommati kinisis apo A->C
lwr.plot([qtr1(:,1,1) qtr1(:,2,1) qtr1(:,3,1) qtr1(:,4,1) qtr1(:,5,1) qtr1(:,6,1)],'workspace',[-2 3 -1 3 0 1.6],'floorlevel',0,'nobase','noname');
%2o kommati kinisis apo C->D
lwr.plot([qtr2(:,1,1) qtr2(:,2,1) qtr2(:,3,1) qtr2(:,4,1) qtr2(:,5,1) qtr2(:,6,1)],'workspace',[-2 3 -1 3 0 1.6],'floorlevel',0,'nobase','noname');
%3o kommati kinisis apo D->M
lwr.plot([qtr3(:,1,1) qtr3(:,2,1) qtr3(:,3,1) qtr3(:,4,1) qtr3(:,5,1) qtr3(:,6,1)],'workspace',[-2 3 -1 3 0 1.6],'floorlevel',0,'nobase','noname');
hold on

plot3(p_plat(1,:)+p_akrou(1,1,1), p_plat(2,:)+p_akrou(2,1,1)+0.35,p_akrou_z_plot'+p_akrou(3,1,1),'b','LineWidth',3)
hold on
plot3(p_akrou_cyl(1,:), p_akrou_cyl(2,:), p_akrou_cyl(3,:),'b','LineWidth',3)
hold on
plot3(syntet_0(:,1),syntet_0(:,2),syntet_0(:,3),'b','LineWidth',3)
hold on
plot3(syntet_1(:,1),syntet_1(:,2),syntet_1(:,3),'b','LineWidth',3)
hold on
plot3(syntet_2(:,1),syntet_2(:,2),syntet_2(:,3),'b','LineWidth',3)
hold on
plot3(syntet_3(:,1),syntet_3(:,2),syntet_3(:,3),'b','LineWidth',3)

figure
q1_plot=[q0(1); qf(1); qtr(:,1); qtr1(:,1); qtr2(:,1); qtr3(:,1)];
plot(q1_plot)
title('Apokrisi q1')
figure 
plot(diff(q1_plot))
title('Taxuthta q1')
figure
q2_plot=[q0(2); qf(2); qtr(:,2); qtr1(:,2); qtr2(:,2); qtr3(:,2)];
plot(q2_plot)
title('Apokrisi q2')
figure
plot(diff(q2_plot))
title('Taxuthta q2')
figure
q3_plot=[q0(3); qf(3); qtr(:,3); qtr1(:,3); qtr2(:,3); qtr3(:,3)];
plot(q3_plot)
title('Apokrisi q3')
figure
plot(diff(q3_plot))
title('Taxuthta q3')
figure
q4_plot=[q0(4); qf(4); qtr(:,4); qtr1(:,4); qtr2(:,4); qtr3(:,4)];
plot(q4_plot)
title('Apokrisi q4')
figure
plot(diff(q4_plot))
title('Taxuthta q4')
figure
q5_plot=[q0(5); qf(5); qtr(:,5); qtr1(:,5); qtr2(:,5); qtr3(:,5)];
plot(q5_plot)
title('Apokrisi q5')
figure
plot(diff(q5_plot))
title('Taxuthta q5')
figure
q6_plot=[q0(6); qf(6); qtr(:,6); qtr1(:,6); qtr2(:,6); qtr3(:,6)];
plot(q6_plot)
title('Apokrisi q6')
figure
plot(diff(q6_plot))
title('Taxuthta q6')


x_akrou_plot=[p_plat(1,:)+p_akrou(1,1,1) p_akrou_cyl(1,:) syntet_0(:,1)' syntet_1(:,1)' syntet_2(:,1)' syntet_3(:,1)'];
y_akrou_plot=[ p_plat(2,:)+p_akrou(2,1,1)+0.35  p_akrou_cyl(2,:) syntet_0(:,2)' syntet_1(:,2)' syntet_2(:,2)' syntet_3(:,2)'];
z_akrou_plot=[p_akrou_z_plot'+p_akrou(3,1,1) p_akrou_cyl(3,:) syntet_0(:,3)' syntet_1(:,3)' syntet_2(:,3)' syntet_3(:,3)'];
figure
plot(x_akrou_plot)
title('Apokrisi x')
figure
plot(y_akrou_plot)
title('Apokrisi y')
figure
plot(z_akrou_plot)
title('Apokrisi z')
