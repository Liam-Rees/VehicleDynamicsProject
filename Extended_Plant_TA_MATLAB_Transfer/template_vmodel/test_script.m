close 'all'

LongLT = (par.mass*par.hcg/(2*par.L))*ax_sim;
LatLT = (par.mass*par.hcg/(4*par.hBf))*ay_sim;

FL = FzF - LongLT - LatLT;
FR = FzF - LongLT + LatLT;
RL = FzR + LongLT - LatLT;
RR = FzR + LongLT + LatLT;

F_total = FL+FR+RL+RR;
F_FL = FL./F_total;
F_FR = FR./F_total;
F_RL = RL./F_total;
F_RR = RR./F_total;

figure()
plot(time, F_FL)
hold on
plot(time, F_RL)
legend('FL','RL')
hold off

figure()
plot(time, squeeze(Tau_FL_sim))
hold on
plot(time, squeeze(Tau_RL_sim))
legend('FL','RL')
hold off