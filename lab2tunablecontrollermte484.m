
clear; clc;
s = tf('s');

% plant
P = -104.413/(s^2 + 55.227*s);

% existing controller
C0 = tunableTF('C', 1, 1);

C0.Numerator.Value = [0.3633 -411.89];
C0.Denominator.Value = [1 56.769];

T0 = feedback(P*C0, 1);
T0.InputName = 'r';
T0.OutputName = 'y';

% Desired response
tau = 0.0625;
OS_target = 2.0;  % percent

Req = TuningGoal.StepTracking('r', 'y', tau, OS_target);

[Tuned, fSoft] = systune(T0, Req);

C_tuned = getBlockValue(Tuned, 'C');

disp('Tuned controller:');
C_tuned

%verify
P = ss(P);
C_tuned = tf(C_tuned);
T = feedback(P*C_tuned, 1);
U = feedback(C_tuned, P); 

t = 0:0.0001:1;
[y, t] = step(1.4*T, t);
info = stepinfo(y, t, 1.4, 'SettlingTimeThreshold', 0.02);

figure;
plot(t, y);
grid on;
yline(1.4*1.02, '--');
yline(1.4*0.98, '--');
xlabel('Time (s)');
ylabel('Output change (rad)');
title('Tuned Closed-Loop Step Response');

fprintf('Settling time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);

% check voltage
u = step(1.4*U, t);
fprintf('Maximum absolute voltage: %.3f V\n', max(abs(u)));
