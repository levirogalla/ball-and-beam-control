%% Outer-loop controller design by partial pole placement
%  Outer controller: (Ca s + Cb) / (s^2 + Cc s + Cd)
clear; clc

%% ---- settings ----
plantVals = [-2.452 -135.4 24.95 -1.8906 0.018107 0.0609 -4.25];  % C1 C2 C3 P1 P2 K2 K3
fastMult  = 5;                       % 2 fast poles at -fastMult*zeta*Wn
inRegion  = @(r) all(real(r) < 0);    % allowed region for ALL closed-loop poles

zetaRange = [0.28 3];
WnRange   = [0.1 2];
wRange    = [-1 3];                   % log10 of frequency range, rad/s

Wn0   = 0.253535;                      % fixed Wn for the zeta sweep
zeta0 = 2.06586;                     % fixed zeta for the Wn sweep

designWn = 0.94;                     % design point to check at the end
designZeta   = 0.994343;

stepSize     = 0.15;                     % reference step size
Vmax         = 6;                     % motor voltage limit
tauPrefilter = 0;                     % reference prefilter 1/(tau*s+1)^2; 0 = none
minSigma     = 4/7;                   % voltage map only keeps zeta*Wn >= this
                                      % (4/7 ~ 7 s settling, from Ts ~ 4/(zeta*Wn))

%% ---- numeric inner plant (monic denominator) ----
syms s C1 C2 C3 P1 P2 K2 K3
innerController = (C1*s + C2) / (s + C3);
motorPlant      = P1 / (P2*s^2 + s);
Lin             = innerController * motorPlant;
innerPlant      = Lin/(1 + Lin) * K2 * K3/s^2;
innerPlant      = subs(innerPlant, [C1 C2 C3 P1 P2 K2 K3], plantVals);

[Np, Dp] = numden(simplifyFraction(innerPlant));
lead = coeffs(Dp, s, 'All');
Np = expand(Np / lead(1));
Dp = expand(Dp / lead(1));

%% ---- outer controller and pole placement ----
% 4 controller unknowns -> choose 4 poles (2 dominant + 2 fast), 3 left over
syms Ca Cb Cc Cd zeta Wn a2 a1 a0
Nc = Ca*s + Cb;
Dc = s^2 + Cc*s + Cd;
charActual = collect(expand(Dc*Dp + Nc*Np), s);          % 7th order

p = fastMult*zeta*Wn;
charDesired = (s^2 + 2*zeta*Wn*s + Wn^2) * (s + p)^2 * (s^3 + a2*s^2 + a1*s + a0);
charDesired = collect(expand(charDesired), s);

c1 = coeffs(charActual,  s, 'All');
c2 = coeffs(charDesired, s, 'All');
vars = [Ca Cb Cc Cd a2 a1 a0];
sol  = solve(c1(2:end) == c2(2:end), vars);              % 7 equations, 7 unknowns

% closed loop: T = Nc*Np / (Dc*Dp + Nc*Np), and that denominator is charDesired
NcSol   = subs(Nc, [Ca Cb], [sol.Ca sol.Cb]);
charSol = subs(charDesired, [a2 a1 a0], [sol.a2 sol.a1 sol.a0]);
T = NcSol * Np / charSol;

H      = matlabFunction(T, 'Vars', {s, zeta, Wn});                       % H(s, zeta, Wn)
cf     = matlabFunction(coeffs(charSol, s, 'All'), 'Vars', {zeta, Wn});  % char. poly coeffs
isGood = @(z, wn) checkPoles(cf(z, wn), inRegion);

%% ---- peak motor voltage for a step reference ----
% V/R = F * [Cout/(1 + Cout*Gp)] * [Cin/(1 + Cin*Pm)]
Cin = subs(innerController, [C1 C2 C3], plantVals(1:3));
Pm  = subs(motorPlant,      [P1 P2],    plantVals(4:5));
Vr  = NcSol * Dp / charSol * Cin/(1 + Cin*Pm);
if tauPrefilter > 0
    Vr = Vr / (tauPrefilter*s + 1)^2;
end
[nV, dV] = numden(simplifyFraction(Vr));
fnV = matlabFunction(coeffs(nV, s, 'All'), 'Vars', {zeta, Wn});
fdV = matlabFunction(coeffs(dV, s, 'All'), 'Vars', {zeta, Wn});

zm = linspace(zetaRange(1), zetaRange(2), 100);
wm = linspace(WnRange(1),   WnRange(2),   100);
[Zm, WNm] = ndgrid(zm, wm);
peakV = nan(size(Zm));
for k = 1:numel(Zm)
    if Zm(k)*WNm(k) >= minSigma && isGood(Zm(k), WNm(k))
        sysV = tf(fnV(Zm(k), WNm(k)), fdV(Zm(k), WNm(k)));
        peakV(k) = stepSize * max(abs(step(sysV)));
    end
end

figure
contourf(wm, zm, peakV, 20, 'LineColor', 'none'), hold on
contour(wm, zm, peakV, [Vmax Vmax], 'r', 'LineWidth', 2)
plot(wm, minSigma./wm, 'k--', 'LineWidth', 1.5)       % zeta*Wn = minSigma boundary
ylim(zetaRange)
caxis([0 5*Vmax])                     % values above 5x the limit show as the top colour
xlabel('\omega_n'), ylabel('\zeta')
cb = colorbar;  cb.Label.String = 'Peak voltage (V)';
title(sprintf('Peak voltage for a %g step (red = %g V, dashed: \\zeta\\omega_n = %.3g)', ...
      stepSize, Vmax, minSigma))

%% ---- magnitude vs frequency and zeta (Wn fixed); black = outside region ----
w  = logspace(wRange(1), wRange(2), 400);
zv = linspace(zetaRange(1), zetaRange(2), 60);
[W, Z] = meshgrid(w, zv);
M = 20*log10(abs(H(1j*W, Z, Wn0)));
good = arrayfun(@(z) isGood(z, Wn0), zv);
plotMasked(W, Z, M, good, '\zeta', sprintf('Varying \\zeta  (\\omega_n = %g)', Wn0))

%% ---- magnitude vs frequency and Wn (zeta fixed); black = outside region ----
wnv = linspace(WnRange(1), WnRange(2), 60);
[W, WN] = meshgrid(w, wnv);
M = 20*log10(abs(H(1j*W, zeta0, WN)));
good = arrayfun(@(wn) isGood(zeta0, wn), wnv);
plotMasked(W, WN, M, good, '\omega_n', sprintf('Varying \\omega_n  (\\zeta = %g)', zeta0))

%% ---- 4D: frequency, zeta, Wn as axes, magnitude as colour ----
% shows only points inside the pole region with magnitude below 0 dB
w3  = logspace(wRange(1), wRange(2), 80);
zv3 = linspace(zetaRange(1), zetaRange(2), 30);
wn3 = linspace(WnRange(1), WnRange(2), 30);

[Wg, Zg, WNg] = meshgrid(w3, zv3, wn3);            % dims: zeta x omega x Wn
M = 20*log10(abs(H(1j*Wg, Zg, WNg)));

[Zs, WNs] = ndgrid(zv3, wn3);
good   = arrayfun(isGood, Zs, WNs);                % zeta x Wn
stable = repmat(permute(good, [1 3 2]), 1, numel(w3), 1);
keep   = stable & M < 0;

figure
scatter3(Wg(keep), Zg(keep), WNg(keep), 10, M(keep), 'filled')
set(gca, 'XScale', 'log')
xlabel('\omega (rad/s)'), ylabel('\zeta'), zlabel('\omega_n')
title('Inside pole region and magnitude < 0 dB')
colormap(parula)
cb = colorbar;
cb.Label.String = 'Magnitude (dB)';
grid on, view(-35, 25)

%% ---- check the chosen design point ----
gains   = double(subs([sol.Ca sol.Cb sol.Cc sol.Cd], ...
                      [zeta Wn], [designZeta designWn]));
ctrlNum = gains(1:2);
ctrlDen = [1 gains(3:4)];
clPoles = roots(cf(designZeta, designWn));

fprintf('\nDesign point: zeta = %g, Wn = %g\n', designZeta, designWn)
fprintf('Controller numerator:   [%s]\n', num2str(ctrlNum, '%.5g  '))
fprintf('Controller denominator: [%s]\n', num2str(ctrlDen, '%.5g  '))
disp('Closed-loop poles:'),  disp(clPoles)
disp('Controller poles:'),   disp(roots(ctrlDen))
if ~inRegion(clPoles)
    warning('Design point has closed-loop poles outside the region.')
end

%% ---- local functions (must stay at the end of the script) ----
function ok = checkPoles(c, inRegion)
    ok = all(isfinite(c)) && inRegion(roots(c));
end

function plotMasked(X, Y, M, good, yLabel, ttl)
    mask  = repmat(good(:), 1, size(M, 2));        % one row per zeta/Wn value
    Mgood = M;  Mgood(~mask) = NaN;
    Mbad  = M;  Mbad(mask)   = NaN;

    figure
    surf(X, Y, Mgood, 'EdgeColor', 'none'), hold on
    surf(X, Y, Mbad, 'FaceColor', 'k', 'EdgeColor', 'none')
    set(gca, 'XScale', 'log')
    lims = [min(Mgood(:)) max(Mgood(:))];
    if all(isfinite(lims)) && lims(2) > lims(1)
        caxis(lims)                                % colours set by the good region
    end
    xlabel('\omega (rad/s)'), ylabel(yLabel), zlabel('Magnitude (dB)')
    title(ttl)
    cb = colorbar;
    cb.Label.String = 'Magnitude (dB)';
    view(-35, 30)
end