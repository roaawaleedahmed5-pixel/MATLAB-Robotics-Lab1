function A = dhStd(a, alpha, d, theta)
% Standard DH: Rz(theta)*Tz(d)*Tx(a)*Rx(alpha).
c = cos(theta); s = sin(theta);
ca = cos(alpha); sa = sin(alpha);
A = [c -s*ca  s*sa a*c;
     s  c*ca -c*sa a*s;
     0  sa    ca   d;
     0  0     0    1];
end
