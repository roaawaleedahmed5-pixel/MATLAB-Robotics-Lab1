function [Q,valid,reason] = scaraIK(x,y,z,yaw,p)
% Columns contain the +sin(q2) and -sin(q2) branches respectively.
% Elbow-up/down labels depend on the drawing and viewing direction.
% Targets refer to the tool, including its 5 cm offset.
Q = nan(4,2); valid = false(1,2); reason = 'Outside planar reach';
c2 = (x*x+y*y-p.L1^2-p.L2^2)/(2*p.L1*p.L2);
if abs(c2) > 1+1e-12
    return;
end
c2 = min(1,max(-1,c2)); % Only remove floating-point roundoff.
q3 = z-p.h+p.toolDrop;
for b = 1:2
    signBranch = 3-2*b;
    q2 = atan2(signBranch*sqrt(max(0,1-c2*c2)),c2);
    q1 = atan2(y,x)-atan2(p.L2*sin(q2),p.L1+p.L2*cos(q2));
    q4 = yaw-q1-q2;
    q = [q1;q2;q3;q4];
    q([1 2 4]) = atan2(sin(q([1 2 4])),cos(q([1 2 4])));
    Q(:,b) = q;
    valid(b) = all(q >= p.limits(:,1)-1e-10 & q <= p.limits(:,2)+1e-10);
end
if any(valid)
    reason = 'At least one valid branch';
else
    reason = 'Geometric solution exists, but joint limits reject both branches';
end
end
