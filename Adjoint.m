function AdT = Adjoint(T)
R = T(1:3,1:3);
t = T(1:3,end);
AdT = [ R,      zeros(3,3);...
       skew(t) * R,     R];
% AdT = [ R,      skew(t) * R;...
%        zeros(3,3),       R];
end

function rot = skew(theta)
rot = [       0, -theta(3),  theta(2);...
       theta(3),         0, -theta(1);...
      -theta(2),  theta(1),         0];
end