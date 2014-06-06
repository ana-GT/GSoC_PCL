%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate derivative code for Jacobian and Hessian stuff

syms a b c e1 e2 px py pz ra pa ya 

% Local Variables
syms nx ny nz ox oy oz ax ay az

% Input per each point
syms x y z

% A very coarse way to enter the rotation matrix
nx = cos(ya)*cos(pa);
ny = sin(ya)*cos(pa);
nz = -sin(pa);

ox = cos(ya)*sin(pa)*sin(ra) - sin(ya)*cos(ra);
oy = sin(ya)*sin(pa)*sin(ra) + cos(ya)*cos(ra);
oz = cos(pa)*sin(ra);

ax = cos(ya)*sin(pa)*cos(ra) + sin(ya)*sin(ra);
ay = sin(ya)*sin(pa)*cos(ra) - cos(ya)*sin(ra);
az = cos(pa)*cos(ra);

F = ( ( ( (nx*x + ny*y + nz*z - px*nx - py*ny - pz*nz)/a  )^(2))^(1.0/e2) + ( ( (ox*x + oy*y + oz*z - px*ox - py*oy - pz*oz)/b  )^(2))^(1.0/e2) )^(e2 / e1) + ( ( (ax*x + ay*y + az*z - px*ax - py*ay - pz*az)/c  )^(2))^(1.0/e1);    
Er = ( F^e1 - 1 )^2;
Res = sqrt(a*b*c)*( (F^e1-1)^2 );

% Jacobian
%J : jacobian( [Res],[a,b,c, e1,e2, px,py,pz, ra,pa,ya] )$
%H : hessian( Res, [a,b,c, e1,e2, px,py,pz, ra,pa,ya] )$
J = [ diff(Res,a), diff(Res,b), diff(Res,c), diff(Res,e1), diff(Res,e2), diff(Res,px), diff(Res,py), diff(Res,pz), diff(Res,ra), diff(Res,pa), diff(Res,ya)];

% Hessian
H = sym( zeros(11,11) );
% H(1,:) = [diff(J(1),a), diff(J(2),a),diff(J(3),a), diff(J(4),a),diff(J(5),a), diff(J(6),a),diff(J(7),a), diff(J(8),a),diff(J(9),a), diff(J(10),a),diff(J(11),a)];
% H(2,:) = [diff(J(1),b), diff(J(2),b),diff(J(3),b), diff(J(4),b),diff(J(5),b), diff(J(6),b),diff(J(7),b), diff(J(8),b),diff(J(9),b), diff(J(10),b),diff(J(11),b)];
% H(3,:) = [diff(J(1),c), diff(J(2),c),diff(J(3),c), diff(J(4),c),diff(J(5),c), diff(J(6),c),diff(J(7),c), diff(J(8),c),diff(J(9),c), diff(J(10),c),diff(J(11),c)];
% H(4,:) = [diff(J(1),e1), diff(J(2),e1),diff(J(3),e1), diff(J(4),e1),diff(J(5),e1), diff(J(6),e1),diff(J(7),e1), diff(J(8),e1),diff(J(9),e1), diff(J(10),e1),diff(J(11),e1)];
% H(5,:) = [diff(J(1),e2), diff(J(2),e2),diff(J(3),e2), diff(J(4),e2),diff(J(5),e2), diff(J(6),e2),diff(J(7),e2), diff(J(8),e2),diff(J(9),e2), diff(J(10),e2),diff(J(11),e2)];
% H(6,:) = [diff(J(1),px), diff(J(2),px),diff(J(3),px), diff(J(4),px),diff(J(5),px), diff(J(6),px),diff(J(7),px), diff(J(8),px),diff(J(9),px), diff(J(10),px),diff(J(11),px)];
% H(7,:) = [diff(J(1),py), diff(J(2),py),diff(J(3),py), diff(J(4),py),diff(J(5),py), diff(J(6),py),diff(J(7),py), diff(J(8),py),diff(J(9),py), diff(J(10),py),diff(J(11),py)];
% H(8,:) = [diff(J(1),pz), diff(J(2),pz),diff(J(3),pz), diff(J(4),pz),diff(J(5),pz), diff(J(6),pz),diff(J(7),pz), diff(J(8),pz),diff(J(9),pz), diff(J(10),pz),diff(J(11),pz)];
% H(9,:) = [diff(J(1),ra), diff(J(2),ra),diff(J(3),ra), diff(J(4),ra),diff(J(5),ra), diff(J(6),ra),diff(J(7),ra), diff(J(8),ra),diff(J(9),ra), diff(J(10),ra),diff(J(11),ra)];
% H(10,:) = [diff(J(1),pa), diff(J(2),pa),diff(J(3),pa), diff(J(4),pa),diff(J(5),pa), diff(J(6),pa),diff(J(7),pa), diff(J(8),pa),diff(J(9),pa), diff(J(10),pa),diff(J(11),pa)];
% H(11,:) = [diff(J(1),ya), diff(J(2),ya),diff(J(3),ya), diff(J(4),ya),diff(J(5),ya), diff(J(6),ya),diff(J(7),ya), diff(J(8),ya),diff(J(9),ya), diff(J(10),ya),diff(J(11),ya)];

% Error function: Er

% C Code Generation
ccode(Res,'file','funcEq.txt');
%ccode(J,'file','jacobianEq.txt');
%ccode(H,'file','HessianEq.txt');
%ccode(Er, 'file','errorEq.txt');