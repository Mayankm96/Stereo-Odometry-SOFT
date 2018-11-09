function [E_all, R_all, t_all, Eo_all] = fivePointAlgorithm( pts1, pts2, K1, K2 )
% Five Point Algorithm:Given five points matches between two images, and the
% intrinsic parameters of each camera. Estimate the essential matrix E, the 
% rotation matrix R and translation vector t, between both images. This 
% algorithm is based on the method described by David Nister in "An 
% Efficient Solution to the Five-Point Relative Pose Problem"
% DOI: http://dx.doi.org/10.1109/TPAMI.2004.17
%
% E = fivePointAlgorithm(pts1, pts2, K1, K2) returns in E all the valid
% Essential matrix solutions for the five point correspondence. If you
% don't need the R and t, use this version as it avoids computing
% unnecessary results.
%
% [E_all, R_all, t_all, Eo_all] = fivePointAlgorithm(pts1, pts2, K1, K2) 
% also returns in R_all and t_all all the rotation matrices and translation
% vectors of camera 2 for the different essential matrices, such that a 3D
% point in camera 1 reference frame can be transformed into the camera 2
% reference frame through p_2 = R{n}*p_1 + t{n}. Eo_all is the essential
% matrix before the imposing the structure U*diag([1 1 0])*V'. It should
% help get a better feeling on the accuracy of the solution. All these
% return values a nx1 cell arrays. 
%
%
% Arguments:
% pts1, pts2 - assumed to have dimension 3x5 and of equal size. 
% K1, K2 - 3x3 intrinsic parameters of cameras 1 and 2 respectively
%
% Know Issues:
% - R and t computation is done assuming perfect point correspondence.
%
% Author: Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com 
% Date: Feb 2015
% Last modified: Mar 2015
% Version: 0.9
% Repo: https://github.com/SergioRAgostinho/fivePointAlgorithm
% Feel free to provide feedback or contribute.

if ~all(size(pts1) == [3,5]) || ~all(size(pts2) == [3,5])
    error('fivePointAlgorithm:wrong_dimensions','pts1 and pts2 must be of size 3x5');
end

if ~all(size(K1) == [3, 3]) || ~all(size(K2) == [3, 3])
    error('fivePointAlgorithm:wrong_dimensions','K1 and K2 must be of size 3x3');
end

q1 = K1 \ pts1;
q2 = K2 \ pts2;

q = [q1(1,:)'.* q2(1,:)', q1(2,:)'.* q2(1,:)', q1(3,:)'.* q2(1,:)', ...
     q1(1,:)'.* q2(2,:)', q1(2,:)'.* q2(2,:)', q1(3,:)'.* q2(2,:)', ...
     q1(1,:)'.* q2(3,:)', q1(2,:)'.* q2(3,:)', q1(3,:)'.* q2(3,:)'];
 
%according to the author, the null space step can be further optimized, 
%following the efficiency considerations in section 3.2.1
% Can be further expand it to N > 5 by extracting the four singular vectors
% corresponding to the four smallest singular values.
nullSpace = null(q); 
X = nullSpace(:,1);
Y = nullSpace(:,2);
Z = nullSpace(:,3);
W = nullSpace(:,4);

% populating the equation system
mask = [1,2,3;4,5,6;7,8,9];
Xmat = X(mask);
Ymat = Y(mask);
Zmat = Z(mask);
Wmat = W(mask);
X_ = (K2') \ Xmat / K1;
Y_ = (K2') \ Ymat / K1;
Z_ = (K2') \ Zmat / K1;
W_ = (K2') \ Wmat / K1;

%det(F)
detF = p2p1(p1p1([X_(1,2),Y_(1,2),Z_(1,2),W_(1,2)], ...
                 [X_(2,3),Y_(2,3),Z_(2,3),W_(2,3)]) ...
          - p1p1([X_(1,3),Y_(1,3),Z_(1,3),W_(1,3)], ...
                 [X_(2,2),Y_(2,2),Z_(2,2),W_(2,2)]),...
                 [X_(3,1),Y_(3,1),Z_(3,1),W_(3,1)]) + ...
       p2p1(p1p1([X_(1,3),Y_(1,3),Z_(1,3),W_(1,3)], ...
                 [X_(2,1),Y_(2,1),Z_(2,1),W_(2,1)]) ...
          - p1p1([X_(1,1),Y_(1,1),Z_(1,1),W_(1,1)], ...
                 [X_(2,3),Y_(2,3),Z_(2,3),W_(2,3)]),...
                 [X_(3,2),Y_(3,2),Z_(3,2),W_(3,2)]) + ...
       p2p1(p1p1([X_(1,1),Y_(1,1),Z_(1,1),W_(1,1)], ...
                 [X_(2,2),Y_(2,2),Z_(2,2),W_(2,2)]) ...
          - p1p1([X_(1,2),Y_(1,2),Z_(1,2),W_(1,2)], ...
                 [X_(2,1),Y_(2,1),Z_(2,1),W_(2,1)]),...
                 [X_(3,3),Y_(3,3),Z_(3,3),W_(3,3)]);
             
%Flipped V
EE_t11 = p1p1([Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)], ...
              [Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)]) + ...
         p1p1([Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)], ...
              [Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)]) + ...
         p1p1([Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)], ...
              [Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)]);
EE_t12 = p1p1([Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)], ...
              [Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)]) + ...
         p1p1([Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)], ...
              [Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)]) + ...
         p1p1([Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)], ...
              [Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)]);
EE_t13 = p1p1([Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)], ...
              [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]) + ...
         p1p1([Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)], ...
              [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]) + ...
         p1p1([Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)], ...
              [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);
EE_t22 = p1p1([Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)], ...
              [Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)]) + ...
         p1p1([Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)], ...
              [Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)]) + ...
         p1p1([Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)], ...
              [Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)]);
EE_t23 = p1p1([Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)], ...
              [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]) + ...
         p1p1([Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)], ...
              [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]) + ...
         p1p1([Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)], ...
              [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);
EE_t33 = p1p1([Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)], ...
              [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]) + ...
         p1p1([Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)], ...
              [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]) + ...
         p1p1([Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)], ...
              [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);

A_11 = EE_t11 - 0.5*(EE_t11 + EE_t22 + EE_t33);
A_12 = EE_t12;
A_13 = EE_t13;
A_21 = A_12;
A_22 = EE_t22 - 0.5*(EE_t11 + EE_t22 + EE_t33);
A_23 = EE_t23;
A_31 = A_13;
A_32 = A_23;
A_33 = EE_t33 - 0.5*(EE_t11 + EE_t22 + EE_t33);

AE_11 = p2p1(A_11, [Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)]) + ...
        p2p1(A_12, [Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)]) + ...
        p2p1(A_13, [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]);
AE_12 = p2p1(A_11, [Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)]) + ...
        p2p1(A_12, [Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)]) + ...
        p2p1(A_13, [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]);
AE_13 = p2p1(A_11, [Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)]) + ...
        p2p1(A_12, [Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)]) + ...
        p2p1(A_13, [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);
AE_21 = p2p1(A_21, [Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)]) + ...
        p2p1(A_22, [Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)]) + ...
        p2p1(A_23, [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]);
AE_22 = p2p1(A_21, [Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)]) + ...
        p2p1(A_22, [Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)]) + ...
        p2p1(A_23, [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]);
AE_23 = p2p1(A_21, [Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)]) + ...
        p2p1(A_22, [Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)]) + ...
        p2p1(A_23, [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);
AE_31 = p2p1(A_31, [Xmat(1,1),Ymat(1,1),Zmat(1,1),Wmat(1,1)]) + ...
        p2p1(A_32, [Xmat(2,1),Ymat(2,1),Zmat(2,1),Wmat(2,1)]) + ...
        p2p1(A_33, [Xmat(3,1),Ymat(3,1),Zmat(3,1),Wmat(3,1)]);
AE_32 = p2p1(A_31, [Xmat(1,2),Ymat(1,2),Zmat(1,2),Wmat(1,2)]) + ...
        p2p1(A_32, [Xmat(2,2),Ymat(2,2),Zmat(2,2),Wmat(2,2)]) + ...
        p2p1(A_33, [Xmat(3,2),Ymat(3,2),Zmat(3,2),Wmat(3,2)]);
AE_33 = p2p1(A_31, [Xmat(1,3),Ymat(1,3),Zmat(1,3),Wmat(1,3)]) + ...
        p2p1(A_32, [Xmat(2,3),Ymat(2,3),Zmat(2,3),Wmat(2,3)]) + ...
        p2p1(A_33, [Xmat(3,3),Ymat(3,3),Zmat(3,3),Wmat(3,3)]);
    
% Group and permute the collumns of our polynomial vectors to prepare for
% the Gaussian Jordan elimination with partial pivoting
% Previously our 3rd order polynomial vector arrangement was like this 
% x^3 | y^3 | z^3 | x^2y | xy^2 | x^2z | xz^2 | y^2z | yz^2 | xyz
% x^2 | y^2 | z^2 | xy | xz | yz | x | y | z | 1
% and now we are going to need this
% x^3 | y^3 | x^2y | xy^2 | x^2z | x^2 | y^2z | y^2 | xyz | xy
% xz^2 | xz | x | yz^2 | yz | y | z^3 | z^2 | z | 1

A = [detF; AE_11; AE_12; AE_13; AE_21; AE_22; AE_23; AE_31; AE_32; AE_33];
A = A(:,[1,2,4,5,6,11,8,12,10,14,7,15,17,9,16,18,3,13,19,20]);

% Gauss Jordan elimination (partial pivoting after)
A_el = gj_elim_pp(A);

% Subtraction and forming matrix B
k_row = partial_subtrc(A_el(5,11:20), A_el(6,11:20));
l_row = partial_subtrc(A_el(7,11:20), A_el(8,11:20));
m_row = partial_subtrc(A_el(9,11:20), A_el(10,11:20));

B_11 = k_row(1,1:4);
B_12 = k_row(1,5:8);
B_13 = k_row(1,9:13);
B_21 = l_row(1,1:4);
B_22 = l_row(1,5:8);
B_23 = l_row(1,9:13);
B_31 = m_row(1,1:4);
B_32 = m_row(1,5:8);
B_33 = m_row(1,9:13);

p_1 = pz4pz3(B_23, B_12) - pz4pz3(B_13, B_22);
p_2 = pz4pz3(B_13, B_21) - pz4pz3(B_23, B_11);
p_3 = pz3pz3(B_11, B_22) - pz3pz3(B_12, B_21);

n_row = pz7pz3(p_1, B_31) + pz7pz3(p_2, B_32) + pz6pz4(p_3, B_33);

%Extracting roots from n_row using companion matrix eigen values
n_row_scaled = n_row/n_row(1);

e_val = eig([-n_row_scaled(2:end);
            eye(9), zeros(9,1)]);
        

m = 0;
for n = 1:10
    if ~isreal(e_val(n))
        continue
    end
    
    m = m + 1;
end

R_all = cell(m,1);
t_all = cell(m,1);
E_all = cell(m,1);
Eo_all = cell(m,1);

m = 1;
for n = 1:10
    if ~isreal(e_val(n))
        continue
    end
    z = e_val(n);

    %Backsubstition
    p_z6 = [z^6; z^5; z^4; z^3; z^2; z; 1];
    p_z7 = [z^7; p_z6];

    x = (p_1*p_z7)/(p_3*p_z6);
    y = (p_2*p_z7)/(p_3*p_z6);

    Eo = x*Xmat + y*Ymat + z*Zmat + Wmat;
    Eo_all{m} = Eo;
    [U,~,V] = svd(Eo);


    E = U*diag([1 1 0])*V';
    E_all{m} = E;

    %stop here if nothing else is required to be computed
    if nargout < 2
        m = m + 1;
        continue
    end
    
    %check determinan signs
    if(det(U) < 0)
        U(:,3) = -U(:,3);
    end

    if (det(V) < 0)
        V(:,3) = -V(:,3);
    end

    %Extracting R and t from E 
    D = [0  1   0;
         -1 0   0;
         0  0   1];

    q_1 = q1(:,1);
    q_2 = q2(:,1);


    for n = 1:4
        switch(n)
            case 1
                t = U(:,3);
                R = U*D*V';
            case 2
                t = -U(:,3);
                R = U*D*V';
            case 3
                t = U(:,3);
                R = U*D'*V';
            case 4
                t = -U(:,3);
                R = U*D'*V';
        end

        %Cheirality (points in front of the camera) constraint assuming perfect
        %point correspondence
        a = E'*q_2;
        b = cross_vec3(q_1, [a(1:2); 0]);
        c = cross_vec3(q_2, diag([1 1 0])*E*q_1);
        d = cross_vec3(a, b);

        P = [R t];
        C = P'*c;
        Q = [d*C(4); -d(1:3)'*C(1:3)];

        %Cheirality test

        %behind the 1st camera
        if (Q(3)*Q(4) < 0)    
            continue
        end

        %behind the 2nd camera
        c_2 = P*Q;
        if (c_2(3)*Q(4) < 0)
            continue
        end

        R_all{m} = R;
        t_all{m} = t;
        break
    end
    m = m + 1;
end

end

function out = cross_vec3(u, v)
%CROSS_VEC Function to compute the cross product of two 3D column vectors.
%The default MATLAB implementation is simply too slow. 

out = [ u(2)*v(3) - u(3)*v(2);
        u(3)*v(1) - u(1)*v(3);
        u(1)*v(2) - u(2)*v(1)];
end

function po = pz6pz4(p1, p2)
%PZ4PZ3 Function responsible for multiplying a 6th order z polynomial p1
%   by a 4th order z polynomial p2
%   p1 - Is a row vector arranged like: z6 | z5 | z4 | z3 | z2 | z | 1
%   p2 - Is a row vector arranged like: z4 | z3 | z2 | z | 1
%   po - Is a row vector arranged like: 
%       z10 | z9 | z8 | z7 | z6 | z5 | z4 | z3 | z2 | z | 1

po = [  p1(1)*p2(1), ... z10
        p1(2)*p2(1) + p1(1)*p2(2), ... z9
        p1(3)*p2(1) + p1(2)*p2(2) + p1(1)*p2(3), ... z8
        p1(4)*p2(1) + p1(3)*p2(2) + p1(2)*p2(3) + p1(1)*p2(4), ... z7
        p1(5)*p2(1) + p1(4)*p2(2) + p1(3)*p2(3) + p1(2)*p2(4) + p1(1)*p2(5), ... z6
        p1(6)*p2(1) + p1(5)*p2(2) + p1(4)*p2(3) + p1(3)*p2(4) + p1(2)*p2(5), ... z5
        p1(7)*p2(1) + p1(6)*p2(2) + p1(5)*p2(3) + p1(4)*p2(4) + p1(3)*p2(5), ... z4
        p1(7)*p2(2) + p1(6)*p2(3) + p1(5)*p2(4) + p1(4)*p2(5), ... z3
        p1(7)*p2(3) + p1(6)*p2(4) + p1(5)*p2(5), ... z2
        p1(7)*p2(4) + p1(6)*p2(5), ... z
        p1(7)*p2(5)];  % 1
end

function po = pz7pz3(p1, p2)
%PZ4PZ3 Function responsible for multiplying a 7th order z polynomial p1
%   by a 3rd order z polynomial p2
%   p1 - Is a row vector arranged like: z7 | z6 | z5 | z4 | z3 | z2 | z | 1
%   p2 - Is a row vector arranged like: z3 | z2 | z | 1
%   po - Is a row vector arranged like: 
%       z10 | z9 | z8 | z7 | z6 | z5 | z4 | z3 | z2 | z | 1

po = [  p1(1)*p2(1), ... z10
        p1(2)*p2(1) + p1(1)*p2(2), ... z9
        p1(3)*p2(1) + p1(2)*p2(2) + p1(1)*p2(3), ... z8
        p1(4)*p2(1) + p1(3)*p2(2) + p1(2)*p2(3) + p1(1)*p2(4), ... z7
        p1(5)*p2(1) + p1(4)*p2(2) + p1(3)*p2(3) + p1(2)*p2(4), ... z6
        p1(6)*p2(1) + p1(5)*p2(2) + p1(4)*p2(3) + p1(3)*p2(4), ... z5
        p1(7)*p2(1) + p1(6)*p2(2) + p1(5)*p2(3) + p1(4)*p2(4), ... z4
        p1(8)*p2(1) + p1(7)*p2(2) + p1(6)*p2(3) + p1(5)*p2(4), ... z3
        p1(8)*p2(2) + p1(7)*p2(3) + p1(6)*p2(4), ... z2
        p1(8)*p2(3) + p1(7)*p2(4), ... z
        p1(8)*p2(4)];  % 1
end

function po = pz4pz3(p1, p2)
%PZ4PZ3 Function responsible for multiplying a 4th order z polynomial p1
%   by a 3rd order z polynomial p2
%   p1 - Is a row vector arranged like: z4 | z3 | z2 | z | 1
%   p2 - Is a row vector arranged like: z3 | z2 | z | 1
%   po - Is a row vector arranged like: z7 | z6 | z5 | z4 | z3 | z2 | z | 1

po = [  p1(1)*p2(1), ... z7
        p1(2)*p2(1) + p1(1)*p2(2), ... z6
        p1(3)*p2(1) + p1(2)*p2(2) + p1(1)*p2(3), ... z5
        p1(4)*p2(1) + p1(3)*p2(2) + p1(2)*p2(3) + p1(1)*p2(4), ... z4
        p1(5)*p2(1) + p1(4)*p2(2) + p1(3)*p2(3) + p1(2)*p2(4), ... z3
        p1(5)*p2(2) + p1(4)*p2(3) + p1(3)*p2(4), ... z2
        p1(5)*p2(3) + p1(4)*p2(4), ... z
        p1(5)*p2(4)];  % 1
end

function po = pz3pz3(p1, p2)
%PZ3PZ3 Function responsible for multiplying two 3rd order z polynomial
%   p1, p2 - Are row vector arranged like: z3 | z2 | z | 1
%   po - Is a row vector arranged like: z6 | z5 | z4 | z3 | z2 | z | 1

po = [  p1(1)*p2(1), ... z6
        p1(1)*p2(2) + p1(2)*p2(1), ...  z5
        p1(1)*p2(3) + p1(2)*p2(2) + p1(3)*p2(1), ... z4
        p1(1)*p2(4) + p1(2)*p2(3) + p1(3)*p2(2) + p1(4)*p2(1), ... z3
        p1(2)*p2(4) + p1(3)*p2(3) + p1(4)*p2(2), ... z2
        p1(3)*p2(4) + p1(4)*p2(3), ... z
        p1(4)*p2(4)];  % 1
end

function po = partial_subtrc(p1, p2)
%PARTIAL_SUBTRC Given polinomials p1 and p2 substract them according to the
%following expression: p1 - z*p2
% p1, p2 - are row vectors with the following arrangement
%   xz^2 | xz | x | yz^2 | yz | y | z3 | z2 | z | 1
% po - is a row vector with the following arragement
%   xz3 | xz2 | xz | x | yz3 | yz2 | yz | y | z4 | z3 | z2 | z | 1

po = [-p2(1), p1(1) - p2(2), p1(2) - p2(3), p1(3), ...
      -p2(4), p1(4) - p2(5), p1(5) - p2(6), p1(6), ...
      -p2(7), p1(7) - p2(8), p1(8) - p2(9), p1(9) - p2(10), p1(10)]; 
end

function B = gj_elim_pp(A)
%GJ_ELIM_PP Given Matriz A we perform partial pivoting as per specified in 

[~,U] = lu(A);

B = zeros(10,20);
B(1:4,:) = U(1:4,:);

%Back substitution
B(10,:) = U(10,:)/U(10,10);
B(9,:) = (U(9,:) - U(9,10)*B(10,:))/U(9,9);
B(8,:) = (U(8,:) - U(8,9)*B(9,:) - U(8,10)*B(10,:))/U(8,8);
B(7,:) = (U(7,:) - U(7,8)*B(8,:) - U(7,9)*B(9,:) - U(7,10)*B(10,:))/U(7,7);
B(6,:) = (U(6,:) - U(6,7)*B(7,:) - U(6,8)*B(8,:) - U(6,9)*B(9,:) ...
                 - U(6,10)*B(10,:))/U(6,6);
B(5,:) = (U(5,:) - U(5,6)*B(6,:) - U(5,7)*B(7,:) - U(5,8)*B(8,:) ...
                 - U(5,9)*B(9,:) - U(5,10)*B(10,:))/U(5,5);
end

function pout = p1p1(p1, p2)
%P1P1 Given two first order polynomials with the structure [a_x,
%a_y, a_z, a_w] e returns the second order polynomial x,y,z with the
%structure: pout = [a_x2, a_y2, a_z2, a_xy, a_xz, a_yz, a_x, a_y, a_z, a]

pout = [p1(1)*p2(1), ...                %x2
        p1(2)*p2(2), ...                %y2
        p1(3)*p2(3), ...                %z2
        p1(1)*p2(2) + p1(2)*p2(1), ...  %xy
        p1(1)*p2(3) + p1(3)*p2(1), ...  %xz
        p1(2)*p2(3) + p1(3)*p2(2), ...  %yz
        p1(1)*p2(4) + p1(4)*p2(1), ...  %x
        p1(2)*p2(4) + p1(4)*p2(2), ...  %y
        p1(3)*p2(4) + p1(4)*p2(3), ...  %z
        p1(4)*p2(4)];                   %1
end

function pout = p2p1(p1,p2)
%P2P1 Given two polynomials, p1 of order 2 and p2 of order 1, with
%unknowns x, y and z, return their product. An order 3 polynomial with
%structure shown below

pout = [p1(1)*p2(1), ...                                %x3
        p1(2)*p2(2), ...                                %y3
        p1(3)*p2(3), ...                                %z3
        p1(1)*p2(2) + p1(4)*p2(1), ...                  %x2y
        p1(2)*p2(1) + p1(4)*p2(2), ...                  %xy2
        p1(1)*p2(3) + p1(5)*p2(1), ...                  %x2z
        p1(3)*p2(1) + p1(5)*p2(3), ...                  %xz2
        p1(2)*p2(3) + p1(6)*p2(2), ...                  %y2z
        p1(3)*p2(2) + p1(6)*p2(3), ...                  %yz2
        p1(4)*p2(3) + p1(5)*p2(2) + p1(6)*p2(1), ...    %xyz
        p1(1)*p2(4) + p1(7)*p2(1), ...                  %x2
        p1(2)*p2(4) + p1(8)*p2(2), ...                  %y2
        p1(3)*p2(4) + p1(9)*p2(3), ...                  %z2
        p1(4)*p2(4) + p1(7)*p2(2) + p1(8)*p2(1), ...    %xy
        p1(5)*p2(4) + p1(7)*p2(3) + p1(9)*p2(1), ...    %xz
        p1(6)*p2(4) + p1(8)*p2(3) + p1(9)*p2(2), ...    %yz
        p1(7)*p2(4) + p1(10)*p2(1), ...                 %x
        p1(8)*p2(4) + p1(10)*p2(2), ...                 %y
        p1(9)*p2(4) + p1(10)*p2(3), ...                 %z
        p1(10)*p2(4)];                                  %1
end
