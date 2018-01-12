function [R, T] = ICP(p1, p2, method, varargin)
% Runs Iterative closest point on 2 point sets p1 and p2
% ------------------ Inputs -----------------------------------------
% p1 and p2 and 3XN
% p1 is the reference point set (destination) and
% p2 is the query point set (source)
% ------------------ Outputs ----------------------------------------
% R: Rotation
% T: Translation
% p1 =  R*p2 + T;
% Code by: Nitin J. Sanket (nitinsan@seas.upenn.edu)

if(nargin==2)
    % Default is point to point ICP
    method = 'PointToPoint';
end

if(strcmp(method,'PointToPoint'))
    % Point to point ICP (slow but default)
    % Solve the optimization problem:
    % Find (R,T) such that min sum(p1- (R*p2+T))^2
    % First compute the center of mass of each point set
    p1Mean = mean(p1,2);
    p2Mean = mean(p2,2);
    
    % Compute q, which is the mean subtracted values
    q1 = bsxfun(@minus, p1, p1Mean);
    q2 = bsxfun(@minus, p2, p2Mean);
    
    % Compute H matrix
    H = q1*q2';
    
    % Compute SVD
    [U, ~, V] = svd(H);
    
    % Compute X
    X = V*U';
    R = X;
    
    % disp(['Determinant is ',num2str(det(R))]);
    
    % if(det(X)==1)
    %     R = X;
    %     disp('R found....');
    % else
    %     if(det(X)==-1)
    %         R = [];
    %         T = [];
    %         disp('R not found....');
    %         return;
    %     end
    % end
    
    T = p2Mean-R*p1Mean;
    return;
end

if(strcmp(method,'PointToPlane'))
     % Solve the optimization problem:
    % Find (R,T) such that min sum((p1 - (R*p2+T)).Normals)^2
    % Get the normals corresponding to p1 (destination/reference)
    Normals = varargin{1}; % Size 3XN
    Diff = p1-p2; % destination - source
    % Solving Ax-b = 0
    b = transpose(sum(Diff.*Normals,1)); % Take dot product with all the normals, NX1
    A = [Normals(3,:).*p2(2,:)-Normals(2,:).*p2(3,:);
         Normals(1,:).*p2(3,:)-Normals(3,:).*p2(1,:);
         Normals(2,:).*p2(1,:)-Normals(1,:).*p2(2,:);
         Normals]'; % NX6
    % x = [alpha, beta, gamma, tx, ty, tz]'
    x = A\b;
    
    cosAlpha = cos(x(1));
    sinAlpha = sin(x(1));
    cosBeta = cos(x(2));
    sinBeta = sin(x(2));
    cosGamma = cos(x(3));
    sinGamma = sin(x(3));
        
    R = [cosGamma*cosBeta, -sinGamma*cosAlpha+cosGamma*sinBeta*sinAlpha, sinGamma*sinAlpha+cosGamma*sinBeta*cosAlpha;
         sinGamma*cosBeta, cosGamma*cosAlpha+sinGamma*sinBeta*sinAlpha, -cosGamma*sinAlpha+sinGamma*sinBeta*cosAlpha;
         -sinBeta,          cosBeta*sinAlpha,                            cosBeta*cosAlpha];
    T = x(4:end);
    return;
end
end