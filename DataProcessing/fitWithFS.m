function [X, center, newpt] = fitWithFS( N, Xi, Yi, center_in, isAdd)
% FITWITHFS fit the feature with fourier series.
% Param:    N ---- the number of Fourier series.
%                Xi ---- x coordinate of the input points.
%                Yi ---- y coordinate of the input points.
%         center ---- initial cneter of the input points.
%          isAdd ---- whether to complement points.
% Output:   X ---- Fourier series: [a0; a1; ...; an; b1; b2; ...; bn].
%         center ---- Fitted center.
%         newpt ---- Fitted points with respect to the input points' theta.
% Author: Jiaheng Zhao

if isempty(center_in)
    % Fit the feature with circle to obtain the center.
    [~,~,initialc]= fitWithCircle( Xi, Yi);
    center = initialc;
else
    center = center_in;
end

M = length(Xi);
if size(Xi,1) > size(Xi,2)
    pt = [Xi';Yi'];
else
    pt = [Xi;Yi];
end

% Decentralization
pt_noCenter = pt  - center;

% Order theta_i and r_i, according to Eq.(3)
theta = atan2(pt_noCenter(2,:),pt_noCenter(1,:));
theta = theta .* (theta >= 0) + (theta + 2 * pi) .* (theta < 0);
r = sqrt(pt_noCenter(2,:).^2 + pt_noCenter(1,:).^2);
[theta,od] = sort(theta);
r=r(od);

% find non-observed area
step = 0;

if isAdd % if complement
    % index of breaking point. may be more than 1.
    bk = find(abs(diff(theta).*180/pi)>15);
    res_deg = trimmean(diff(theta),60);
    % theta at the breaking point
    theta_1 = theta(bk);
    r_1 = r(bk);
    theta_2 = theta(bk+1);
    r_2 = r(bk+1);
    theta_i = cell(1,length(theta_1));
    r_i = theta_i;
    
    % interpolate angles between theta_1 and theta_2
    if ~isempty(theta_1)
        for index = 1 : length(theta_1)
            if theta_1(index) < theta_2(index)
                step = res_deg;
            else
                step = -res_deg;
            end
            tmp = wrapTo2Pi(theta_1(index) : step : theta_2(index));
            tmp(1) = [];
            if ~isempty(tmp)
                if tmp(end) == theta_2(index)
                    tmp(end) = [];
                end
            end
            theta_i{1,index} = tmp;
            % interpolate ranges which is linear to the r_1 and r_2.
            rstep = (r_2 - r_1)/(length(theta_i{1,index})-1);
            r_i{1,index} = r_1 : rstep: r_2;
        end % end for
    else
        step = res_deg;
    end
    % complemented points: end->2pi
    theta_b = wrapTo2Pi(theta(end) : step : (theta(1)+2*pi));
    rstep =  (r(1) - r(end))/(length(theta_b)-1);
    r_b =  r(end) : rstep: r(1);
    % complemented points
    r_i = cat(2,r_i{:});
    theta_i = cat(2,theta_i{:});
    % all points
    theta_sup = [theta, theta_i, theta_b];
    [theta_sup,od1] = sort(theta_sup);
    r_sup = [r, r_i, r_b];
    r_sup = r_sup(od1);
    M_sup = length(r_sup);
else
    r_sup=r;
    theta_sup = theta;
    M_sup = M;
end

%% Eq. (5) - Eq. (8)  in the paper. You need to complete the empty parts.
% r_sup and theta_sub are the range and bearing of all the points.
S2 = zeros(N,1);
S3 = zeros(N,1);
A = zeros(2*N+1);

%-------------------- Write your answer here (this is an example)----------%

S1 =  sum(r_sup);  %Eq. (6) ----- sum(r_rup) is the implement.

%----------------------------------------------------------------------------%

for p = 0:N
    if p > 0
        %-------------------- Write your answer here ------------------------------%
        
        S2(p) = []; %Eq. (6)
        S3(p) = []; %Eq. (6)
        
        %----------------------------------------------------------------------------%
    end
    for q = 1:N
        %-------------------- Write your answer here ------------------------------%
        if p==0
            A(1,q+1) = []; %Eq. (8)
            A(1,N+q+1) = []; %Eq. (8)
        else
            A(p+1,q+1) = []; %Eq. (8)
            A(p+1,N+q+1) = []; %Eq. (8)
            A(N+p+1,q+1) = []; %Eq. (8)
            A(N+p+1,N+q+1) = []; %Eq. (8)
        end
        %----------------------------------------------------------------------------%
    end
end

A(1,1) = length(theta_sup);
A(2:N+1,1) = A(1,2:N+1)';
A(N+2:end,1) = A(1,N+2:end);

S =[S1; S2; S3];
X = A\S; % X = A^{-1} * S.

an = X(1:N+1,1);
bn = X(N+2:end,1);
bn = [0;bn];
dtheta = zeros(1,M_sup);
for i = 1:N+1
    dtheta = dtheta + an(i).*cos((i-1).*theta_sup) + bn(i).*sin((i-1).*theta_sup);
end
% to point
newpt = [dtheta.*cos(theta_sup); dtheta.*sin(theta_sup)] + center;
end