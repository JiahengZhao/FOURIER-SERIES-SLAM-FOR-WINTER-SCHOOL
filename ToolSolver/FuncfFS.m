function [fval_odom, fval_center, fval_fs, refsID] = ...
    FuncfFS(feaOccurredID,Xstate,Zstate)
% FUNCFFS calculate fourier series
% Input:    feaOccurredID ---- data association index
%                            Xstate ---- state variable matrix.
%                            Zstate ---- observation state matrix
% Output: fval_odom ---- odometry cost value. Eq. (10)
%              fval_center ---- center cost value. Eq. (10)
%                     fval_fs ---- fourier series cost value. Eq. (10)

% Author: Jiaheng Zhao

ZstateCenter = Zstate.center;
ZstatePts = Zstate.pts;
ZstateOdom = Zstate.odom;
fsN_f = Zstate.fsN;
if isfield(Zstate, 'fsN_rect')
    fsN_rect = Zstate.fsN_rect;
else
    fsN_rect = 0;
end
if isfield(Zstate, 'fsN_border')
    fsN_border = Zstate.fsN_border;
else
    fsN_border = 0;
end
Pose_state = Xstate(Xstate(:,2)==1,:);
step = size(Xstate(Xstate(:,2)==1),1)/3;
nFea = size(feaOccurredID,1);
fval_odom=[];
fval_center=[];
fval_fs = [];
saveNocenterID = [];
nF = size(find(feaOccurredID(:,2)<10),1);
nB = size(find(feaOccurredID(:,2)>9 & feaOccurredID(:,2) < 100),1);
for i=1:step % solve value by pose number.
    PoseState_ThisStep = Pose_state(3*i-2:3*i,1);
    phi = PoseState_ThisStep(3);
    R = theta2R(phi);
    
    % Calculate odometry part
    if i==1
        fval_tmp_1 =  ZstateOdom(3*i-2:3*i,1) - PoseState_ThisStep;
        fval_tmp_1(3)=wrapToPi(fval_tmp_1(3));
        fval_odom = [fval_odom;fval_tmp_1];
    else
        PoseState_LastStep = Pose_state(3*i-5:3*i-3,1);
        phi_last = PoseState_LastStep(3);
        R_last = theta2R(phi_last);
        fval_tmp_1 =  ZstateOdom(3*i-2:3*i,1) -...
            [R_last'*(PoseState_ThisStep(1:2)-PoseState_LastStep(1:2));phi-phi_last ];
        if ~isreal(fval_tmp_1(3))
            fval_tmp_1(3)
        end
        fval_tmp_1(3)=wrapToPi(fval_tmp_1(3));
        fval_odom = [fval_odom;fval_tmp_1];
    end
    
    % transfer all points to global frame
    transFun = @(x) R*x + PoseState_ThisStep(1:2);
    tmpZstatePts(i,:) = cellfun(transFun,ZstatePts(i,:),'UniformOutput',false);
    
    % center
    thisZstateCenter = ZstateCenter(ZstateCenter(:,4)==i,:);
    thisSeeFeaid = unique(thisZstateCenter(:,3));
    for ss = 1:nFea
        newsid = ss;
        oldsid = feaOccurredID(feaOccurredID(:,1)==newsid,2);
        [a,b]=ismember(oldsid,thisSeeFeaid);
        if a
            saveNocenterID= [saveNocenterID; [i, 0]]; % zero
            % fs center
            thisXstateFS = Xstate(Xstate(:,2)==2 & Xstate(:,3) == newsid);
            thisCenter = thisZstateCenter(thisZstateCenter(:,3)==oldsid,1);
            fval_center = [fval_center; R*thisCenter+PoseState_ThisStep(1:2) - thisXstateFS(1:2)];
        else
            saveNocenterID= [saveNocenterID; [i, newsid]];
            continue;
        end
    end
end

%% This is cost function part for Fourier series, you need to complete the remaining equations.
numAllPt = cellfun('size',tmpZstatePts,2); numAllPt = [zeros(1,size(numAllPt,2)) ; numAllPt(1:end-1,:)];
for si1 = 2:size(numAllPt,1)
    numAllPt(si1,:) = numAllPt(si1,:) + numAllPt(si1-1,:);
end
numAllPt = num2cell(numAllPt);
numAllPt2 = cellfun(@(x) myfun(x),tmpZstatePts,'UniformOutput',false);
numAllPt3 = cellfun(@plus,numAllPt,numAllPt2,'UniformOutput',false);
refsID = cell(size(numAllPt));

% starts from the first feature.
for ss = 1:nFea
    oldsid = feaOccurredID(ss,2);
    if oldsid > 9 && oldsid < 100                   % Pre-defined border ID
        oldsid = oldsid/10 + nF;
        fsN = fsN_border;
    elseif oldsid > 99                                       % Pre-defined rectangle features ID
        fsN = fsN_rect;
        oldsid = oldsid/100 + nF + nB;
    else                                                            % Pre-defined other features ID
        fsN = fsN_f;
    end
    
    % this feature's state
    thisXstateFS = Xstate(Xstate(:,2)==2 & Xstate(:,3) == ss);
    % points of this feature
    thispt = cat(2,tmpZstatePts{:,oldsid}); % use all points although no center
    % decentralized points of this feature
    decentralizedPts = thispt-thisXstateFS(1:2);
    % theta and r ->Eq. (3)
    thisr = sqrt(decentralizedPts(1,:).^2+decentralizedPts(2,:).^2);
    thistheta = atan2(decentralizedPts(2,:),decentralizedPts(1,:));
    [thistheta, Isave] = sort(thistheta+pi); thistheta = thistheta-pi;
    thisr = thisr(Isave);
    
    refsID(:,oldsid) = cellfun(@(x) myfun2(x,Isave),numAllPt3(:,oldsid),'UniformOutput',false);
    
    %-------------------- Write your answer here ------------------------------%
    an = thisXstateFS(3:fsN+3,1);
    bn = thisXstateFS(fsN+4:end,1); bn = [0;bn];
    dtheta = 0;
    for j = 1:fsN+1
        dtheta = dtheta + []; % Eq. (2);
    end
    eq3 =  []; % Eq. (3); n by 1;
    fval_fs = [fval_fs; eq3];
    %---------------------------------------------------------------------------%
end
end

function a = myfun(A)
a = [1:size(A,2)]';
end

function a = myfun2(A,Isave)
[~,a]=ismember(A,Isave);
end
