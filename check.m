function imP = check( rectp )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
rectp= im2single(rectp);
rectp = rectp + rand(size(rectp)) * 1e-10;
[cxy, c45, Ix, Iy] = ...
    vision.internal.calibration.checkerboard.secondDerivCornerMetric(rectp, 2);
[Ix2, Iy2, Ixy] = computeJacobianEntries(Ix, Iy);
points0 = vision.internal.calibration.checkerboard.find_peaks(cxy, 0.2);
scores0 = cxy(sub2ind(size(cxy), points0(:, 2), points0(:, 1)));
board0 = growCheckerboard(points0, scores0, Ix2, Iy2, Ixy, 0);

points45 = vision.internal.calibration.checkerboard.find_peaks(c45, 0.2);
scores45 = c45(sub2ind(size(c45), points45(:, 2), points45(:, 1)));
board45 = growCheckerboard(points45, scores45, Ix2, Iy2, Ixy, pi/4);

if  board0.Energy < board45.Energy
    board0 = orient(board0, rectp);
    [points, ~] = toPoints(board0);
    points = vision.internal.calibration.checkerboard.subPixelLocation(cxy, points);
else
    board45 = orient(board45, rectp);
    [points, ~] = toPoints(board45);
    points = vision.internal.calibration.checkerboard.subPixelLocation(c45, points);
end

imP=points;
end


%--------------------------------------------------------------------------
function [Ix2, Iy2, Ixy] = computeJacobianEntries(Ix, Iy)

Ix2 = Ix .^ 2;
Iy2 = Iy .^ 2;
Ixy = Ix .* Iy;

G = fspecial('gaussian', 2, 15);

Ix2 = imfilter(Ix2, G);
Iy2 = imfilter(Iy2, G);
Ixy = imfilter(Ixy, G);

end
%--------------------------------------------------------------------------
function board = growCheckerboard(points, scores, Ix2, Iy2, Ixy, theta)

% Exit immediately if no corner points were found
if isempty(scores)
    if isempty(coder.target)
        board = struct('BoardIdx', zeros(3), 'BoardCoords', zeros(3,3,3), ...
            'Energy', Inf, 'isValid', 0);
    else
        board = vision.internal.calibration.checkerboard.Checkerboard;
    end
    return;
end

seedIdx = 1:size(points, 1);

% only use corners with high scores as seeds to reduce computation
if mean(scores) < max(scores) % need this check in case all scores are the same
    seedIdx = seedIdx(scores >= mean(scores));
end

[~, sortedIdx] = sort(scores(seedIdx), 'descend');
seedIdx = seedIdx(sortedIdx);

angleThreshold = 3 * pi / 16;

if isempty(coder.target)
    v1_matrix = [];
    v2_matrix = [];
    seedIdx_matrix = [];
    
    for i = seedIdx
        [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, round(points(i, :)));
        alpha1 = abs(atan2(v1(2), v1(1)));
        alpha2 = abs(atan2(v2(2), v2(1)));
        if abs(abs(alpha1 - pi) - theta) > angleThreshold && ...
                abs(abs(alpha2 - pi) - theta) > angleThreshold
            continue;
        else
            v1_matrix = [v1_matrix;v1]; %#ok<AGROW>
            v2_matrix = [v2_matrix;v2]; %#ok<AGROW>
            seedIdx_matrix = [seedIdx_matrix;i]; %#ok<AGROW>
        end
    end
    
    board = visionInitializeAndExpandCheckerboard(seedIdx_matrix,single(points),v1_matrix,v2_matrix);
else
    previousBoard = vision.internal.calibration.checkerboard.Checkerboard;
    currentBoard = vision.internal.calibration.checkerboard.Checkerboard;
    for i = 1:numel(seedIdx)
        [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, round(points(seedIdx(i), :)));
        alpha1 = abs(atan2(v1(2), v1(1)));
        alpha2 = abs(atan2(v2(2), v2(1)));
        if abs(abs(alpha1 - pi) - theta) > angleThreshold && ...
                abs(abs(alpha2 - pi) - theta) > angleThreshold
            continue;
        end
        
        currentBoard.initialize(seedIdx(i), points, v1, v2);
        expandBoardFully(currentBoard);
        if currentBoard.Energy < previousBoard.Energy            
            tmpBoard = previousBoard;
            previousBoard = currentBoard;
            currentBoard = tmpBoard;
        end
    end
    board = previousBoard;
end
end

%--------------------------------------------------------------------------
function [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, p)
% The orientation vectors are the eigen vectors of the 
% structure tensor:
% [Ix^2  Ixy ]
% [Ixy   Iy^2]

a = Ix2(p(2), p(1));
b = Ixy(p(2), p(1));
c = Iy2(p(2), p(1));

% % Computing eigenvectors "by hand", because the eig() function behaves
% % differently in codegen.
% % Since the matrix is positive-semidefinite, its eigenvectors are
% % orthogonal. Compute the first eigenvector, then swap its elements and
% % negate the y-component to make the second one.
sm = a + c;
df = a - c;
adf = abs(df);
tb = b + b;
ab = abs(tb);

if adf > ab
    rt = adf * sqrt(1 + (ab/adf)^2);
elseif adf < ab
    rt = ab * sqrt(1 + (adf/ab)^2);
else
    rt = ab * sqrt(2);
end

if sm < 0
    sgn1 = -1;
else
    sgn1 = 1;
end

if df > 0
    cs = df + rt;
    sgn2 = 1;
else
    cs = df - rt;
    sgn2 = -1;
end

acs = abs(cs);
if acs > ab
    ct = -tb / cs;
    sn1 = 1 / sqrt(1 + ct * ct);
    cs1 = ct * sn1;
else
    if ab == single(0)
        cs1 = single(1);
        sn1 = single(0);
    else
        tn = -cs / tb;
        cs1 = 1 / sqrt(1 + tn * tn);
        sn1 = tn * cs1;
    end
end
if sgn1 == sgn2
    tn = cs1;
    cs1 = -sn1;
    sn1 = tn;
end

v1 = [-sn1, cs1];
v2 = [cs1, sn1];
end

%--------------------------------------------------------------------------

function [points, boardSize] = toPoints(this)
% returns the points as an Mx2 matrix of x,y coordinates, and
% the size of the board

if any(this.BoardIdx(:) == 0)
    points = [];
    boardSize = [0 0];
    return;
end

numPoints = size(this.BoardCoords, 1) * size(this.BoardCoords, 2);
points = zeros(numPoints, 2);
x = this.BoardCoords(:, :, 1)';
points(:, 1) = x(:);
y = this.BoardCoords(:, :, 2)';
points(:, 2) = y(:);
boardSize = [size(this.BoardCoords, 2)+1, size(this.BoardCoords, 1)+1];
end

    
%--------------------------------------------------------------------------
function board = orient(board, I)

    if ~isinf(board.Energy)
        % orient the board so that the long side is the X-axis
        if size(board.BoardCoords, 1) < size(board.BoardCoords, 2)
            board = rot90_checkerboard(board, 1);
        end

        % try to orient the board so that (0,0) is on a black square
        if ~isUpperLeftBlack(board, I);
            board = rot90_checkerboard(board, 2);
        end

        % if both sides are odd or both sides are even, make sure
        % that (0,0) is at the upper-left corner.
        if ~xor(mod(size(board.BoardCoords, 1), 2) == 0,... 
            mod(size(board.BoardCoords, 2), 2) == 0)
            if any(board.BoardCoords(1,1,:) > board.BoardCoords(end, end, :))
                board = rot90_checkerboard(board, 2);
            end
        end
    end
end

%--------------------------------------------------------------------------
function board = rot90_checkerboard(board, k)
board.BoardIdx = rot90(board.BoardIdx, k);
newBoardCoords1 = rot90(board.BoardCoords(:,:,1), k);
newBoardCoords2 = rot90(board.BoardCoords(:,:,2), k);
board.BoardCoords = cat(3, newBoardCoords1, newBoardCoords2);
end

%--------------------------------------------------------------------------

function tf = isUpperLeftBlack(this, I)
% check if the upper-left square of the board is black

% create a mask for the upper-left square
upperLeftPolyX = [this.BoardCoords(1, 1, 1), ...
    this.BoardCoords(1, 2, 1), this.BoardCoords(2, 2, 1), ...
    this.BoardCoords(2, 1, 1)];
upperLeftPolyY = [this.BoardCoords(1, 1, 2), ...
    this.BoardCoords(1, 2, 2), this.BoardCoords(2, 2, 2), ...
    this.BoardCoords(2, 1, 2)];
upperLeftMask = poly2RectMask(upperLeftPolyX, upperLeftPolyY, ...
    size(I, 1), size(I, 2));

% create a mask for the square to the right of it
nextSquarePolyX = [this.BoardCoords(1, 2, 1), ...
    this.BoardCoords(1, 3, 1), this.BoardCoords(2, 3, 1), ...
    this.BoardCoords(2, 2, 1)];
nextSquarePolyY = [this.BoardCoords(1, 2, 2), ...
    this.BoardCoords(1, 3, 2), this.BoardCoords(2, 3, 2), ...
    this.BoardCoords(2, 2, 2)];
nextSquareMask = poly2RectMask(nextSquarePolyX, nextSquarePolyY,...
    size(I, 1), size(I, 2));

% check if the first square is darker than the second
tf = mean(mean(I(upperLeftMask))) < mean(mean(I(nextSquareMask)));
end

%--------------------------------------------------------------------------
function mask = poly2RectMask(X, Y, height, width)
X = sort(X);
Y = sort(Y);
x1 = X(2)+1;
x2 = X(3)-1;
y1 = Y(2)+1;
y2 = Y(3)-1;
mask = false(height, width);
mask(y1:y2, x1:x2) = true;
end