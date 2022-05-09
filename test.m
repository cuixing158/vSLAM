features = [0,0,1,0,1,1;
            1,0,1,0,1,1;
            1,0,1,0,1,1;
            0,0,1,0,1,1;
            1,1,1,0,0,1;
            1,1,1,0,0,0];
index = computeDistinctiveDescriptors(features)% index为最有代表性的特征索引

%------------------------------------------------------------------
function index = computeDistinctiveDescriptors(features)
%computeDistinctiveDescriptors Find the distinctive discriptor

if size(features, 1) < 3
    index       = size(features, 1);
else
    scores      = helperHammingDistance(features, features);
    [~, index]  = min(sum(scores, 2));
end
end

function scores = helperHammingDistance(features1, features2)
%helperHammingDistance compute hamming distance between two groups of
%   binary feature vectors.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

persistent lookupTable; % lookup table for counting bits

N1 = size(features1, 1);
N2 = size(features2, 1);

scores = zeros(N1, N2);

if isempty(lookupTable)
    lookupTable = zeros(256, 1);
    for i = 0:255
        lookupTable(i+1) = sum(dec2bin(i)-'0');
    end
end

for r = 1:N1
    for c = 1:N2
        temp = bitxor(features1(r, :), features2(c, :));
        idx = double(temp) + 1; % cast needed to avoid integer math
        scores(r,c) = sum(lookupTable(idx));
    end
end

end