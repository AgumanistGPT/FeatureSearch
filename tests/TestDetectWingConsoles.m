function tests = TestDetectWingConsoles
%TESTDETECTWINGCONSOLES Integration tests for DetectWingConsoles.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
addpath(fullfile(repoRoot, 'src'));
testCase.TestData.repoRoot = repoRoot;
end

function testBaselineRealData(testCase)
S = load(fullfile(testCase.TestData.repoRoot, 'data', 'RealData.mat'));
consoles = DetectWingConsoles(S.segments, S.contour, S.curvature, S.tangent_angles);
verifyEqual(testCase, numel(consoles), 4);
end

function testRandomLinePermutations(testCase)
S = load(fullfile(testCase.TestData.repoRoot, 'data', 'RealData.mat'));
lineIdx = find(cellfun(@(s) strcmp(s.type, 'line'), S.segments));
nLines = numel(lineIdx);
nRandomPermutations = 100;
totalPossiblePermutations = exp(gammaln(double(nLines) + 1)); % nLines!

if nLines < 2
    verifyFail(testCase, 'Not enough line segments for permutation test.');
end
verifyGreaterThanOrEqual(testCase, totalPossiblePermutations, nRandomPermutations, ...
    'Requested random permutation count exceeds total possible permutations.');

rng(42);
for k = 1:nRandomPermutations
    shuffledSegments = S.segments;
    permutedLineOrder = lineIdx(randperm(nLines));
    shuffledSegments(lineIdx) = S.segments(permutedLineOrder);

    consoles = DetectWingConsoles(shuffledSegments, S.contour, S.curvature, S.tangent_angles);
    verifyEqual(testCase, numel(consoles), 4, sprintf('Permutation #%d failed', k));
end
end
