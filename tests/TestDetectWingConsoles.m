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
verifyGreaterThanOrEqual(testCase, numel(consoles), 4);
end

function testRandomLinePermutations(testCase)
S = load(fullfile(testCase.TestData.repoRoot, 'data', 'RealData.mat'));
lineIdx = find(cellfun(@(s) strcmp(s.type, 'line'), S.segments));

if numel(lineIdx) < 2
    verifyFail(testCase, 'Not enough line segments for permutation test.');
end

rng(42);
for k = 1:30
    shuffledSegments = S.segments;
    permutedLineOrder = lineIdx(randperm(numel(lineIdx)));
    shuffledSegments(lineIdx) = S.segments(permutedLineOrder);

    consoles = DetectWingConsoles(shuffledSegments, S.contour, S.curvature, S.tangent_angles);
    verifyGreaterThanOrEqual(testCase, numel(consoles), 4, sprintf('Permutation #%d failed', k));
end
end
