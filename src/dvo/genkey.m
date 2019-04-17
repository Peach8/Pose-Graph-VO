% gen test key
% 1-2, 2-3, 1-3
KEYS = cell(length(freiburg2)-4,1);

for i = 1:(length(freiburg2)-4)
    KEYS{i}{1}(1) = i;
    KEYS{i}{1}(2) = i+1;
    KEYS{i}{2}(1) = i+1;
    KEYS{i}{2}(2) = i+2;
    KEYS{i}{3}(1) = i+2;
    KEYS{i}{3}(2) = i+3;
    KEYS{i}{4}(1) = i+3;
    KEYS{i}{4}(2) = i+4;
    KEYS{i}{5}(1) = i;
    KEYS{i}{5}(2) = i+2;
    KEYS{i}{6}(1) = i;
    KEYS{i}{6}(2) = i+3;
    KEYS{i}{7}(1) = i;
    KEYS{i}{7}(2) = i+4;
    KEYS{i}{8}(1) = i+1;
    KEYS{i}{8}(2) = i+3;
    KEYS{i}{9}(1) = i+1;
    KEYS{i}{9}(2) = i+4;
    KEYS{i}{10}(1) = i+2;
    KEYS{i}{10}(2) = i+4;
end
